"""drive slope and reverse (backing.py): up a lane forwards, an S backing down to the next; the whole plan backwards for
the body turned round. Both with the body checked whatever the settings say."""

import dataclasses
import hashlib
import math
import os
import struct
import sys

import pytest
from shapely.geometry import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), HERE]

from mowbite_planner import PlannerConfig, PlanningError, plan_coverage  # noqa: E402
from mowbite_planner import backing  # noqa: E402
from mowbite_planner.config import LANE, PERIMETER  # noqa: E402
from mowbite_planner.export import fingerprint, plan_pieces  # noqa: E402
from mowbite_planner.footprint import SPIN_BACK, Footprint  # noqa: E402
from mowbite_planner.mathutil import Pose2D  # noqa: E402
from mowbite_planner.report import check_plan  # noqa: E402
from planrules import check_invariants  # noqa: E402

AREA = [(0.0, 0.0), (3.1, 0.0), (3.0, 2.9), (0.1, 3.0)]
GARDEN = [(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)]
STUMP = [(2.5, 1.6), (3.3, 1.6), (3.3, 2.4), (2.5, 2.4)]
# (asked loose: the body check turned on all the same)
CFG = PlannerConfig(mower_width=0.18, lane_spacing_max=0.13, robot_width=0.41, robot_front=0.43, robot_rear=0.18,
                    blade_ahead=0.17, blade_offset=0.02, body_fit=False, spin_margin=0.0, perimeter_passes=3,
                    obstacle_passes=-1)
_plans = {}


def plan_of(drive, area=AREA, obstacles=(), **more):
    key = (drive, tuple(area), tuple(map(tuple, obstacles)), tuple(sorted(more.items())))
    if key not in _plans:
        _plans[key] = plan_coverage(area, list(obstacles), CFG.with_overrides(dict(more, drive=drive)))
    return _plans[key]


def steps_against(plan, seg):
    """For each step of seg whether it goes against the heading of the pose it leaves."""
    return [(b.x - a.x) * math.cos(a.yaw) + (b.y - a.y) * math.sin(a.yaw) < 0
            for a, b in zip(plan.poses[seg.start:seg.end], plan.poses[seg.start + 1:seg.end + 1])]


def test_settings_check_the_body():
    """slope and reverse plan with body_fit and spin_margin at least DRIVE_SPIN_MARGIN; slope lanes only, reverse the
    body turned round (front and rear, the blade's place, the outline)."""
    slope, reverse = (backing.settings(CFG.with_overrides({'drive': d})) for d in ('slope', 'reverse'))
    for cfg in (slope, reverse):
        assert cfg.body_fit and cfg.spin_margin == 0.05
    assert (slope.perimeter_passes, slope.obstacle_passes, slope.fill_pattern) == (0, 0, 'lanes')
    assert (reverse.robot_front, reverse.robot_rear, reverse.blade_ahead, reverse.blade_offset) == (0.18, 0.43, -0.17,
                                                                                                    -0.02)
    assert reverse.perimeter_passes == 3 and reverse.drive == 'reverse'
    outline = CFG.with_overrides({'robot_outline': [(0.4, 0.2), (-0.1, 0.2), (-0.1, -0.2), (0.4, -0.2)],
                                  'drive': 'reverse', 'spin_margin': 0.1})
    assert backing.settings(outline).robot_outline == ((-0.4, -0.2), (0.1, -0.2), (0.1, 0.2), (-0.4, 0.2))
    assert backing.settings(outline).spin_margin == 0.1
    assert backing.forced(CFG) == ['body_fit on', 'spin_margin 0.05'] and not backing.forced(backing.settings(outline))


def test_slope_up_forwards_down_backwards():
    """Every second lane backwards, the poses facing up the lanes all along (the S at most SHIFT_MAX flat), steps of
    at most 0.1 m and none doubled, the body inside where check_plan has it (backing up is no fault there), one path
    for mower_logic, the forced body check in the warnings."""
    up = plan_of('slope')
    assert not up.perimeter_loops and up.stats['drive'] == 'slope'
    lengths = {k: sum(math.dist(up.poses[i][:2], up.poses[i + 1][:2]) for s in up.segments if s.kind == k
                      for i in range(s.start, s.end)) for k in (LANE, 'turn')}
    assert up.stats['lane_length'] == pytest.approx(lengths[LANE]) and up.stats['poses'] == len(up.poses)
    assert up.stats['turn_length'] == pytest.approx(lengths['turn']) and up.stats['lanes'] == len(up.lanes)
    assert up.stats['backing_m'] > 0.4 * up.stats['path_length']
    lanes = [s for s in up.segments if s.kind == LANE]
    assert len(lanes) >= 10 and [s.reverse for s in lanes] == [i % 2 == 1 for i in range(len(lanes))]
    for seg in up.segments:
        assert all(back == seg.reverse for back in steps_against(up, seg))
        for a, b in zip(up.poses[seg.start:seg.end], up.poses[seg.start + 1:seg.end + 1]):
            assert 1e-6 < math.hypot(b.x - a.x, b.y - a.y) <= 0.1 + 1e-9
    for p in up.poses:
        off = (p.yaw - up.angle + math.pi) % (2 * math.pi) - math.pi
        assert min(abs(off), abs(abs(off) - math.pi)) < math.radians(25)
    check_invariants(up, up.config)
    report = check_plan(up)
    assert not report['body_out'] and not any('backwards' in h for h in report['hard'])
    assert len(plan_pieces(up)) == 1
    assert up.config.drive == 'slope' and up.config.body_fit and up.config.spin_margin == 0.05
    assert up.warnings == ['drive slope: body_fit on and spin_margin 0.05, ' + backing.WHY]


def test_slope_no_leftovers():
    """slope plans no leftovers: they'd turn on the slope."""
    with pytest.raises(PlanningError, match='drive slope: no leftovers'):
        plan_coverage(AREA, [], CFG.with_overrides({'drive': 'slope'}), mowed=Polygon(AREA).buffer(-0.5))


def _body(cfg, p):
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    return [(p.x + c * x - s * y, p.y + s * x + c * y) for x, y in cfg.body_outline]


def test_reverse_is_the_plan_for_the_body_turned_round():
    """reverse: every pose turned round and backwards, the body where the plan for the body turned round has it,
    check_plan finds it inside also turning on the spot (the way in ahead of the pose: it came backing up), the stump
    too; the blade where it mows the same; the outermost loop goes round the other way than the outline was recorded
    (the mower backing up the way it recorded it: its body where it was then; with the blade in the middle, else the
    blade's side decides)."""
    back = plan_of('reverse', GARDEN, [STUMP], blade_offset=0.0)
    turned = backing.settings(back.config.with_overrides({'drive': 'reverse'}))
    assert back.config.drive == 'reverse' and back.config.robot_front == 0.43 and back.stats['drive'] == 'reverse'
    assert all(s.reverse for s in back.segments) and back.perimeter_loops and back.recorded_loops
    for seg in back.segments:
        assert all(steps_against(back, seg))
    spins = [i for i in range(1, len(back.poses)) if Footprint.spins(None, back.poses, i)]
    assert spins  # (it turns on the spot: the check below counts)
    for p in back.poses[::7]:
        q = Pose2D(p.x, p.y, p.yaw + math.pi)
        assert Polygon(_body(turned, q)).symmetric_difference(Polygon(_body(back.config, p))).area < 1e-9
    check_invariants(back, back.config, [STUMP])
    report = check_plan(back)
    assert not report['body_out'] and not report['blade_over'], report['body_out']
    assert not any('backwards' in h for h in report['hard'])
    # FTC backs up all along: a step from a pose backs up where it and the next pose elsewhere both face against it
    # (backing.corners), but where it turns round on the spot (no heading between both ways)
    for i, (a, b) in enumerate(zip(back.poses, back.poses[1:])):
        way = (b.x - a.x, b.y - a.y)
        if i + 1 not in back.section_starts and not all(way[0] * math.cos(p.yaw) + way[1] * math.sin(p.yaw) < 0
                                                        for p in (a, b)):
            assert abs(math.cos(a.yaw - b.yaw) + 1) < 0.01, i
    # (the plan's first loop: driven clockwise, against the outline's counterclockwise points)
    loop = next(s for s in back.segments if s.kind == PERIMETER and s.source in back.recorded_loops)
    pts = [(p.x, p.y) for p in back.poses[loop.start:loop.end + 1]]
    assert sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1])) < 0


def test_spin_checked_where_it_came_backing_up():
    """Turning on the spot after backing up the body is checked anywhere on SPIN_BACK of the way in ahead of the pose,
    as going forwards behind it: by a wall ahead it may turn going forwards, not after backing up."""
    space = Polygon([(-5.0, -5.0), (1.0, -5.0), (1.0, 5.0), (-5.0, 5.0)])
    fp = Footprint(space, CFG.body_outline, 0.05)
    ahead = [x / 100.0 for x in range(0, 60)]
    forwards = [fp.fits_turn(x, 0.0, 0.0, math.pi / 2) for x in ahead]
    backwards = [fp.fits_turn(x, 0.0, 0.0, math.pi / 2, backwards=True) for x in ahead]
    pairs = list(zip(forwards, backwards))
    assert any(f and not b for f, b in pairs) and not any(b and not f for f, b in pairs)
    i = next(k for k, (f, b) in enumerate(zip(forwards, backwards)) if f and not b)
    poses = [Pose2D(ahead[i] + 0.1, 0.0, 0.0), Pose2D(ahead[i], 0.0, math.pi / 2)]
    assert fp.bad_poses(poses) == [] and fp.bad_poses(poses, backwards=[False, True]) == [1]
    turns = [(poses[0], poses[1])]
    assert fp.out_by(turns)[0][1] is None and fp.out_by(turns, backwards=[True])[0][0] > 0
    assert SPIN_BACK > 0


def test_fingerprint_takes_the_direction():
    """The fingerprint: the poses' alone going forwards (as before), every segment backing up taken too."""
    plan = plan_of('slope')
    h = hashlib.sha256()
    for p in plan.poses:
        h.update(struct.pack('<3d', p.x, p.y, p.yaw))
    forwards = dataclasses.replace(plan, segments=[dataclasses.replace(s, reverse=False) for s in plan.segments])
    assert fingerprint(forwards) == h.hexdigest() != fingerprint(plan)


def test_no_body_no_backing_up():
    """Without the body's size it can't be checked: forwards, said in the warnings."""
    cfg = PlannerConfig(mower_width=0.18, drive='reverse')
    plan = plan_coverage(AREA, [], cfg)
    assert not any(s.reverse for s in plan.segments) and plan.config.drive == 'forward'
    assert plan.warnings[-1].startswith('drive reverse ignored: no body set')
    with pytest.raises(PlanningError, match='drive must be one of'):
        plan_coverage(AREA, [], cfg.with_overrides({'drive': 'backwards'}))
