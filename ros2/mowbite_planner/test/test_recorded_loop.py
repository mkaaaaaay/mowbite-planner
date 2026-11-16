"""recorded_loop: the outermost loop along the outline is the line the area was recorded on, the way it was recorded,
recorded_loop_margin inside it, the body where it was while recording (recorded.py)."""

import dataclasses
import math
import os
import sys

import pytest
from shapely.geometry import LineString, Point, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), HERE, os.path.join(HERE, '..', 'tools')]

from mowbite_planner import PlannerConfig, PlanningError, plan_coverage  # noqa: E402
from mowbite_planner import remote  # noqa: E402
from mowbite_planner.config import LANE, PERIMETER, RECORDED_OFF, RECORDED_SLACK  # noqa: E402
from mowbite_planner.export import to_trace_json  # noqa: E402
from mowbite_planner.geometry import inset  # noqa: E402
from mowbite_planner.mathutil import Pose2D, angle_diff  # noqa: E402
from mowbite_planner.report import check_plan, rules  # noqa: E402
from mowbite_planner.sweep import body_out, on_recorded, plan_footprint, recorded_footprint, swept_area  # noqa: E402
import metrics  # noqa: E402
from planrules import check_invariants  # noqa: E402
from test_planner import outside_room, recorded_room  # noqa: E402


def rounded(w, h, r=1.0, step=10):
    """A lawn w by h m as a mower records it: its corners driven round (r), a point every step degrees."""
    return [(cx + r * math.cos(math.radians(a0 + a)), cy + r * math.sin(math.radians(a0 + a)))
            for cx, cy, a0 in ((w - r, r, -90), (w - r, h - r, 0), (r, h - r, 90), (r, r, 180))
            for a in range(0, 91, step)]


LAWN = rounded(8.0, 5.0)
YARD = [(0.0, 0.0), (8.0, 0.0), (8.0, 5.0), (0.0, 5.0)]  # (drawn: right angles)
# (LAWN's last edge a cm off either way, a dent in it; a spike into the lawn)
KINKED = LAWN + [(1.0 + 0.25 * k, 0.02 * (k % 2)) for k in range(1, 8)] + [(3.0, 0.0), (4.0, 0.22), (5.0, 0.0)]
SPIKED = LAWN + [(3.9, 0.0), (4.0, 0.5), (4.1, 0.0)]
BED = [(3.0, 2.0), (4.0, 2.0), (4.0, 3.0), (3.0, 3.0)]
NX60I = dict(robot_width=0.40, robot_front=0.43, robot_rear=0.14)
CFG = PlannerConfig(mower_width=0.18, overlap=1 - 0.13 / 0.18, blade_ahead=0.17, perimeter_passes=3,
                    lane_order='skip', recorded_loop_margin=0.0, **NX60I)
_plans = {}


def plan_of(area=LAWN, obstacles=(), more=None, **settings):
    """The plan of area with CFG and settings (more: plan_coverage's other arguments by name), planned once."""
    key = (tuple(area), tuple(map(tuple, obstacles)), repr(more), tuple(sorted(settings.items())))
    if key not in _plans:
        _plans[key] = plan_coverage(list(area), list(obstacles), CFG.with_overrides(settings), **(more or {}))
    return _plans[key]


def its_poses(plan, leads=False):
    """The poses of the recorded loop itself (leads: of the ways onto it and off it too)."""
    return [p for s in plan.segments if s.kind == PERIMETER and s.source in plan.recorded_loops
            and (leads or not s.maneuver) for p in plan.poses[s.start:s.end + 1]]


def along(ring, poses):
    """(steps on along the ring the way its points go, steps back) of the poses, each less than a metre."""
    line = LineString(list(ring) + [ring[0]])
    at = [line.project(Point(p.x, p.y)) for p in poses]
    return sum(0.0 < b - a < 1.0 for a, b in zip(at, at[1:])), sum(-1.0 < b - a < 0.0 for a, b in zip(at, at[1:]))


@pytest.mark.parametrize('ring', [LAWN, LAWN[::-1]], ids=['ccw', 'cw'])
@pytest.mark.parametrize('margin', [0.0, 0.03, 0.09])
def test_on_the_line_the_way_it_was_recorded(ring, margin):
    """The loop's centre on the recorded line, recorded_loop_margin inside it (a kink's arc RECORDED_OFF beside it at
    most), all the way round, the way the ring's points go whichever way round that is; perimeter_passes counts it."""
    plan = plan_of(ring, recorded_loop=True, recorded_loop_margin=margin)
    told, line = plan.stats['recorded_loop'], LineString(ring + ring[:1])
    assert told['pieces'] == 1 and told['left_m'] < 0.1 and told['else_m'] == 0 and plan.recorded_loops == ('loop-0',)
    poses = its_poses(plan)
    assert all(abs(line.distance(Point(p.x, p.y)) - margin) <= RECORDED_OFF + 1e-6 for p in poses)
    assert all(Polygon(ring).buffer(RECORDED_OFF + 1e-6).covers(Point(p.x, p.y)) for p in its_poses(plan, True))
    on, back = along(ring, poses)
    assert on > 200 and back == 0
    assert plan.stats['chosen']['perimeter_passes'] == 3
    assert sorted({o['parent'] for n, o in plan.origins.items() if n.startswith('loop')}) == \
        ['level-0', 'level-1', 'recorded']
    check_invariants(plan, plan.config)
    assert [h for h in check_plan(plan)['hard'] if 'unmowed' not in h] == []


@pytest.mark.parametrize('direction', ['cw', 'ccw', 'random'])
def test_perimeter_direction_is_not_its(direction):
    """perimeter_direction doesn't count for the recorded loop: the same poses whatever it says."""
    plan = plan_of(recorded_loop=True, perimeter_direction=direction)
    assert along(LAWN, its_poses(plan))[1] == 0
    assert its_poses(plan) == its_poses(plan_of(recorded_loop=True))


def test_the_body_where_it_was_recording():
    """On the loop the body is where it was while the line was recorded (worked out here on its own, RECORDED_SLACK
    past it), not edge_margin off that as everywhere else: checked in its own space the plan's body sticks out
    nowhere, in the others' all along the loop."""
    plan = plan_of(recorded_loop=True)
    cfg, fp = plan.config, plan_footprint(plan, blade=False)
    mild = on_recorded(plan.segments, plan.recorded_loops, len(plan.poses))
    assert body_out(plan.poses, plan.segments, plan.section_starts, fp, recorded_footprint(plan)) == []
    assert len(body_out(plan.poses, plan.segments, plan.section_starts, fp)) > 100
    then = recorded_room(dataclasses.replace(cfg, edge_margin=0.0), [LAWN], [], [])
    assert [i for i in outside_room(plan, then, slack=RECORDED_SLACK + 1e-3) if mild[i]] == []
    room = recorded_room(cfg, [LAWN], [], [])
    assert [i for i in outside_room(plan, room, spin=cfg.spin_margin) if not mild[i]] == []
    assert len([i for i in outside_room(plan, room) if mild[i]]) > 100


@pytest.mark.parametrize('margin', [0.0, 0.03])
def test_no_strip_between_it_and_the_next_loop(margin):
    """The loops after the recorded one follow a lane spacing in from it: nothing left between them, and less by the
    edge than without it."""
    lawn = Polygon(LAWN)
    band = lawn.difference(inset(lawn, 0.45))

    def left(plan):
        return lawn.difference(swept_area(plan, plan.config.mower_width)).intersection(band).area

    assert left(plan_of(recorded_loop=True, recorded_loop_margin=margin)) < 0.005 < left(plan_of())


@pytest.mark.parametrize('margin', [0.0, 0.02, 0.03, 0.09])
def test_a_kink_moved_in_stays_a_kink(margin):
    """A line that zigzags and has a dent (kinks turning away from the lawn, 9 and 25 degrees): moved in it keeps them
    as they are, a point each, and is driven like the line itself, no step turning on the spot (as arcs of the
    margin's radius round those points it turned on the spot at each point of an arc)."""
    plan = plan_of(KINKED, recorded_loop=True, recorded_loop_margin=margin)
    told, line = plan.stats['recorded_loop'], LineString(KINKED + KINKED[:1])
    assert told['pieces'] == 1 and told['left_m'] < 0.1
    poses = its_poses(plan)
    assert [a for a, b in zip(poses, poses[1:])
            if math.hypot(b.x - a.x, b.y - a.y) < 0.005 and angle_diff(a.yaw, b.yaw) > 1e-6] == []
    assert all(abs(line.distance(Point(p.x, p.y)) - margin) <= RECORDED_OFF + 1e-6 for p in poses)


@pytest.mark.parametrize('margin', [0.02, 0.09])
def test_a_spike_moved_in_keeps_the_margin(margin):
    """A kink too sharp to stay a point moved in (a spike into the lawn, 157 degrees): the loop is nowhere closer to
    the line than the margin."""
    plan = plan_of(SPIKED, recorded_loop=True, recorded_loop_margin=margin)
    line = LineString(SPIKED + SPIKED[:1])
    assert plan.stats['recorded_loop']['m'] > 20.0
    assert min(line.distance(Point(p.x, p.y)) for p in its_poses(plan)) >= margin - 1e-6
    assert [h for h in check_plan(plan)['hard'] if 'unmowed' not in h] == []


def test_the_obstacles_loops_as_ever():
    """Round an obstacle nothing changes: its loops where they are without recorded_loop."""
    def loops(plan):
        return sorted((Polygon(plan.perimeter_loops[int(n.split('-')[1])]) for n, o in plan.origins.items()
                       if n.startswith('loop') and o['kind'] == 'obstacle'), key=lambda p: p.area)

    on, off = loops(plan_of(obstacles=[BED], recorded_loop=True)), loops(plan_of(obstacles=[BED]))
    assert len(on) == len(off) == 3
    assert all(a.symmetric_difference(b).area < 1e-3 for a, b in zip(on, off))


def test_right_angles_turned_on_the_spot_as_recorded():
    """A drawn outline's right angles: on the line the mower turns on the spot there as it did recording; moved in it
    doesn't fit turning there (the nose swings out past where it was), that corner is driven round further in."""
    plan = plan_of(YARD, recorded_loop=True)
    line = LineString(YARD + YARD[:1])
    assert plan.stats['recorded_loop']['m'] == pytest.approx(26.0, abs=0.05)
    assert max(line.distance(Point(p.x, p.y)) for p in its_poses(plan)) < 1e-6
    moved = plan_of(YARD, recorded_loop=True, recorded_loop_margin=0.03)
    told = moved.stats['recorded_loop']
    assert told['pieces'] == 1 and told['repaired'] == 4 and told['m'] > 24.0
    far = [line.distance(Point(p.x, p.y)) for p in its_poses(moved)]
    assert min(far) == pytest.approx(0.03, abs=1e-6) and 0.1 < max(far) < 0.3
    for plan in (plan, moved):
        assert body_out(plan.poses, plan.segments, plan.section_starts, plan_footprint(plan, blade=False),
                        recorded_footprint(plan)) == []


@pytest.mark.parametrize('settings, why', [
    (dict(robot_width=None, robot_front=None, robot_rear=None), 'no mower size set'),
    (dict(edges='hard'), 'edges hard'),
    (dict(blade_offset=0.02), 'blade_offset set'),
    (dict(fill_pattern='concentric'), 'fill_pattern concentric'),
    (dict(perimeter_passes=0), 'no loops along the outline'),
], ids=['no_body', 'hard', 'blade_offset', 'concentric', 'passes0'])
def test_planned_without_it_where_it_cant_be(settings, why):
    """What else is set keeps the recorded loop from being driven: the plan as without it, its warnings say why."""
    area = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]
    plan, plain = plan_of(area, recorded_loop=True, **settings), plan_of(area, **settings)
    assert [w for w in plan.warnings if w.startswith('recorded_loop ignored: ')][0][23:].startswith(why)
    assert plan.poses == plain.poses and plan.recorded_loops == () and plan.recorded_space is None
    assert not any('recorded_loop' in w for w in plain.warnings)
    assert CFG.with_overrides(dict(settings, recorded_loop=True)).recorded_off.startswith(why)
    assert CFG.with_overrides(settings).recorded_off == ''


def test_a_larger_mower_plans_without_it():
    """An area recorded with a smaller mower (the map's recorded_body): planned without the recorded loop, said in
    the warnings; recorded with this one (or none told): with it."""
    small = PlannerConfig(robot_width=0.3, robot_front=0.3, robot_rear=0.1).body_outline
    ring = tuple(tuple(map(float, p)) for p in LAWN)
    plan = plan_of(more={'bodies': ((ring, small),)}, recorded_loop=True)
    assert 'recorded_loop ignored: the mower is larger than the one the area was recorded with' in plan.warnings
    assert plan.recorded_loops == () and plan.poses == plan_of(more={'bodies': ((ring, small),)}).poses
    same = plan_of(more={'bodies': ((ring, CFG.body_outline),)}, recorded_loop=True)
    assert same.recorded_loops and same.poses == plan_of(recorded_loop=True).poses


def test_planned_without_it_where_no_way_gets_to_it(monkeypatch):
    """A robot that drives to its plans itself (arrives) and gets to nothing of the plan the recorded loop starts (its
    first pose out by the edge): the area is planned without the loop and the warnings say so, nothing refused."""
    from mowbite_planner import planner
    reached = planner.reached_starts

    def none_with_it(plan, arrives, back_out=False):
        if plan.recorded_loops:
            raise PlanningError('no way gets to any section of the plan facing its heading: at (0.00, 0.00)')
        return reached(plan, arrives, back_out)

    def gets(pose, came):
        return True
    monkeypatch.setattr(planner, 'reached_starts', none_with_it)
    plan = plan_coverage(list(LAWN), [], CFG.with_overrides({'recorded_loop': True}), arrives=gets)
    assert plan.recorded_loops == () and plan.warnings[-1] == planner.RECORDED_UNREACHED
    assert plan.poses == plan_coverage(list(LAWN), [], CFG, arrives=gets).poses
    with pytest.raises(PlanningError):  # (not what kept it from being got to: refused as without it)
        monkeypatch.setattr(planner, 'reached_starts', lambda plan, arrives, back_out=False: none_with_it(
            dataclasses.replace(plan, recorded_loops=('loop-0',)), arrives, back_out))
        plan_coverage(list(LAWN), [], CFG, arrives=gets)


def test_planned_without_it_where_a_section_is_left_out(monkeypatch):
    """The robot gets to some of the plan with the recorded loop but a section is left out (no way to its start):
    planned without the loop too and that plan taken where it leaves out less. A section starting later doesn't
    count, and no second plan where the ways it may ask about ran out."""
    from mowbite_planner import planner
    reached, left, plain = planner.reached_starts, {}, {}

    def some_left(plan, arrives, back_out=False):
        got = reached(plan, arrives, back_out)
        more = left if plan.recorded_loops else plain
        return dataclasses.replace(got, stats=dict(got.stats, starts=dict(got.stats['starts'], **more)))

    def gets(pose, came):
        return True
    monkeypatch.setattr(planner, 'reached_starts', some_left)
    cfg = CFG.with_overrides({'recorded_loop': True})
    left.update(places=[[0.0, 0.0, 0.3]])
    assert plan_coverage(list(LAWN), [], cfg, arrives=gets).recorded_loops == ('loop-0',)
    left.update(dropped=[[0.0, 0.0]])
    plan = plan_coverage(list(LAWN), [], cfg, arrives=gets)
    assert plan.recorded_loops == () and plan.warnings[-1] == planner.RECORDED_UNREACHED
    plain.update(dropped=[[1.0, 1.0]])  # (as much left out without it)
    assert plan_coverage(list(LAWN), [], cfg, arrives=gets).recorded_loops == ('loop-0',)
    plain.clear()
    left.update(untried=[[2.0, 2.0]])
    assert plan_coverage(list(LAWN), [], cfg, arrives=gets).recorded_loops == ('loop-0',)


def test_settings_checked():
    for bad in (dict(recorded_loop=1), dict(recorded_loop_margin=-0.01), dict(recorded_loop_margin=0.51),
                dict(recorded_loop_margin=math.nan)):
        with pytest.raises(PlanningError):
            CFG.with_overrides(bad).validate()
    CFG.with_overrides(dict(recorded_loop=True, recorded_loop_margin=0.5)).validate()


def test_the_loops_counted():
    """perimeter_passes 1: the recorded loop alone; -1: as many after it as the lanes' turns need; perimeter_order
    last: it's the last of the outline's, after the lanes; skip_outline_loops: none."""
    def parents(plan):
        return sorted({o['parent'] for n, o in plan.origins.items() if n.startswith('loop')})

    one = plan_of(recorded_loop=True, perimeter_passes=1)
    assert parents(one) == ['recorded'] and one.stats['chosen']['perimeter_passes'] == 1
    auto = plan_of(recorded_loop=True, perimeter_passes=-1)
    count = auto.stats['chosen']['perimeter_passes']
    assert parents(auto) == [f'level-{k}' for k in range(count - 1)] + ['recorded']
    assert count >= plan_of(perimeter_passes=-1).stats['chosen']['perimeter_passes']
    last = plan_of(recorded_loop=True, perimeter_order='last')
    kinds = [(s.kind, s.source in last.recorded_loops) for s in last.segments]
    assert kinds.index((PERIMETER, True)) > max(i for i, k in enumerate(kinds) if k[0] == LANE)
    skipped = plan_of(recorded_loop=True, skip_outline_loops=True)
    assert skipped.recorded_loops == () and parents(skipped) == []


def test_the_blade_kept_off_the_lawn_next_door():
    """Along an area not to mow the loop runs further in, the blade doesn't reach over it."""
    next_door = [(8.0 - 1e-9, 1.5), (11.0, 1.5), (11.0, 3.5), (8.0 - 1e-9, 3.5)]
    plan = plan_of(more={'unmowed': [next_door]}, recorded_loop=True)
    assert rules(plan)['blade_over'] == [] and plan.stats['recorded_loop']['repaired'] >= 1
    assert max(LineString(LAWN + LAWN[:1]).distance(Point(p.x, p.y)) for p in its_poses(plan)) > 0.08


def test_backwards_the_body_as_recorded():
    """drive reverse: the loop driven the other way round backing up, the body facing as it did recording."""
    plan = plan_of(recorded_loop=True, drive='reverse')
    poses = its_poses(plan)
    on, back = along(LAWN, poses)
    assert on == 0 and back > 200
    line = LineString(LAWN + LAWN[:1])
    for p in poses[::10]:
        at = line.project(Point(p.x, p.y))
        a, b = line.interpolate(at - 0.05), line.interpolate(at + 0.05)
        assert angle_diff(p.yaw, math.atan2(b.y - a.y, b.x - a.x)) < math.radians(12)
    assert body_out(plan.poses, plan.segments, plan.section_starts, plan_footprint(plan, blade=False),
                    recorded_footprint(plan)) == []
    assert [h for h in check_plan(plan)['hard'] if 'unmowed' not in h] == []


@pytest.mark.parametrize('settings', [dict(mop_up=False), dict(back_out=True), dict(lane_spacing_mode='auto'),
                                      dict(headland_turns=True), dict(narrow_parts='loops'), dict(split_zones=True),
                                      dict(perimeter_order='last')],
                         ids=['no_mop_up', 'back_out', 'auto', 'headland', 'narrow_loops', 'zones', 'last'])
def test_with_other_settings(settings):
    """The loop whole and on the line with what else changes the path, no move of the completion stage from it."""
    plan = plan_of(recorded_loop=True, **settings)
    assert plan.stats['recorded_loop']['left_m'] < 0.1
    assert max(LineString(LAWN + LAWN[:1]).distance(Point(p.x, p.y)) for p in its_poses(plan)) < RECORDED_OFF + 1e-6
    assert not any(s.maneuver == 'mop_up' and s.source in plan.recorded_loops for s in plan.segments)
    check_invariants(plan, plan.config)
    assert [h for h in check_plan(plan)['hard'] if 'unmowed' not in h] == []


def test_too_small_for_it():
    """A lawn no way onto the line fits on: the first loop as without it, in its place."""
    plan = plan_of([(0.0, 0.0), (0.9, 0.0), (0.9, 0.9), (0.0, 0.9)], recorded_loop=True)
    told = plan.stats['recorded_loop']
    assert told['pieces'] == 0 and told['m'] == 0 and told['else_m'] > 2.0 and plan.recorded_loops == ()
    assert 'instead' in {o['parent'] for o in plan.origins.values()}
    assert body_out(plan.poses, plan.segments, plan.section_starts, plan_footprint(plan, blade=False)) == []


def test_remote_form_and_key():
    """The plan's form (remote_planning) has the loop's pieces, read back the plan has them and its space again;
    recorded_loop and its margin are part of a plan's key. A plan without one has no such field."""
    cfg = CFG.with_overrides({'recorded_loop': True})
    plan = plan_of(recorded_loop=True)
    form = remote.plan_json(plan, cfg)
    assert form['recorded_loops'] == ['loop-0'] and 'recorded_loops' not in remote.plan_json(plan_of(), CFG)
    args = (remote.canon(LAWN), [], cfg, None, None, (), (), ())
    back = remote.plan_from(form, args)
    assert back.recorded_loops == ('loop-0',) and back.poses == plan.poses
    assert back.recorded_space.symmetric_difference(plan.recorded_space).area < 1e-9
    assert remote.refused(back) == ''
    keys = {remote.plan_key('code', remote.request_json((args[0], [], c) + args[3:]))
            for c in (CFG, cfg, cfg.with_overrides({'recorded_loop_margin': 0.03}))}
    assert len(keys) == 3


def test_what_a_mow_left_has_none():
    """Over what a mow left (mowed) no loop is driven: the same moves with recorded_loop as without."""
    whole = plan_of(recorded_loop=True)
    mowed = swept_area(whole, whole.config.mower_width).difference(Polygon(BED))
    on, off = (plan_coverage(list(LAWN), [], CFG.with_overrides({'recorded_loop': loop}), mowed=mowed)
               for loop in (True, False))
    assert on.poses and on.recorded_loops == () and on.poses == off.poses


def test_told_where_it_comes_from():
    """Planner Studio's trace: the recorded loop made from the area, saying what it is."""
    objects = to_trace_json(plan_of(recorded_loop=True))['lineage']['objects']
    loop = next(o for o in objects if o['id'] == 'loop-0')
    assert loop['op']['reason'] == 'the line the area was recorded on (recorded_loop)'
    assert loop['parents'] == [o['id'] for o in objects if o['stage'] == 'area']


def test_measured_in_its_own_space():
    """The measuring tools' rules: the loop against where the body was recording (ring_out), the rest against the
    margins (rm_out); a loop 3 cm further out than recorded is found."""
    plan = plan_of(recorded_loop=True)
    ref = metrics.Ref(Polygon(LAWN), metrics.request(LAWN, [], plan.config))
    got = dict(metrics.rules(plan, ref), **metrics.plan_body(plan, ref))
    assert got['rm_out'] == 0 and got['ring_out'] == 0 and got['centre_out'] == 0
    assert 'ring_out' not in metrics.plan_body(plan_of(), metrics.Ref(Polygon(LAWN), metrics.request(LAWN, [], CFG)))
    mild = on_recorded(plan.segments, plan.recorded_loops, len(plan.poses))
    out = [Pose2D(p.x + 0.03 * math.sin(p.yaw), p.y - 0.03 * math.cos(p.yaw), p.yaw) if m else p
           for p, m in zip(plan.poses, mild)]
    assert metrics.plan_body(dataclasses.replace(plan, poses=out), ref)['ring_out'] > 0
