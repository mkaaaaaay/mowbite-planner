"""Tests of the simulation (simulate.py): OpenMower's FTC controller and mower_logic driving a plan, xbot_positioning
and the sensors, the track as position_history keeps it, what the mow did."""

import dataclasses
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', '..', 'ros2', 'mowbite_planner'),
                os.path.join(HERE, '..', '..', 'ros2', 'mowbite_planner', 'test')]

from mowbite_planner import PlanningError  # noqa: E402
from mowbite_planner.export import plan_pieces  # noqa: E402
from mowbite_planner.mathutil import Pose2D  # noqa: E402
from golden import golden_plan  # noqa: E402
from mowbite_sim.simulate import (_FOLLOWING, APPROACH, HISTORY_DIST, PATH, MowerModel, _decimate,  # noqa: E402
                                  _Ftc, _History, _rdp, evaluate, simulate)


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


AREA = rect(0, 0, 10, 6)
BED = rect(4, 2, 5, 3)
BODY = dict(robot_width=0.4, robot_front=0.43, robot_rear=0.14)
_runs = {}


def plan_of(**kw):
    """The planner's plan of AREA with BED (mower_width and blade_ahead 0.18) as it was kept (golden.py), kw: none or
    BODY."""
    return golden_plan('garden-sim-body' if kw == BODY else 'garden-sim')


def run_of(model, **kw):
    """The plan of plan_of(**kw) simulated with model, once each."""
    key = (model, tuple(sorted(kw.items())))
    if key not in _runs:
        plan = plan_of(**kw)
        _runs[key] = plan, simulate(plan, model)
    return _runs[key]


def test_ideal_mower_keeps_to_the_plan():
    """An ideal mower (it knows where it is, the wheels do what they're told) keeps to the plan within FTC's own
    errors: a few mm on the lanes, mm more in the turns, mows what the plan mows, never gives up."""
    plan, res = run_of(MowerModel())
    ev = evaluate(plan, res)
    assert ev['nav_errors'] == 0 and not ev['aborted']
    assert ev['tracking']['lane']['p95'] < 0.01 and ev['tracking']['lane']['max'] < 0.03
    assert ev['tracking']['estimate']['max'] == 0.0
    assert abs(ev['left_m2'] - ev['planned_left_m2']) < 0.05
    assert ev['mowed_m2'] > 0.98 * ev['area_m2'] - ev['planned_left_m2']


def test_time_like_drive_time():
    """The ideal mower takes about as long as report.drive_time's guess (FTC's speeds), plus mower_logic's seconds
    at each path (a second's sleep after each goal, spinning the blade up)."""
    plan, res = run_of(MowerModel())
    ev = evaluate(plan, res)
    assert 0.85 * ev['model_s'] < ev['mow_s'] < 1.1 * ev['model_s']
    assert ev['s'] > ev['mow_s']


def test_same_seed_same_mow():
    """The same seed gives the same mow to the bit, another one another (within the errors' bounds)."""
    plan = plan_of()
    a, b = simulate(plan, MowerModel.realistic(3)), simulate(plan, MowerModel.realistic(3))
    c = simulate(plan, MowerModel.realistic(4))
    assert a.samples == b.samples and a.history == b.history
    assert a.samples != c.samples
    ea, ec = evaluate(plan, a), evaluate(plan, c)
    assert abs(ea['left_m2'] - ec['left_m2']) < 0.2
    for ev in ea, ec:
        assert ev['tracking']['lane']['p95'] < 0.06 and not ev['aborted']


def test_realistic_mower_is_off_by_cm():
    """A realistic mower is cm off the plan where it really is (where it thinks it is it keeps to it as well as the
    ideal one), and leaves a bit more."""
    plan = plan_of()
    ev = evaluate(plan, simulate(plan, MowerModel.realistic(1)))
    assert 0.003 < ev['tracking']['lane']['p50'] < 0.03
    assert 0.005 < ev['tracking']['estimate']['p50'] < 0.06


def test_pieces_like_mower_logic():
    """Every path of the plan is driven to its first pose (move_base) and then along (exe_path); the blade is spun up
    once (it stays on between the paths of an area) and goes off at the end."""
    plan, res = run_of(MowerModel())
    starts = [s[9] for s, before in zip(res.samples[1:], res.samples) if s[8] == PATH and before[8] != PATH]
    assert starts == [a for _, a, _ in plan_pieces(plan)]
    blades = [e['enabled'] for e in res.events if e['type'] == 'BLADES']
    assert blades == [True, False]
    kinds = [e['type'] for e in res.events]
    assert kinds[0] == 'STATE' and kinds[-2:] == ['JOB_COMPLETE', 'STATE']
    assert all(s[8] in (APPROACH, PATH) for s in res.samples)


def test_ftc_turns_on_the_spot_before_following():
    """FTC turns on the spot to the first pose's heading (to within max_goal_angle_error) before it follows, then
    drives along: here a straight line behind the mower."""
    m = MowerModel()
    ftc = _Ftc(m)
    poses = [(0.1 * i, 0.0, 0.0) for i in range(30)]
    ftc.set_plan(poses, 0.0)
    v, w, ok = ftc.compute((0.0, 0.0, 0.5 * math.pi), 0.05)
    assert ok and v == 0.0 and abs(w) == m.max_cmd_vel_ang
    # (within 20 degrees: following, the point running ahead; the D term's kick first)
    v, w, ok = ftc.compute((0.0, 0.0, 0.3), 0.1)
    assert ok and ftc.state == _FOLLOWING
    for k in range(3, 10):
        v, w, ok = ftc.compute((0.0, 0.0, 0.0), 0.05 * k)
    assert ok and v > 0 and abs(w) < 0.5


def test_ftc_gives_up_far_off():
    """FTC gives up (crashed) when the mower is max_follow_distance off its point, and stays given up."""
    ftc = _Ftc(MowerModel())
    ftc.set_plan([(0.1 * i, 0.0, 0.0) for i in range(30)], 0.0)
    ftc.compute((0.0, 0.0, 0.0), 0.05)
    ftc.compute((0.0, 0.0, 0.0), 0.1)
    assert ftc.compute((0.0, 1.5, 0.0), 0.15)[2] is False
    assert ftc.compute((0.0, 0.0, 0.0), 0.2) == (0.0, 0.0, False)
    assert not ftc.reached()


def test_nav_error_where_the_mower_cant_follow():
    """Where the mower can't follow (a drive that does a third of what it's told), FTC gives up: a navigation error,
    the blade off and on again, the mow goes on from where FTC's point got to."""
    plan = plan_of()
    res = simulate(plan, dataclasses.replace(MowerModel(), drive_gain=0.3))
    assert res.nav_errors
    kinds = [e['type'] for e in res.events]
    assert 'NAVIGATION_ERROR' in kinds
    blades = [e['enabled'] for e in res.events if e['type'] == 'BLADES']
    assert blades[:3] == [True, False, True]


def test_body_places_where_the_mower_is_off():
    """Off by a few cm to the side (a wheel slower than the other, positioning that doesn't notice: steered off), the
    body gets into the edges' margins where the plan had it fit."""
    plan = plan_of(**BODY)
    ideal = evaluate(plan, simulate(plan, MowerModel()))
    off = evaluate(plan, simulate(plan, dataclasses.replace(MowerModel(), positioning=True, gps_drift=0.06,
                                                            gps_drift_tau=1e6, seed=2)))
    assert ideal['body']['worst'] < 0.01
    assert off['body']['worst'] > ideal['body']['worst'] + 0.01
    assert all(len(place) == 3 for place in off['body']['places'])


def test_history_like_position_history():
    """The track kept like xbot_monitoring keeps it: points 0.1 m apart on a straight line, a point where it turns,
    1 cm Douglas-Peucker, a new segment where the blade changes."""
    line = [(0.01 * i, 0.0, True) for i in range(200)]
    kept = _rdp(_decimate(line))
    # (a straight line: its ends, the last one kept by decimation not further than 0.1 m from the line's end)
    assert len(kept) == 2 and kept[0][0] == 0.0 and kept[1][0] > 1.88
    kept = _decimate(line)
    assert all(abs((b[0] - a[0]) - HISTORY_DIST) < 0.011 for a, b in zip(kept, kept[1:-1]))
    corner = [(0.01 * i, 0.0, True) for i in range(100)] + [(0.99, 0.01 * i, True) for i in range(1, 100)]
    # (the corner: the first point past it, where the heading has changed)
    assert any(math.hypot(x - 0.99, y) <= 0.011 for x, y, _ in _rdp(_decimate(corner)))
    h = _History()
    h.event({'type': 'STATE', 'job_id': 'j', 'session_id': 's'}, 0.0)
    for i in range(100):
        h.add(0.01 * i, 0.0, 0.0)
    h.event({'type': 'BLADES', 'enabled': True}, 1.0)
    for i in range(100):
        h.add(1.0 + 0.01 * i, 0.0, 1.0)
    lines = h.lines()
    assert [s['attributes']['blades'] for s in lines] == [False, True]
    assert lines[1]['points'][0] == lines[0]['points'][-1]  # (the bridge)
    plan, res = run_of(MowerModel())
    assert [s['attributes']['blades'] for s in res.history][:2] == [False, True]
    assert all(s['attributes']['job_id'] == res.events[0]['job_id'] for s in res.history)


def test_model_checked():
    """Unknown names and bad values are a PlanningError."""
    with pytest.raises(PlanningError):
        MowerModel().with_overrides({'wheels': 3})
    with pytest.raises(PlanningError):
        MowerModel().with_overrides({'speed_fast': 0})
    with pytest.raises(PlanningError):
        MowerModel().with_overrides({'slip': 1.5})
    with pytest.raises(PlanningError):
        MowerModel().with_overrides({'gps_sigma': 'a lot'})
    m = MowerModel().with_overrides({'gps_sigma': '0.01', 'positioning': 1, 'seed': 4.0})
    assert m.gps_sigma == 0.01 and m.positioning is True and m.seed == 4


def test_start_elsewhere():
    """From the dock (somewhere else) the mower drives to the first path's start first, the blade off."""
    plan = plan_of()
    first = plan.poses[plan_pieces(plan)[0][1]]
    res = simulate(plan, MowerModel(), start=Pose2D(first.x + 1.0, first.y + 1.0, 0.0))
    approach = [s for s in res.samples if s[8] == APPROACH and not s[7]]
    assert approach and math.hypot(approach[0][1] - first.x, approach[0][2] - first.y) > 1.0


def drive_ftc(model, poses, steps=2000, dt=0.05):
    """FTC (_Ftc) on poses driving a mower that does what it's told: ([(t, x, y, yaw, v)] each cycle till it's done
    or gives up, whether it got there)."""
    ftc = _Ftc(model)
    ftc.set_plan(poses, 0.0)
    x, y, yaw = poses[0]
    out = []
    for k in range(1, steps):
        t = k * dt
        v, w, ok = ftc.compute((x, y, yaw), t)
        if not ok:
            break
        x, y, yaw = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt
        out.append((t, x, y, yaw, v))
        if ftc.reached():
            break
    return out, ftc.reached()


def test_ftc_plan_backs_up():
    """planBacksUp: where a pose and the next one elsewhere (more than 1 mm off) both face against the way between
    them; a pose facing the way it came (slic3r's obstacle outlines) doesn't."""
    ftc = _Ftc(MowerModel())
    ahead = [(0.1 * i, 0.0, 0.0) for i in range(5)]
    back = [(-0.1 * i, 0.0, 0.0) for i in range(5)]
    ftc.set_plan(ahead, 0.0)
    assert not any(ftc._backs_up(i) for i in range(5))
    ftc.set_plan(back[:1] + [(0.0, 0.0005, 0.0)] + back[1:], 0.0)
    assert all(ftc._backs_up(i) for i in range(5))
    ftc.set_plan([(0.0, 0.0, 0.0), (0.1, 0.0, math.pi), (0.2, 0.0, 0.0), (0.3, 0.0, 0.0)], 0.0)
    assert not ftc._backs_up(0) and not ftc._backs_up(1)


def test_ftc_backs_up_with_the_plan():
    """back_up_with_plan: 2 m ahead, 2 m back facing the same way: backed up, the point stopping where the plan turns
    back (no jump in the speed asked for), at speed_reverse at most; without it the mower never backs up."""
    line = [(0.05 * i, 0.0, 0.0) for i in range(41)]
    poses = line + [(x, y, a) for x, y, a in reversed(line[:-1])]
    model = MowerModel().with_overrides({'back_up_with_plan': True})
    run, there = drive_ftc(model, poses)
    v = [s[4] for s in run]
    turn = [s[4] for s in run if s[1] > 1.0]  # (where it turns back, at 2 m)
    assert min(v) < -0.1 and max(abs(b - a) for a, b in zip(turn, turn[1:])) < 0.05
    assert there and abs(run[-1][1]) < 0.2 and abs(run[-1][2]) < 0.02
    slow, there = drive_ftc(model.with_overrides({'speed_reverse': 0.08}), poses)
    backing = [s for s in slow if s[4] < 0]
    assert there and backing and -0.08 * 1.3 < sum(s[4] for s in backing) / len(backing) < 0
    run, there = drive_ftc(MowerModel(), poses)
    assert not there and min(s[4] for s in run) >= 0.0


def test_back_up_with_plan_leaves_forward_plans_alone():
    """back_up_with_plan on: a plan that only goes forwards is driven to the bit as without it."""
    plan = plan_of()
    a = simulate(plan, MowerModel.realistic(0))
    b = simulate(plan, MowerModel.realistic(0).with_overrides({'back_up_with_plan': True}))
    assert a.samples == b.samples
