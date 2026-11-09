"""plan_coverage's progress: from angles to done, done never going back, of the same all along, the plan the same."""

import json

from mowbite_planner import PlannerConfig, plan_coverage
from mowbite_planner import core
from mowbite_planner.progress import STAGES, Progress
from shapely.geometry import box

AREA = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.5), (0.0, 2.5)]
CFG = PlannerConfig(mower_width=0.18, lane_spacing_max=0.13, robot_width=0.41, robot_front=0.43, robot_rear=0.18)


def planned(**kw):
    seen = []
    plan = plan_coverage(AREA, [], progress=lambda *step: seen.append(step), **kw)
    return plan, seen


def check(seen):
    assert seen and seen[0][:2] == ('angles', 0) and seen[-1] == ('done', seen[-1][2], seen[-1][2])
    assert len({of for _, _, of in seen}) == 1 and all(stage in STAGES for stage, _, _ in seen)
    dones = [done for _, done, _ in seen]
    assert dones == sorted(dones) and all(0 <= d <= seen[0][2] for d in dones)


def test_fixed_spacing():
    plan, seen = planned(config=CFG)
    check(seen)
    assert {'route', 'mop_up'} <= {stage for stage, _, _ in seen}
    assert plan.poses == plan_coverage(AREA, [], CFG).poses


def test_auto_spacing():
    cfg = CFG.with_overrides({'lane_spacing_mode': 'auto'})
    plan, seen = planned(config=cfg)
    check(seen)
    assert 'spacing' in {stage for stage, _, _ in seen}
    assert plan.poses == plan_coverage(AREA, [], cfg).poses


def test_leftovers():
    _, seen = planned(config=CFG, mowed=box(0.0, 0.0, 3.0, 2.0))
    check(seen)


def test_progress_never_goes_back():
    seen = []
    p = Progress(lambda *step: seen.append(step), 4)
    p.at('angles', 0)
    p.at('spacing')
    p.at('spacing', 0)
    p.at('done', 9)
    assert seen == [('angles', 0, 4), ('spacing', 1, 4), ('spacing', 1, 4), ('done', 4, 4)]


def test_took_kept_over_a_restart(tmp_path, monkeypatch):
    """The time each area's last planning took is in took_file, a PlanProgress after it estimates with it; failed ones
    not, only the latest TOOK_AREAS areas, a broken file none (said)."""
    monkeypatch.setattr(core, 'TOOK_AREAS', 2)
    path = str(tmp_path / core.TOOK_FILE)
    sent, warned = [], []
    p = core.PlanProgress(sent.append, took_file=path)
    for area, error in (('a', None), ('b', 'no'), ('c', None), ('d', None)):
        p.begin(area, 'k')
        p.end(error)
    with open(path) as f:
        assert set(json.load(f)['took']) == {'c', 'd'}
    again = core.PlanProgress(sent.append, took_file=path)
    assert again.took == p.took
    del sent[:]
    again.begin('d', 'k2')
    assert json.loads(sent[0])['estimate_s'] is not None
    again.end()
    with open(path, 'w') as f:
        f.write('{"took": [1]}')
    assert core.PlanProgress(sent.append, logwarn=lambda *a: warned.append(a), took_file=path).took == {}
    assert warned and core.PlanProgress(sent.append).took == {}
