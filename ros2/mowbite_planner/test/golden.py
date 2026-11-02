"""Plans of the planner kept as they were (test/golden/*.json.gz), for the tests of what measures or drives plans
(export, report, the simulation): those test the measure against a plan that stays the same, not the planner. Made
anew only when CoveragePlan changes: python3 test/golden.py (from ros2/mowbite_planner)."""

import dataclasses
import gzip
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

import shapely  # noqa: E402

from mowbite_planner import CoveragePlan, PlannerConfig, Pose2D, SegmentInfo  # noqa: E402
from mowbite_planner.config import Lane  # noqa: E402

DIR = os.path.join(HERE, 'golden')
AREA = [(0, 0), (10, 0), (10, 6), (0, 6)]
BED = [(4, 2), (5, 2), (5, 3), (4, 3)]
BODY = dict(robot_width=0.4, robot_front=0.43, robot_rear=0.14)
# name: what it was planned from (plan_coverage's boundary, obstacles and settings)
PLANS = {
    'garden': (AREA, [BED], {}),
    'garden-sim': (AREA, [BED], dict(mower_width=0.18, blade_ahead=0.18)),
    'garden-sim-body': (AREA, [BED], dict(mower_width=0.18, blade_ahead=0.18, **BODY)),
}
SPACES = ('area', 'free_space', 'robot_space', 'lane_region', 'remaining', 'transit_space', 'body_space', 'narrow',
          'recorded_space', 'unmowed')
_loaded = {}


def golden_plan(name: str) -> CoveragePlan:
    """The plan kept as name (one of PLANS), the same object every time (don't change it)."""
    if name not in _loaded:
        with gzip.open(os.path.join(DIR, name + '.json.gz'), 'rt') as f:
            _loaded[name] = from_json(json.load(f))
    return _loaded[name]


def to_json(plan: CoveragePlan):
    cfg = {f.name: getattr(plan.config, f.name) for f in dataclasses.fields(PlannerConfig)}
    return {
        'poses': [[p.x, p.y, p.yaw] for p in plan.poses],
        'segments': [dataclasses.astuple(s) for s in plan.segments],
        'angle': plan.angle, 'lane_spacing': plan.lane_spacing, 'perimeter_loops': plan.perimeter_loops,
        'lanes': plan.lanes, 'stats': plan.stats, 'warnings': plan.warnings, 'angles': plan.angles,
        'config': cfg, 'section_starts': plan.section_starts, 'origins': plan.origins,
        'recorded_loops': plan.recorded_loops,
        'cells': [[a, [[dataclasses.astuple(lane) for lane in cell] for cell in cells]] for a, cells in plan.cells],
        'levels': [shapely.to_wkb(g, hex=True) for g in plan.levels],
        **{k: None if getattr(plan, k) is None else shapely.to_wkb(getattr(plan, k), hex=True)
           for k in SPACES}}


def from_json(d) -> CoveragePlan:
    def tuples(v):
        return tuple(tuples(x) for x in v) if isinstance(v, list) else v

    cfg = PlannerConfig(**{k: tuples(v) for k, v in d['config'].items()})
    return CoveragePlan(
        poses=[Pose2D(*p) for p in d['poses']], segments=[SegmentInfo(*s) for s in d['segments']], angle=d['angle'],
        lane_spacing=d['lane_spacing'], perimeter_loops=[[tuple(p) for p in loop] for loop in d['perimeter_loops']],
        lanes=[(tuple(a), tuple(b)) for a, b in d['lanes']], stats=d['stats'], warnings=d['warnings'],
        angles=d['angles'], config=cfg, section_starts=d['section_starts'], origins=d['origins'],
        recorded_loops=d['recorded_loops'], levels=[shapely.from_wkb(g) for g in d['levels']],
        cells=[(a, [[Lane(*lane[:4], tuples(lane[4])) for lane in cell] for cell in cells]) for a, cells in d['cells']],
        **{k: None if d[k] is None else shapely.from_wkb(d[k]) for k in SPACES})


def main():
    from mowbite_planner import plan_coverage
    os.makedirs(DIR, exist_ok=True)
    for name, (boundary, obstacles, settings) in PLANS.items():
        plan = plan_coverage(boundary, obstacles, PlannerConfig(**settings))
        with gzip.GzipFile(os.path.join(DIR, name + '.json.gz'), 'wb', mtime=0) as f:
            f.write(json.dumps(to_json(plan), separators=(',', ':')).encode())
        back = from_json(to_json(plan))
        assert [tuple(p) for p in back.poses] == [tuple(p) for p in plan.poses]
        assert json.dumps(to_json(back)) == json.dumps(to_json(plan))  # (all of it back as it was)
        print(name, len(plan.poses), 'poses')


if __name__ == '__main__':
    main()
