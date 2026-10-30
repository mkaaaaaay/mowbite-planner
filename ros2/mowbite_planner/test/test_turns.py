"""The turns' catalogue (turns.py), the ways of a cell (stripes.py) and the order (order.py) as invariants."""
import itertools
import math
import os
import random
import sys
from types import SimpleNamespace

import pytest
from shapely.geometry import Polygon

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner.config import Lane, PlannerConfig  # noqa: E402
from mowbite_planner.footprint import Footprint  # noqa: E402
from mowbite_planner.mathutil import Pose2D, angle_diff  # noqa: E402
from mowbite_planner.objective import counted  # noqa: E402
from mowbite_planner.order import EXACT, best_order, exact_order, plan_order  # noqa: E402
from mowbite_planner.stripes import Cells, Run  # noqa: E402
from mowbite_planner.transit import TransitPlanner  # noqa: E402
from mowbite_planner.turns import Catalog, _as_driven  # noqa: E402

FIELD = Polygon([(0, 0), (10, 0), (10, 6), (0, 6)])


def route(cfg, space=FIELD, body=None):
    area = TransitPlanner(space, 0.0)
    fp = Footprint(space, body, cfg.spin_margin) if body is not None else None
    arcs = any(t in cfg.allowed_turns for t in ('u_turn', 'bulb', 'detour'))
    return SimpleNamespace(cfg=cfg, footprint=fp, step=cfg.waypoint_spacing, area=area,
                           rounds=cfg.min_turn_radius > 0 and arcs, turn_space=lambda at, a: area.is_free,
                           detour_path=area.path)


def catalog(cfg, **kw):
    return Catalog(route(cfg, **kw), lambda geom: geom.area, cfg.lane_spacing)


def forwards(poses):
    """Every step goes the way the pose it leaves faces (or turns on the spot)."""
    for p, q in zip(poses, poses[1:]):
        d = math.hypot(q.x - p.x, q.y - p.y)
        if d > 1e-6:
            assert angle_diff(math.atan2(q.y - p.y, q.x - p.x), p.yaw) < math.radians(60.0)


@pytest.mark.parametrize('d', [0.13, 0.26, 0.5, 0.65, 1.3])
@pytest.mark.parametrize('spot', [False, True])
def test_turns_go_forwards_from_end_to_start_without_loops(d, spot):
    cfg = PlannerConfig(turn_on_spot=spot, mower_width=0.18)
    c = catalog(cfg)
    end, start = Pose2D(7.0, 2.0, 0.0), Pose2D(7.0, 2.0 + d, math.pi)
    t = c.turn(end, start, 1.0, 1.0)
    assert t is not None
    a, b = t.poses[0], t.poses[-1]
    assert math.hypot(a.x - (7.0 - t.cut_exit), a.y - 2.0) < 1e-6 and math.hypot(b.x - (7.0 - t.cut_entry),
                                                                                 b.y - 2.0 - d) < 1e-6
    forwards(t.poses)
    assert counted(_as_driven(list(t.poses))).loops == 0
    assert t.name in cfg.allowed_turns and (t.name != 'bulb' or not spot) and (t.name != 'pivot' or spot)


def test_turn_types_and_turn_on_spot():
    """Without pivot no turning on the spot between lanes, with turn_on_spot no bulb; turn_on_spot off: no pivot, but
    where turn_types allow nothing else."""
    end, start = Pose2D(7.0, 2.0, 0.0), Pose2D(7.0, 2.13, math.pi)
    for types, spot, allowed in ((('u_turn', 'bulb', 'detour', 'pivot'), False, {'u_turn', 'bulb'}),
                                 (('u_turn', 'pivot'), True, {'u_turn', 'pivot'}),
                                 (('pivot',), True, {'pivot'}),
                                 (('pivot',), False, {'pivot'})):
        t = catalog(PlannerConfig(turn_types=types, turn_on_spot=spot)).turn(end, start, 1.0, 1.0)
        assert t is not None and t.name in allowed
        if t.name != 'pivot':
            assert counted(_as_driven(list(t.poses))).spins == 0


def test_tight_only_where_nothing_of_turn_radius_fits():
    """Lanes closer than 2 turn_radius: a bulb where it fits, the half circle as wide as the lanes are apart only where
    none does (a wall right past the lanes' ends)."""
    cfg = PlannerConfig(turn_radius=0.25)
    end, start = Pose2D(7.0, 2.0, 0.0), Pose2D(7.0, 2.2, math.pi)
    assert catalog(cfg).turn(end, start, 0.0, 0.0).name == 'bulb'
    tight = catalog(cfg, space=Polygon([(0, 0), (7.15, 0), (7.15, 6), (0, 6)])).turn(end, start, 0.0, 0.0)
    assert tight is not None and tight.name == 'u_turn' and max(p.x for p in tight.poses) < 7.15


def test_the_body_ends_the_lanes_earlier_where_the_turn_needs_it():
    """A wall 0.3 m past the lanes' ends: the U-turn with the body checked (spin_margin too) moves back into the lanes,
    by what the body needs, what that leaves counted."""
    body = [(0.43, 0.2), (-0.14, 0.2), (-0.14, -0.2), (0.43, -0.2)]
    cfg = PlannerConfig(turn_radius=0.25, robot_width=0.4, robot_front=0.43, robot_rear=0.14, spin_margin=0.02)
    wall = Polygon([(0, 0), (7.3, 0), (7.3, 6), (0, 6)])
    end, start = Pose2D(7.0, 2.0, 0.0), Pose2D(7.0, 2.5, math.pi)
    t = catalog(cfg, space=wall, body=body).turn(end, start, 1.0, 1.0)
    assert t is not None and t.cut_exit > 0.1 and t.cut_entry > 0.1 and t.cost.left_m2 > 0.0
    fp = Footprint(wall, body, cfg.spin_margin)
    assert fp.poses_fit(list(t.poses), True)
    assert catalog(cfg, space=wall, body=body).turn(end, start, 0.05, 0.05) is None  # (no room to end earlier)


def test_a_cells_lanes_without_loops():
    """lane_order skip: the lanes rows apart for U-turns in passes (each the same way across) where that costs less than
    neighbour after neighbour (snake: always); every lane once, every turn found, and no two turns going round a lane
    too short to be between them (no spiral of loops), with long lanes and short ones."""
    for order, n, length in itertools.product(('skip', 'snake'), (3, 6, 9, 14), (9.0, 0.8)):
        cfg = PlannerConfig(lane_order=order, turn_radius=0.25, lane_spacing_mode='fixed', mower_width=0.18,
                            overlap=1 - 0.13 / 0.18)
        cell = [Lane(i, 0.5, 0.5 + length, 0.5 + 0.13 * i) for i in range(n)]

        def runs(lane, direction, arrive, leave):
            a, b = (lane.x0, lane.x1) if direction > 0 else (lane.x1, lane.x0)
            return Run((a, lane.y), (b, lane.y), 0.0 if direction > 0 else math.pi), ''

        cells = Cells(cfg, runs, catalog(cfg), lambda a, b: counted([a, b]), lambda lane: 0.0)
        got = cells.ways(cell)
        assert got and len(got) == 4 and all(w.turns is None for w in got)
        for way in map(cells.realize, got):
            rows = [lane.row for lane, _ in way.runs]
            assert sorted(rows) == list(range(n)) and all(t is not None for t in way.turns)
            if order == 'snake':
                assert all(abs(b - a) == 1 for a, b in zip(rows, rows[1:]))
            elif n > 4 and length > 5.0:  # (passes of rows 4 apart, for a U-turn of turn_radius)
                passes = [i for p in range(4) for i in range(p, n, 4)]
                assert rows in (passes, passes[::-1])
            for (lane, run), t0, t1 in zip(way.runs[1:], way.turns, way.turns[1:]):
                poses = list(t0.poses) + list(t1.poses)
                assert counted(_as_driven(poses)).loops == 0, (order, n, length, lane.row)


@pytest.mark.parametrize('seed', range(6))
def test_exact_order_is_the_cheapest(seed):
    rng = random.Random(seed)
    n = rng.randint(1, 6)
    k = [rng.randint(1, 3) for _ in range(n)]
    start = [[rng.uniform(0, 10) for _ in range(k[i])] for i in range(n)]
    end = [[rng.uniform(0, 10) for _ in range(k[i])] for i in range(n)]
    table = {(i, a, j, b): rng.uniform(0, 10) for i in range(n) for a in range(k[i]) for j in range(n)
             for b in range(k[j])}

    def link(i, a, j, b):
        return table[i, a, j, b]

    def total(seq):
        return start[seq[0][0]][seq[0][1]] + sum(link(*p, *q) for p, q in zip(seq, seq[1:])) + \
            end[seq[-1][0]][seq[-1][1]]

    brute = min(total(list(zip(perm, ways))) for perm in itertools.permutations(range(n))
                for ways in itertools.product(*[range(k[i]) for i in perm]))
    got = exact_order(start, link, end)
    assert sorted(i for i, _ in got) == list(range(n)) and total(got) == pytest.approx(brute)
    assert plan_order(start, link, end) == got  # (up to EXACT groups exactly)
    assert total(best_order(start, link, end)) >= brute - 1e-9
    assert n <= EXACT
