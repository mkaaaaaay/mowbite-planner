#!/usr/bin/env python3
"""Plans many gardens the way the mower's node does, drives the plans in the simulation and ranks what looks wrong:
a bench for finding planner bugs. Not part of the package (not on the mower).

    python3 tools/simbench.py --quick --out bench                # each kind of garden once, the real ones
    python3 tools/simbench.py --seeds 10 --mowers mkay,tango      # kinds x 10 seeds, the real and the tests' gardens
    python3 tools/simbench.py --case neck-3/0@tango#0             # one case again, exactly
    python3 tools/simbench.py --list --kinds neck,strip           # the cases only

A case is <garden>/<area>@<mower>#<seed>: a garden of tools/gardens.py by name, the index of the area among the
garden's areas to plan, a mower of gardens.MOWERS and the seed of the simulated mower (MowerModel.realistic), the
simulation run with seeds 0..--sim-seeds - 1 without it. Every case plans in a process of its own, killed after
--timeout s. What looks wrong is an anomaly with a score (see SCORES), a case's severity is its worst score and a
fifth of the others. In --out: results.json (every case, worst first), summary.md and <case>.png of the flagged ones
(severity FLAG at least): the garden, the plan, the simulated track, what's left unmowed, where the body sticks out.

The simulation (mowbite_planner.simulate) is used if it can be imported, else only the plans are checked.
"""

import argparse
import json
import math
import multiprocessing
import os
import re
import sys
import time
import traceback
from multiprocessing.connection import wait
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(TOOLS, '..'))
sys.path.insert(0, TOOLS)

import gardens  # noqa: E402
from mowbite_planner import PlannerConfig, PlanningError  # noqa: E402
from mowbite_planner.config import LANE, PERIMETER, TRANSIT, TURN, CoveragePlan  # noqa: E402
from mowbite_planner.export import plan_pieces  # noqa: E402
from mowbite_planner.mqtt_service import PlannerService  # noqa: E402
from mowbite_planner.planner import plan_coverage  # noqa: E402
from mowbite_planner.report import SPOT_TURN, check_plan, drive_time  # noqa: E402
from shapely.geometry import Point, Polygon  # noqa: E402

try:
    from mowbite_planner.simulate import MowerModel, evaluate, simulate
    SIM_MISSING = ''
except Exception as e:  # (not there yet, or broken while it's being written)
    MowerModel = evaluate = simulate = None
    SIM_MISSING = f'{type(e).__name__}: {e}'

# the ROS 1 node: its launch file's settings (read from there, these if it's missing), the settings stored ones of
# beat mower_logic's (MOWER_LOGIC), its own ones that aren't the planner's
LAUNCH_FILE = os.path.join(TOOLS, '..', '..', '..', 'ros1', 'mowbite_planner_ros1', 'launch', 'mowbite_planner.launch')
LAUNCH = {'turn_radius': 0.25, 'narrow_parts': 'loops', 'edges': 'recorded', 'lane_spacing_mode': 'auto'}
MOWER_LOGIC = ('overlap', 'perimeter_offset', 'perimeter_passes', 'lane_overlap_passes')
NODE_SETTINGS = ('planner', 'leftovers_auto')

# what looks wrong and its score (the worst cases first)
SCORES = {
    'crash': 100,  # the planner raised something else than PlanningError
    'timeout': 90,  # no plan in --timeout s
    'aborted': 80,  # simulated: mower_logic gave up the area
    'outside': 70,  # check_plan: poses outside the space the mower's centre may be in
    'gap': 70,  # check_plan: poses further apart than waypoint_spacing
    'backwards': 70,  # check_plan: a segment backwards
    'refused': 50,  # a PlanningError for a garden that passed gardens.validate()
    'hard': 40,  # another hard finding of check_plan
    'nav_error': 40,  # simulated: FTC gave up a path, + 10 each (80 at most)
    'body_out': 30,  # check_plan: the body over the edges, + 5 a place (80 at most)
    'sim_body': 15,  # simulated: the body (true pose) out of body_space by SIM_BODY at least, + 400 x the most m,
    # + 1 a place (60 at most)
    'sim_error': 30,  # the simulation raised
    'stuck': 30,  # planned, but checking or simulating took longer than --timeout s
    'slow': 20,  # planned in more than SLOW s, + 1 for every 6 s more (60 at most)
    'pirouette': 15,  # turning a full circle on the spot or nearly (see turning), + 5 each (50 at most)
    'sim_left': 15,  # simulated: much more left than planned, + 500 x the share of the area more (60 at most)
    'left': 10,  # left unmowed within reach (LEFT_*, see reach), + 1000 x the share of the area + 10 x the biggest
    # piece m2 (70 at most)
    'transit': 10,  # TRANSIT_SHARE of the way at least drives between parts, + 1 a % more (40 at most)
    'spins': 10,  # SPIN_SHARE of the time at least turning on the spot, + 1 a % more (30 at most)
    'sections': 10,  # a garden of one piece planned in several sections (the mower drives between them on its own)
    'duration': 10,  # simulated time off drive_time by more than DURATION, + 20 x how far (30 at most)
}
# severity from which a case is flagged (drawn, in the summary's list)
FLAG = 15.0
# s, planning slower than this is an anomaly
SLOW = 60.0
# left unmowed counts from LEFT_SHARE of the area or LEFT_M2 m2 (all pieces), or a piece of LEFT_PIECE m2
LEFT_SHARE, LEFT_M2, LEFT_PIECE = 0.005, 0.25, 0.25
TRANSIT_SHARE = 0.25
SPIN_SHARE = 0.15
# rad, m: the heading turning a full circle while the mower stays this close to where it began (not on the loops
# along the edges and round obstacles)
PIROUETTE, PIROUETTE_REACH = 2 * math.pi, 0.6
# m, body_out poses closer than this are one place
PLACE = 0.5
# m, the simulated body out of body_space by less doesn't count: body_space keeps edge_margin and obstacle_margin
# (both mowers' 0.05) off the real edges, tracking is off by 2..3 cm (p95) in the realistic model
SIM_BODY = 0.05
# simulated s / drive_time s outside 1 +- DURATION
DURATION = 0.4


# planning like the node ---------------------------------------------------------------------------------------------

def launch_settings() -> Dict[str, Any]:
    """The settings of the node's launch file (<rosparam param="settings">{name: value, ...}</rosparam>)."""
    try:
        with open(LAUNCH_FILE) as f:
            found = re.search(r'<rosparam param="settings">\s*\{(.*?)\}\s*</rosparam>', f.read(), re.S)
    except OSError:
        found = None
    if not found:
        return dict(LAUNCH)
    return {k.strip(): gardens._scalar(v) for k, v in (item.split(':', 1) for item in found.group(1).split(','))}


def _canon(v: Any) -> Any:  # the node's canon()
    if isinstance(v, (list, tuple)):
        return tuple(_canon(x) for x in v)
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else v


def _node_plan(boundary, obstacles, cfg, start=None, end=None, drivable=(), keep_out=(), margins=(),
               reach_over=()) -> CoveragePlan:
    """plan_coverage with the arguments of the node's kept_plan(): obstacles sorted with their margins, finished_mows
    only for perimeter_direction random."""
    if cfg.perimeter_direction != 'random' and cfg.finished_mows:
        cfg = cfg.with_overrides({'finished_mows': 0})
    margins = list(margins) + [None] * (len(obstacles) + len(keep_out) - len(margins))
    pairs = sorted(zip(_canon(obstacles), _canon(margins)), key=lambda pair: pair[0])
    obstacles = [o for o, _ in pairs]
    margins = tuple(m for _, m in pairs) + _canon(margins[len(obstacles):])
    return plan_coverage(_canon(boundary), obstacles, cfg, _canon(start), _canon(end), _canon(drivable),
                         _canon(keep_out), margins, reach_over=_canon(reach_over))


def plan_area(garden: gardens.Garden, area_id: str, mower: gardens.Mower) -> CoveragePlan:
    """The plan the mower's node makes for an area (its setup(), rpc_plan(), PlannerService.plan, kept_plan())."""
    stored = {n: v for n, v in mower.stored.items() if n not in NODE_SETTINGS}
    settings = dict(launch_settings(), **{n: v for n, v in stored.items() if n != 'mower_width'})
    base = PlannerConfig(lane_order='skip').with_overrides(settings).with_overrides({
        'mower_width': stored.get('mower_width', mower.blade), 'angle_steps': garden.angle.get('steps', 0),
        'finished_mows': garden.angle.get('finished', 0)})
    service = PlannerService(base, 'angle' in settings or 'angle_strategy' in settings)
    service.own = frozenset(n for n in settings if n in MOWER_LOGIC)
    service.map, service.params = garden.map, mower.mower_logic
    kept: List[CoveragePlan] = []
    service.planner = lambda *args, **kw: kept.append(_node_plan(*args, **kw)) or kept[-1]
    service.plan({'area_id': area_id})
    return kept[-1]


# measuring ----------------------------------------------------------------------------------------------------------

def turning(plan: CoveragePlan) -> Dict[str, Any]:
    """{'spins', 'spin_deg', 'arc_deg', 'pirouettes': [[x, y], ...]}: turns on the spot (heading jumps of SPOT_TURN
    or on one position, one after another counted once), the turning on arcs, the places where the heading turns
    PIROUETTE while the mower stays within PIROUETTE_REACH m (not on the loops along the edges and round obstacles)."""
    poses = plan.poses
    loop = [False] * len(poses)
    for seg in plan.segments:
        if seg.kind == PERIMETER or seg.maneuver == 'ring':
            loop[seg.start + 1:seg.end + 1] = [True] * (seg.end - seg.start)
    jumps = {s - 1 for s in plan.section_starts[1:]}
    spins, spin, arc, was = 0, 0.0, 0.0, False
    turned = [0.0]  # rad, the heading's turns added up
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        d, turn = math.hypot(b.x - a.x, b.y - a.y), math.remainder(b.yaw - a.yaw, 2 * math.pi)
        turned.append(turned[-1] + (0.0 if i in jumps else turn))
        on_spot = i not in jumps and (d < 1e-6 or abs(turn) > SPOT_TURN)
        spins += on_spot and not was
        was = on_spot
        if on_spot:
            spin += abs(turn)
        elif i not in jumps:
            arc += abs(turn)
    places: List[List[float]] = []
    i = 0
    while i < len(poses):
        j, hit = i, 0
        while j + 1 < len(poses) and not loop[j + 1] and j not in jumps and \
                math.hypot(poses[j + 1].x - poses[i].x, poses[j + 1].y - poses[i].y) <= PIROUETTE_REACH:
            j += 1
            if abs(turned[j] - turned[i]) >= PIROUETTE:
                hit = j
                break
        if hit:
            places.append([round(poses[i].x, 2), round(poses[i].y, 2)])
        i = hit + 1 if hit else i + 1
    return {'spins': spins, 'spin_deg': round(math.degrees(spin)), 'arc_deg': round(math.degrees(arc)),
            'pirouettes': places}


def kinds_m(plan: CoveragePlan) -> Dict[str, float]:
    """m of way by segment kind."""
    out = {PERIMETER: 0.0, LANE: 0.0, TURN: 0.0, TRANSIT: 0.0}
    for seg in plan.segments:
        out[seg.kind] += sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in
                             zip(plan.poses[seg.start:seg.end], plan.poses[seg.start + 1:seg.end + 1]))
    return out


def reach(plan: CoveragePlan) -> Any:
    """Where the blade could get at most: the body's centre at least the body's inscribed radius inside body_space (its
    outline holds that circle round the centre whatever the heading), the blade's disk round it at any heading (without
    a body: the centre in robot_space). More than the mower can reach (headings and turns aren't looked at), so what's
    left out of it is out of reach for sure."""
    cfg = plan.config
    blade = math.hypot(cfg.blade_ahead, cfg.blade_offset) + 0.5 * cfg.mower_width
    if cfg.body and plan.body_space is not None:
        inner = Polygon(cfg.body_outline).exterior.distance(Point(0.0, 0.0))
        return plan.body_space.buffer(-inner).buffer(blade)
    return plan.robot_space.buffer(blade)


def group(points: Sequence[Sequence[float]], near: float = PLACE) -> List[List[float]]:
    """Points closer than near to a place's first one put together: [[x, y, count or the most m], ...]."""
    places: List[List[float]] = []
    for p in points:
        m = p[2] if len(p) > 2 else 1.0
        for q in places:
            if math.hypot(p[0] - q[0], p[1] - q[1]) < near:
                q[2] = q[2] + m if len(p) <= 2 else max(q[2], m)
                break
        else:
            places.append([round(p[0], 2), round(p[1], 2), m])
    return places


def _count(v: Any) -> int:
    return len(v) if isinstance(v, (list, tuple)) else int(v or 0)


def _track(history: Sequence[Any]) -> List[Tuple[bool, List[List[float]]]]:
    """position_history lines (JSON or dicts) as [(blades, points)]."""
    out: List[Tuple[bool, List[List[float]]]] = []
    for line in history or ():
        d = json.loads(line) if isinstance(line, (str, bytes)) else line
        if d.get('type') == 'new_segment':
            out.append((bool(d.get('attributes', {}).get('blades')), list(d.get('points') or [])))
        elif d.get('type') == 'append_points' and out:
            out[-1][1].extend(d.get('points') or [])
    return out


def _add(found: List[Dict[str, Any]], kind: str, detail: str, extra: float = 0.0, cap: float = 100.0) -> None:
    found.append({'kind': kind, 'score': round(min(cap, SCORES[kind] + extra), 1), 'detail': detail})


def severity(anomalies: Sequence[Dict[str, Any]]) -> float:
    scores = sorted((a['score'] for a in anomalies), reverse=True)
    return round(scores[0] + 0.2 * sum(scores[1:]), 1) if scores else 0.0


def plan_anomalies(rec: Dict[str, Any], check: Dict[str, Any]) -> List[Dict[str, Any]]:
    """What looks wrong with a plan, from rec's numbers and check_plan."""
    found: List[Dict[str, Any]] = []
    for text in check['hard']:
        if 'outside the space' in text:
            _add(found, 'outside', text)
        elif text.startswith(tuple('0123456789')) and 'gaps between poses' in text:
            _add(found, 'gap', text)
        elif text == 'drives backwards':
            _add(found, 'backwards', text)
        elif not ('body sticks out' in text or 'left unmowed' in text):
            _add(found, 'hard', text)
    if rec['body_places']:
        _add(found, 'body_out', f'{len(check["body_out"])} poses at {len(rec["body_places"])} places, the first at '
             f'({rec["body_places"][0][0]}, {rec["body_places"][0][1]})', 5 * len(rec['body_places']), 80)
    area = max(rec['area_m2'], 1e-6)
    share = rec['reach_left_m2'] / area
    if share >= LEFT_SHARE or rec['reach_left_m2'] >= LEFT_M2 or rec['reach_biggest'] >= LEFT_PIECE:
        _add(found, 'left', f'{rec["reach_left_m2"]:.2f} m2 ({100 * share:.1f} %) within reach of {rec["left_m2"]:.2f} '
             f'm2 left in {rec["left_pieces"]} pieces, the biggest {rec["reach_biggest"]:.2f} m2 at {rec["left_at"]}',
             1000 * share + 10 * rec['reach_biggest'], 70)
    if rec['plan_s'] > SLOW:
        _add(found, 'slow', f'planned in {rec["plan_s"]:.0f} s', (rec['plan_s'] - SLOW) / 6, 60)
    if rec['transit_share'] >= TRANSIT_SHARE:
        _add(found, 'transit', f'{100 * rec["transit_share"]:.0f} % of the way are transits',
             100 * (rec['transit_share'] - TRANSIT_SHARE), 40)
    if rec['pirouettes']:
        _add(found, 'pirouette', f'{len(rec["pirouettes"])}, the first at {rec["pirouettes"][0]}',
             5 * (len(rec['pirouettes']) - 1), 50)
    spin_share = rec['spin_s'] / max(rec['drive_s'], 1e-6)
    if spin_share >= SPIN_SHARE:
        _add(found, 'spins', f'{rec["spins"]} spins, {100 * spin_share:.0f} % of the time',
             100 * (spin_share - SPIN_SHARE), 30)
    if rec['sections'] > 1 and rec['free_parts'] == 1:
        _add(found, 'sections', f'{rec["sections"]} sections in a lawn of one piece')
    return found


def sim_anomalies(rec: Dict[str, Any], ev: Dict[str, Any], seed: int) -> List[Dict[str, Any]]:
    found: List[Dict[str, Any]] = []
    at = f'seed {seed}: '
    places = [p for p in (ev.get('body') or {}).get('places') or [] if p[2] >= SIM_BODY]
    if places:
        worst = max(p[2] for p in places)
        _add(found, 'sim_body', f'{at}{len(places)} places {100 * SIM_BODY:.0f} cm out or more, the worst '
             f'{100 * worst:.1f} cm at ({places[0][0]}, {places[0][1]})', 400 * worst + len(places), 60)
    nav = _count(ev.get('nav_errors'))
    if nav:
        _add(found, 'nav_error', f'{at}{nav} paths FTC gave up', 10 * (nav - 1), 80)
    if ev.get('aborted'):
        _add(found, 'aborted', f'{at}aborted ({ev["aborted"]})')
    area = max(rec['area_m2'], 1e-6)
    more = (ev.get('left_m2') or 0.0) - (ev.get('planned_left_m2', rec['left_m2']) or 0.0)
    if more > max(LEFT_M2, 0.01 * area):
        _add(found, 'sim_left', f'{at}{ev["left_m2"]:.2f} m2 left, {more:.2f} more than planned', 500 * more / area, 60)
    ratio = (ev.get('s') or 0.0) / max(rec['drive_s'], 1e-6)
    if abs(ratio - 1.0) > DURATION:
        _add(found, 'duration', f'{at}{ev.get("s", 0) / 60:.1f} min, drive_time {rec["drive_s"] / 60:.1f} min',
             20 * abs(ratio - 1.0), 30)
    return found


def parse(case: str) -> Tuple[str, int, str, Optional[int]]:
    """garden, area index, mower, seed (None: all of --sim-seeds) of <garden>/<area>@<mower>[#<seed>]."""
    head, _, seed = case.partition('#')
    rest, _, mower = head.rpartition('@')
    name, _, area = rest.rpartition('/')
    if not (name and area.isdigit() and mower and (seed == '' or seed.isdigit())):
        raise ValueError(f'not a case: {case!r} (<garden>/<area>@<mower>#<seed>)')
    return name, int(area), mower, int(seed) if seed else None


def run_case(case: str, opts: argparse.Namespace, planned: Callable[[float], None] = lambda s: None) -> Dict[str, Any]:
    """Plans, checks, simulates and draws one case: its record (JSON)."""
    name, index, mower_name, seed = parse(case)
    garden, mower = gardens.garden(name), gardens.mower(mower_name)
    area_id = garden.areas[index]
    rec: Dict[str, Any] = {'case': case, 'garden': name, 'kind': garden.kind, 'area': area_id, 'mower': mower_name,
                           'outline_m2': round(gardens._outline_m2(garden.map, area_id), 2), 'anomalies': []}
    t0 = time.perf_counter()
    try:
        plan = plan_area(garden, area_id, mower)
    except PlanningError as e:
        planned(time.perf_counter() - t0)
        rec['plan_s'] = round(time.perf_counter() - t0, 2)
        _add(rec['anomalies'], 'refused', str(e))
        return _finish(rec, opts, garden)
    except Exception as e:
        planned(time.perf_counter() - t0)
        rec['plan_s'] = round(time.perf_counter() - t0, 2)
        _add(rec['anomalies'], 'crash', f'{type(e).__name__}: {e} ({_where(e)})')
        rec['traceback'] = traceback.format_exc(limit=-6)
        return _finish(rec, opts, garden)
    rec['plan_s'] = round(time.perf_counter() - t0, 2)
    planned(rec['plan_s'])
    try:
        check = measure(rec, plan)
    except Exception as e:  # (check_plan, export and drive_time are the package's too)
        _add(rec['anomalies'], 'crash', f'checking the plan: {type(e).__name__}: {e} ({_where(e)})')
        rec['traceback'] = traceback.format_exc(limit=-6)
        return _finish(rec, opts, garden, plan)
    sims = [] if opts.no_sim else _simulated(rec, plan, [seed] if seed is not None else range(opts.sim_seeds))
    return _finish(rec, opts, garden, plan, check, sims)


def measure(rec: Dict[str, Any], plan: CoveragePlan) -> Dict[str, Any]:
    """rec with the plan's numbers and anomalies, check_plan's result."""
    check = check_plan(plan)
    ways = kinds_m(plan)
    total = sum(ways.values())
    drive = drive_time(plan.poses)
    left = check.get('left', [])
    free = plan.free_space
    rec.update(
        poses=len(plan.poses), path_m=round(total, 1), sections=len(plan.section_starts),
        free_parts=len(getattr(free, 'geoms', [free])), pieces=len(plan_pieces(plan)),
        area_m2=check.get('area_m2', round(free.area, 3)), angle_deg=round(math.degrees(plan.angle), 1),
        left_m2=check.get('left_m2', 0.0), left_pieces=check.get('left_pieces', 0),
        left_biggest=left[0]['m2'] if left else 0.0, left_at=left[0]['outline'][0] if left else None,
        edge_left_m2=check.get('edge_left_m2', 0.0), double_mowed_pct=check.get('double_mowed_pct'),
        body_out=len(check['body_out']), body_places=group([(b['x'], b['y']) for b in check['body_out']]),
        body_kinds=sorted({b['kind'] for b in check['body_out']}),
        transit_share=round(ways[TRANSIT] / total, 3) if total else 0.0, drive_s=round(drive['s'], 1),
        spin_s=round(drive['spin_s'], 1), hard=check['hard'], soft=check['soft'],
        stats={k: plan.stats.get(k) for k in ('lanes', 'cells', 'loops', 'turns_u_turn', 'turns_bulb', 'turns_detour',
                                               'turns_pivot', 'turns_fallback', 'mop_up', 'chosen', 'timings')},
        **turning(plan))
    reachable = reach(plan)
    pieces = [Polygon(p['outline']).buffer(0).intersection(reachable).area for p in left]
    rec.update(reach_left_m2=round(min(sum(pieces), rec['left_m2']), 3),
               reach_biggest=round(max(pieces, default=0.0), 3))
    if pieces and max(pieces) > 0:
        rec['left_at'] = left[pieces.index(max(pieces))]['outline'][0]
    rec['body_fit'] = {k: (plan.stats.get('body_fit') or {}).get(k) for k in ('fixed', 'left', 'skipped_m')}
    rec['anomalies'] = plan_anomalies(rec, check)
    return check


def _simulated(rec: Dict[str, Any], plan: CoveragePlan, seeds: Sequence[int]) -> List[Any]:
    """rec with the simulation's numbers and anomalies for each seed: [(seed, evaluate's, track)]."""
    sims = []
    if simulate is not None:
        for s in seeds:
            t1 = time.perf_counter()
            try:
                res = simulate(plan, MowerModel.realistic(seed=s))
                ev = evaluate(plan, res)
            except Exception as e:
                _add(rec['anomalies'], 'sim_error', f'seed {s}: {type(e).__name__}: {e} ({_where(e)})')
                continue
            sims.append((s, ev, _track(getattr(res, 'history', ()))))
            rec['anomalies'] += sim_anomalies(rec, ev, s)
            rec.setdefault('sim', []).append(dict(
                {k: ev.get(k) for k in ('s', 'driven_m', 'mowed_m2', 'left_m2', 'left_pieces', 'planned_left_m2',
                                        'tracking', 'spins', 'aborted')},
                seed=s, sim_s=round(time.perf_counter() - t1, 1), nav_errors=_count(ev.get('nav_errors')),
                body_places=(ev.get('body') or {}).get('places', [])[:10],
                body_worst=(ev.get('body') or {}).get('worst')))
    return sims


def _where(e: BaseException) -> str:
    tb = traceback.extract_tb(e.__traceback__)
    return f'{os.path.basename(tb[-1].filename)}:{tb[-1].lineno}' if tb else ''


def _finish(rec: Dict[str, Any], opts: argparse.Namespace, garden: gardens.Garden, plan: Optional[CoveragePlan] = None,
            check: Optional[Dict[str, Any]] = None, sims: Sequence[Any] = ()) -> Dict[str, Any]:
    rec['anomalies'].sort(key=lambda a: -a['score'])
    rec['severity'] = severity(rec['anomalies'])
    if opts.png == 'all' or opts.png == 'flagged' and rec['severity'] >= FLAG:
        rec['png'] = png_name(rec['case'])
        draw(os.path.join(opts.out, rec['png']), garden, rec['area'], rec, plan, check, sims)
    return rec


def png_name(case: str) -> str:
    return re.sub(r'[^A-Za-z0-9.-]+', '_', case) + '.png'


# drawing ------------------------------------------------------------------------------------------------------------

def draw(path: str, garden: gardens.Garden, area_id: str, rec: Dict[str, Any], plan: Optional[CoveragePlan] = None,
         check: Optional[Dict[str, Any]] = None, sims: Sequence[Any] = ()) -> None:
    """The garden (the area planned black, other lawns grey, obstacles filled, paths blue), body_space (dotted) and
    reach (dash-dotted), the plan by segment kind,
    the simulated tracks (black, thin), left unmowed (red: planned, magenta: simulated), body places (red x: planned,
    black o: simulated, sized by how far out), pirouettes (circles), spins (dots)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from plot_coverage import STYLE

    xs, ys = [], []
    for a in garden.map['areas']:
        pts = gardens._outline(a)
        if not pts:
            continue
        kind = a.get('properties', {}).get('type')
        if kind == 'mow' and str(a.get('id')) == area_id:
            xs += [p[0] for p in pts]
            ys += [p[1] for p in pts]
    if not xs:
        return
    w, h = max(xs) - min(xs) + 2, max(ys) - min(ys) + 2
    scale = 11.0 / max(w, h)
    fig, ax = plt.subplots(figsize=(max(5.0, w * scale), max(4.0, h * scale) + 1.2))
    for a in garden.map['areas']:
        pts = gardens._outline(a)
        if len(pts) < 3:
            continue
        ring = pts + pts[:1]
        kind = a.get('properties', {}).get('type')
        if kind == 'mow':
            mine = str(a.get('id')) == area_id
            ax.plot(*zip(*ring), color='black' if mine else '#888888', lw=1.2 if mine else 0.8,
                    ls='-' if mine else '--', zorder=3)
        elif kind == 'obstacle':
            ax.fill(*zip(*ring), color='#9a9a9a', alpha=0.6, lw=0, zorder=1)
        elif kind == 'nav':
            ax.fill(*zip(*ring), color='#9ecae9', alpha=0.5, lw=0, zorder=1)
    for d in garden.map.get('docking_stations', []):
        x, y, hd = d['position']['x'], d['position']['y'], d.get('heading', 0.0)
        ax.annotate('', (x + 0.4 * math.cos(hd), y + 0.4 * math.sin(hd)), (x, y),
                    arrowprops=dict(arrowstyle='->', color='#d62728', lw=1.5), zorder=6)
    if plan is not None:
        for space, style in ((plan.body_space, ':'), (reach(plan), '-.')):
            for poly in getattr(space, 'geoms', [space]) if space is not None else ():
                for ring in [poly.exterior, *poly.interiors]:
                    ax.plot(*ring.xy, color='#555555', lw=0.5, ls=style, zorder=2)
        for kind, style in STYLE.items():
            lines = [[(p.x, p.y) for p in plan.poses[s.start:s.end + 1]] for s in plan.segments if s.kind == kind]
            ax.add_collection(LineCollection(lines, colors=style['color'], linewidths=0.6, zorder=4,
                                             label=style['label']))
        for spin in _spin_points(plan):
            ax.plot(*spin, '.', color='#eb6834', ms=2.5, zorder=5)
    for p in (check or {}).get('left', []):
        ax.fill(*zip(*p['outline']), color='#e3242b', alpha=0.55, lw=0, zorder=5)
    for b in (check or {}).get('body_out', []):
        ax.plot(b['x'], b['y'], 'x', color='#e3242b', ms=6, mew=1.2, zorder=7)
    for x, y in rec.get('pirouettes', []):
        ax.plot(x, y, 'o', mfc='none', mec='black', ms=14, mew=1.2, zorder=7)
    for _, ev, track in sims[:1]:  # (the first seed's)
        for blades, pts in track:
            if len(pts) > 1:
                ax.plot(*zip(*[(p[0], p[1]) for p in pts]), color='black', lw=0.4, ls='-' if blades else ':',
                        alpha=0.8, zorder=6)
        for p in ev.get('left', []):
            ax.fill(*zip(*p['outline']), facecolor='none', edgecolor='#c51b8a', hatch='////', lw=0.6, zorder=5)
        for x, y, m in (ev.get('body') or {}).get('places', []):
            ax.plot(x, y, 'o', mfc='none', mec='black', ms=4 + 100 * m, zorder=7)
    ax.set_aspect('equal')
    ax.set_xlim(min(xs) - 1, max(xs) + 1)
    ax.set_ylim(min(ys) - 1, max(ys) + 1)
    ax.tick_params(labelsize=7)
    ax.set_title(f'{rec["case"]}  severity {rec["severity"]}  ({rec.get("outline_m2", 0):.0f} m2)', fontsize=9)
    text = '\n'.join(f'{a["kind"]} {a["score"]:.0f}: {a["detail"]}'[:150] for a in rec['anomalies'][:6])
    fig.text(0.01, 0.01, text, fontsize=7, va='bottom', family='monospace')
    if plan is not None:
        ax.legend(loc='upper right', fontsize=6, framealpha=0.7)
    fig.subplots_adjust(left=0.06, right=0.98, top=0.95, bottom=0.04 + 0.022 * min(6, len(rec['anomalies'])))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _spin_points(plan: CoveragePlan) -> List[Tuple[float, float]]:
    jumps = {s - 1 for s in plan.section_starts[1:]}
    return [(b.x, b.y) for i, (a, b) in enumerate(zip(plan.poses, plan.poses[1:])) if i not in jumps and (
        math.hypot(b.x - a.x, b.y - a.y) < 1e-6 or abs(math.remainder(b.yaw - a.yaw, 2 * math.pi)) > SPOT_TURN)]


# running ------------------------------------------------------------------------------------------------------------

def _work(case: str, opts: argparse.Namespace, conn: Any) -> None:
    try:
        rec = run_case(case, opts, lambda s: conn.send(('planned', s)))
    except BaseException as e:  # (a garden that can't be made, a mower without its files, a bug of the bench)
        rec = {'case': case, 'anomalies': [], 'bench_error': f'{type(e).__name__}: {e}',
               'traceback': traceback.format_exc(limit=-6), 'severity': 0.0}
    conn.send(('done', rec))
    conn.close()


def run(cases: Sequence[str], opts: argparse.Namespace, log: Callable[[str], None]) -> List[Dict[str, Any]]:
    """Every case in a process of its own, opts.jobs at a time, killed after opts.timeout s planning (or as long
    again checking and simulating)."""
    try:
        ctx = multiprocessing.get_context('fork')
    except ValueError:
        ctx = multiprocessing.get_context()
    todo, running, done = list(cases), {}, []  # running: conn -> [process, case, started, planned at]
    t0 = time.time()

    def kill(conn: Any, why: str, kind: str) -> None:
        proc, case, started, at = running.pop(conn)
        proc.kill()
        proc.join()
        rec = {'case': case, 'anomalies': [], 'plan_s': round((at or time.time()) - started, 1)}
        _add(rec['anomalies'], kind, why)
        name, index, mower, _ = parse(case)
        try:
            garden = gardens.garden(name)
            rec.update(garden=name, kind=garden.kind, area=garden.areas[index], mower=mower,
                       outline_m2=round(gardens._outline_m2(garden.map, garden.areas[index]), 2))
            rec = _finish(rec, opts, garden)
        except Exception as e:
            rec.update(bench_error=str(e), severity=severity(rec['anomalies']))
        report(rec)

    def report(rec: Dict[str, Any]) -> None:
        done.append(rec)
        what = ', '.join(f'{a["kind"]} {a["score"]:.0f}' for a in rec['anomalies'][:4]) or rec.get('bench_error', '')
        log(f'[{len(done)}/{len(cases)} {time.time() - t0:5.0f} s] {rec["case"]:28} {rec.get("plan_s", 0):6.1f} s  '
            f'severity {rec.get("severity", 0):5.1f}  {what}')

    while todo or running:
        while todo and len(running) < opts.jobs:
            here, there = ctx.Pipe(duplex=False)
            proc = ctx.Process(target=_work, args=(todo[0], opts, there), daemon=True)
            proc.start()
            there.close()
            running[here] = [proc, todo.pop(0), time.time(), None]
        for conn in wait(list(running), timeout=1.0):
            try:
                kind, value = conn.recv()
            except (EOFError, OSError):
                proc = running[conn][0]
                proc.join(1.0)
                kill(conn, f'the process died (exit code {proc.exitcode})', 'crash')
                continue
            if kind == 'planned':
                running[conn][3] = time.time()
            else:
                running.pop(conn)[0].join()
                report(value)
        now = time.time()
        for conn, (_, _, started, at) in list(running.items()):
            if at is None and now - started > opts.timeout:
                kill(conn, f'no plan in {opts.timeout:.0f} s', 'timeout')
            elif at is not None and now - at > opts.timeout:
                kill(conn, f'planned in {at - started:.0f} s, checking and simulating took over {opts.timeout:.0f} s',
                     'stuck')
    return done


def cases_of(opts: argparse.Namespace, log: Callable[[str], None]) -> List[Tuple[str, float]]:
    """(case, m2) of the gardens and mowers asked for, the real gardens with every mower (in --quick with its own)."""
    kinds = opts.kinds.split(',') if opts.kinds else list(gardens.KINDS)
    names = gardens.names(kinds, range(opts.seed_from, opts.seed_from + opts.seeds), not opts.no_real,
                          not opts.no_tests)
    own = {'real-mkay': 'mkay', 'real-ape': 'tango'}
    out = []
    for name in names:
        try:
            garden = gardens.garden(name)
        except (ValueError, OSError) as e:
            log(f'{name}: {e}')
            continue
        for i, area in enumerate(garden.areas):
            for mower in opts.mowers.split(','):
                if opts.quick and name in own and own[name] != mower:
                    continue
                out.append((f'{name}/{i}@{mower}', gardens._outline_m2(garden.map, area)))
    return out


def summary(recs: Sequence[Dict[str, Any]], opts: argparse.Namespace, seconds: float, top: int = 40) -> str:
    flagged = [r for r in recs if r.get('severity', 0) >= FLAG]
    planned = [r for r in recs if 'poses' in r]
    by_kind: Dict[str, List[str]] = {}
    for r in recs:
        for a in r['anomalies']:
            by_kind.setdefault(a['kind'], []).append(r['case'])
    sim = f'run with {opts.sim_seeds} seed(s)' if simulate is not None and not opts.no_sim else \
        f'not run ({"--no-sim" if opts.no_sim else "mowbite_planner.simulate: " + SIM_MISSING})'
    out = [f'# simbench {time.strftime("%Y-%m-%d %H:%M")}', '',
           f'{len(recs)} cases, {len(planned)} planned, {len(flagged)} flagged (severity {FLAG:.0f} and up); '
           f'{seconds:.0f} s on {opts.jobs} processes, timeout {opts.timeout:.0f} s. Simulation: {sim}.', '',
           '## Worst cases', '',
           '| # | case | severity | anomalies | m2 | plan s | left m2 (%) | in reach m2 | body places | transit % '
           '| spins | png |', '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for i, r in enumerate(flagged[:top], 1):
        what = '; '.join(f'{a["kind"]} {a["score"]:.0f}' for a in r['anomalies'][:4])
        left = f'{r["left_m2"]:.2f} ({100 * r["left_m2"] / max(r["area_m2"], 1e-6):.1f})' if 'left_m2' in r else ''
        png = f'[png]({r["png"]})' if r.get('png') else ''
        out.append(f'| {i} | `{r["case"]}` | {r["severity"]} | {what} | {r.get("outline_m2", 0):.0f} | '
                   f'{r.get("plan_s", 0):.1f} | {left} | {r.get("reach_left_m2", "")} | '
                   f'{len(r.get("body_places", []))} | '
                   f'{100 * r.get("transit_share", 0):.0f} | {r.get("spins", "")} | {png} |')
    out += ['', '## Anomalies', '', '| kind | cases | e.g. |', '|---|---|---|']
    out += [f'| {k} | {len(v)} | `{v[0]}` |' for k, v in sorted(by_kind.items(), key=lambda kv: -SCORES[kv[0]])]
    out += ['', '## Details of the worst', '']
    for r in flagged[:min(top, 15)]:
        out.append(f'- `{r["case"]}` ({r["severity"]}): ' + '; '.join(f'{a["kind"]}: {a["detail"]}'
                                                                       for a in r['anomalies'][:5]))
    errors = [r for r in recs if r.get('bench_error')]
    if errors:
        out += ['', '## Bench errors (not the planner\'s)', ''] + [f'- `{r["case"]}`: {r["bench_error"]}'
                                                                   for r in errors]
    return '\n'.join(out) + '\n'


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='simbench-out', help='folder for results.json, summary.md, the pictures')
    ap.add_argument('--quick', action='store_true', help='each kind once (seed 0), mkay and tango, the real gardens '
                    'with their own mower, not the tests\'')
    ap.add_argument('--case', action='append', help='this case only (repeatable), <garden>/<area>@<mower>#<seed>')
    ap.add_argument('--kinds', help=f'kinds of made-up gardens, of {",".join(gardens.KINDS)} (default all)')
    ap.add_argument('--seeds', type=int, default=5, help='made-up gardens of each kind (default 5)')
    ap.add_argument('--seed-from', type=int, default=0, help='the first garden seed')
    ap.add_argument('--mowers', default='mkay,tango,rect', help=f'of {",".join(gardens.MOWERS)}')
    ap.add_argument('--no-real', action='store_true', help='not the real gardens')
    ap.add_argument('--no-tests', action='store_true', help='not the gardens of the tests')
    ap.add_argument('--no-sim', action='store_true', help='plans only, no simulation')
    ap.add_argument('--sim-seeds', type=int, default=1, help='simulated mowers per plan (seeds 0..n-1)')
    ap.add_argument('--jobs', type=int, default=os.cpu_count() or 1, help='processes at a time')
    ap.add_argument('--timeout', type=float, default=300.0, help='s for a plan (and for checking and simulating)')
    ap.add_argument('--png', choices=('flagged', 'all', 'none'), default='flagged')
    ap.add_argument('--list', action='store_true', help='the cases only')
    opts = ap.parse_args(argv)
    if opts.quick:
        opts.seeds, opts.seed_from, opts.no_tests = 1, 0, True
        if opts.mowers == ap.get_default('mowers'):
            opts.mowers = 'mkay,tango'

    def log(text: str) -> None:
        print(text, file=sys.stderr, flush=True)

    if opts.case:
        cases = list(opts.case)
        for case in cases:
            parse(case)
        if opts.png == 'flagged':
            opts.png = 'all'
    else:
        found = cases_of(opts, log)
        cases = [c for c, _ in sorted(found, key=lambda cm: -cm[1])]  # the big ones first, they take longest
    if opts.list:
        print('\n'.join(cases))
        return 0
    os.makedirs(opts.out, exist_ok=True)
    log(f'{len(cases)} cases, {opts.jobs} processes, simulation: '
        f'{"off" if opts.no_sim else "on" if simulate is not None else "not available (" + SIM_MISSING + ")"}')
    t0 = time.time()
    recs = sorted(run(cases, opts, log), key=lambda r: (-r.get('severity', 0), r['case']))
    seconds = time.time() - t0
    with open(os.path.join(opts.out, 'results.json'), 'w') as f:
        json.dump({'seconds': round(seconds, 1), 'sim': SIM_MISSING or 'on', 'cases': recs}, f, indent=1)
    with open(os.path.join(opts.out, 'summary.md'), 'w') as f:
        f.write(summary(recs, opts, seconds))
    if opts.case:
        print(json.dumps(recs, indent=1))
    log(f'{len(recs)} cases in {seconds:.0f} s, {sum(r.get("severity", 0) >= FLAG for r in recs)} flagged: '
        f'{os.path.join(opts.out, "summary.md")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
