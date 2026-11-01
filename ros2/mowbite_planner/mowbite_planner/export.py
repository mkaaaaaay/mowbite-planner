"""The plan in the shapes others want it in: OpenMower's paths (PlanPath, the mowing.plan RPC), the
Planner Studio payload, a fingerprint. No ROS, plain lists and dicts (JSON ready)."""

import bisect
import hashlib
import math
import struct
from typing import Any, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import Point, Polygon

from .geometry import polygons, rings
from .mathutil import Pose2D
from .config import LANE, PERIMETER, TRANSIT, TURN, CoveragePlan, out_kinds
from .report import check_plan

# OpenMower counts the poses of a path in an int16 (current_path_index), longer paths are split
MAX_PATH_POSES = 30000
# rad, m: OpenMower's FTC takes a path on up to its max_goal_angle_error (20 deg) off the first pose's heading and ends
# one up to max_goal_distance_error (0.2 m) short of its last pose, going on to the next path's first straight: where
# the plan changes between loops and the rest within a section, the cut goes where the way is straight (turning
# PIECE_BEND at most) PIECE_STRAIGHT either side and the body turned PIECE_SWING either way fits from PIECE_STRAIGHT
# before it to PIECE_AFTER after it, the closest to the change within PIECE_REACH m of path (else at the change)
PIECE_SWING = math.radians(20.0)
PIECE_BEND = math.radians(5.0)
PIECE_STRAIGHT = 0.3
PIECE_AFTER = 0.5
PIECE_REACH = 2.0
# what the mowing.plan RPC does to points it sends: simplified to 1 cm, rounded to mm
SIMPLIFY = 0.01

SEGMENT_TYPES = {PERIMETER: 0, LANE: 1, TURN: 2, TRANSIT: 3}
# bytes an answer over MQTT takes at most: Mosquitto drops a message of about 2 MB without a word (Master 31.10.,
# settled with the contracts, I1); a bigger one gets the error ANSWER_TOO_BIG in its place (see too_big)
MAX_ANSWER = 1900000
ANSWER_TOO_BIG = -32002


def plan_pieces(plan: CoveragePlan, max_poses: int = MAX_PATH_POSES) -> List[Tuple[bool, int, int]]:
    """The plan cut into OpenMower paths: (is_outline, first pose, last pose), indices into plan.poses.

    A new piece starts with every section (a part of the area with no way to it, OpenMower drives to the
    start of every path itself), where the plan changes between perimeter loops and the rest (a transit
    goes with what comes after it, so every piece is a continuous drive) and where a piece would get more
    than max_poses poses: a longer run is cut into equal pieces, the next starting on the pose the last
    one ended on. Every piece has at least 3 poses, which OpenMower's FTC controller needs; a shorter
    section is left out (it's a hop of a few cm). So max_poses below 3 counts as 3, and with 3 a run of an
    odd number of steps gets one piece of 4 poses."""
    following = False
    outline = []
    section = None
    for seg in reversed(plan.segments):
        if seg.section != section:
            section, following = seg.section, False
        if seg.kind != TRANSIT:
            following = seg.kind == PERIMETER
        outline.append(following)
    outline.reverse()
    runs: List[list] = []
    for seg, is_outline in zip(plan.segments, outline):
        if runs and runs[-1][0] == is_outline and runs[-1][3] == seg.section:
            runs[-1][2] = seg.end
        else:
            runs.append([is_outline, seg.start, seg.end, seg.section])
    _move_cuts(plan, runs)
    pieces = []
    step = max(2, int(max_poses) - 1)
    for is_outline, start, end, _ in runs:
        if end - start + 1 < 3:
            continue
        # at least 2 steps a piece
        count = min(-(-(end - start) // step), (end - start) // 2)
        cuts = [start + (end - start) * k // count for k in range(count + 1)]
        pieces.extend((is_outline, a, b) for a, b in zip(cuts, cuts[1:]))
    return pieces


def _move_cuts(plan: CoveragePlan, runs: List[list]) -> None:
    """The cuts between runs of a section (sharing a pose) moved where a path is taken on and left well (PIECE_SWING)."""
    cfg, space = getattr(plan, 'config', None), getattr(plan, 'body_space', None)
    if cfg is None or not cfg.body or space is None or len(runs) < 2:
        return
    from .footprint import Footprint, swung
    P = plan.poses
    s, h = [0.0], [P[0].yaw]
    for a, b in zip(P, P[1:]):
        s.append(s[-1] + math.hypot(b.x - a.x, b.y - a.y))
        h.append(h[-1] + math.remainder(b.yaw - a.yaw, 2.0 * math.pi))
    fp = None
    for a, b in zip(runs, runs[1:]):
        if a[3] != b[3] or a[2] != b[1]:
            continue
        at = a[2]
        for c in sorted((c for c in range(a[1] + 2, b[2] - 1) if abs(s[c] - s[at]) <= PIECE_REACH),
                        key=lambda c: (abs(s[c] - s[at]), c)):
            lo, hi = bisect.bisect_left(s, s[c] - PIECE_STRAIGHT), bisect.bisect_right(s, s[c] + PIECE_STRAIGHT) - 1
            if max(h[lo:hi + 1]) - min(h[lo:hi + 1]) > PIECE_BEND:
                continue
            if fp is None:
                fp = Footprint(space, swung(cfg.body_outline, PIECE_SWING))
            if all(fp.fit(P[lo:bisect.bisect_right(s, s[c] + PIECE_AFTER)])):
                a[2] = b[1] = c
                break


def piece_poses(plan: CoveragePlan, start: int, end: int) -> List[Pose2D]:
    return list(plan.poses[start:end + 1])


def fingerprint(plan: CoveragePlan) -> str:
    """SHA-256 of all poses (x, y, yaw as little endian doubles). The planner is deterministic, the same
    request gives the same fingerprint, a changed map or setting a different one (OpenMower compares a
    digest like this to decide whether a saved checkpoint still fits the plan)."""
    h = hashlib.sha256()
    for p in plan.poses:
        h.update(struct.pack('<3d', p.x, p.y, p.yaw))
    return h.hexdigest()


def simplified_indices(points: Sequence[Tuple[float, float]], tolerance: float = SIMPLIFY) -> List[int]:
    """Indices of the points Douglas-Peucker keeps (first and last always)."""
    n = len(points)
    if n <= 2:
        return list(range(n))
    keep = [False] * n
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        (ax, ay), (bx, by) = points[i], points[j]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        best, at = -1.0, -1
        for k in range(i + 1, j):
            px, py = points[k]
            d = abs(dy * (px - ax) - dx * (py - ay)) / length if length > 1e-12 else math.hypot(px - ax, py - ay)
            if d > best:
                best, at = d, k
        if at >= 0 and best > tolerance:
            keep[at] = True
            stack.append((i, at))
            stack.append((at, j))
    return [k for k in range(n) if keep[k]]


def _mm(v: float) -> float:
    return round(v, 3)


def too_big(data: str) -> Optional[str]:
    """The error message for an answer (its JSON, ASCII as json.dumps makes it) of more than MAX_ANSWER bytes, None
    where it's no bigger."""
    if len(data) <= MAX_ANSWER:
        return None
    return (f'answer too big: {len(data) / 1e6:.2f} MB, MQTT takes {MAX_ANSWER / 1e6:.1f} MB at most (poses, report '
            'and trace make it bigger)')


def body_edges(plan: CoveragePlan, tolerance: float = SIMPLIFY) -> Optional[Dict[str, List[List[List[float]]]]]:
    """Where the mower's body keeps to (body_space: the real edges, the margins off them): {'outlines':
    [[[x, y], ...], ...], 'holes': [...]}, the rings simplified to tolerance and rounded to mm, the holes the obstacles
    and areas to keep out of. None without the body checked."""
    if plan.body_space is None:
        return None
    out: Dict[str, List[List[List[float]]]] = {'outlines': [], 'holes': []}
    for kind, ring in rings(plan.body_space.simplify(tolerance, preserve_topology=True)):
        out['outlines' if kind == 'outline' else 'holes'].append([[_mm(x), _mm(y)] for x, y in ring])
    return out


def to_openmower_json(plan: CoveragePlan, area_id: Optional[str] = None,
                      max_poses: int = MAX_PATH_POSES, tolerance: float = SIMPLIFY) -> Dict[str, Any]:
    """The result of OpenMower's mowing.plan RPC: {area_id, angle, paths: [{is_outline, points,
    pose_index}]}, points simplified (pose_index says which pose of the full path each one is) and rounded
    to mm. On top: every path's segments (kind as MowBite OS gets it, config.out_kinds; first and last pose, reverse,
    maneuver), the fingerprint, warnings and stats, with the body checked where it keeps to (body_space, see
    body_edges)."""
    paths = []
    kinds = out_kinds(plan.segments)
    for is_outline, start, end in plan_pieces(plan, max_poses):
        pts = [(p.x, p.y) for p in plan.poses[start:end + 1]]
        keep = simplified_indices(pts, tolerance) if tolerance > 0 else list(range(len(pts)))
        segments = []
        for s, kind in zip(plan.segments, kinds):
            a, b = max(s.start, start), min(s.end, end)
            if a < b:
                segments.append({'type': kind, 'start': a - start, 'end': b - start, 'reverse': s.reverse,
                                 'maneuver': s.maneuver})
        paths.append({
            'is_outline': is_outline,
            'points': [[_mm(pts[k][0]), _mm(pts[k][1])] for k in keep],
            'pose_index': keep,
            'poses': end - start + 1,
            'segments': segments,
        })
    out: Dict[str, Any] = {
        'area_id': area_id,
        'angle': plan.angle,
        'angles': plan.angles,
        'lane_spacing': plan.lane_spacing,
        'fingerprint': fingerprint(plan),
        'paths': paths,
        'warnings': list(plan.warnings),
        'stats': {k: (round(v, 4) if isinstance(v, float) else v) for k, v in plan.stats.items()},
    }
    edges = body_edges(plan, tolerance)
    if edges is not None:
        out['body_space'] = edges
    return out


def to_poses_json(plan: CoveragePlan) -> Dict[str, Any]:
    """Every pose as [x, y, yaw] and the segments with indices into that list, unsimplified, x and y rounded to mm,
    yaw to 1e-4 rad (Master 31.10.: the answer a third as big; the fingerprint stays the unrounded poses')."""
    return {
        'poses': [[_mm(p.x), _mm(p.y), round(p.yaw, 4)] for p in plan.poses],
        'segments': [{'type': kind, 'start': s.start, 'end': s.end, 'reverse': s.reverse,
                      'maneuver': s.maneuver} for s, kind in zip(plan.segments, out_kinds(plan.segments))],
    }


# Planner Studio ------------------------------------------------------------------------------------------


def _ring_coords(ring) -> List[List[float]]:
    return [[_mm(x), _mm(y)] for x, y in ring.coords]


def _line(points: Sequence[Tuple[float, float]], tolerance: float) -> List[List[float]]:
    keep = simplified_indices(points, tolerance)
    return [[_mm(points[k][0]), _mm(points[k][1])] for k in keep]


def _polygon_objects(geom, stage: str, kind: str, prefix: str, parents: List[str], op: Dict[str, Any]):
    out = []
    for i, poly in enumerate(polygons(geom)):
        for j, ring in enumerate([poly.exterior, *poly.interiors]):
            out.append({'id': f'{prefix}-{i}-{j}', 'stage': stage, 'kind': kind if j == 0 else kind + '_hole',
                        'geometry': {'type': 'polygon', 'coords': _ring_coords(ring)},
                        'meta': {'area_m2': round(poly.area, 3)} if j == 0 else {}, 'parents': parents, 'op': op})
    return out


def lineage(plan: CoveragePlan, tolerance: float = SIMPLIFY) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]],
                                                                    List[Dict[str, Any]]]:
    """Where everything in the plan comes from, stage by stage: (stages, objects, events). Objects have an id,
    their stage, kind, geometry, meta, the ids they were made from (parents) and how (op: name, reason,
    params). The mowable area, the loop levels (level-<k>), what's left for the lanes, the cells
    (cell-<set>-<index>), the loops (loop-<i>) and lanes (lane-<i>) as driven, every segment of the route
    (seg-<i>, its parent the loop or lane it mows or leads to), start and end."""
    cfg = plan.config
    timings = plan.stats.get('timings', {}) if isinstance(plan.stats.get('timings'), dict) else {}
    objects: List[Dict[str, Any]] = []
    stages: List[Dict[str, Any]] = []
    events: List[Dict[str, Any]] = []

    def op(name: str, reason: str, **params) -> Dict[str, Any]:
        return {'name': name, 'reason': reason, 'params': params}

    def stage(key: str, name: str, objs: List[Dict[str, Any]], what: Dict[str, Any], inputs: List[str]):
        objects.extend(objs)
        # (the planner times levels as perimeter, cells as lanes, the rest goes with route)
        took = timings.get({'levels': 'perimeter', 'cells': 'lanes'}.get(key, key), 0.0)
        stages.append({'n': len(stages) + 1, 'key': key, 'name': name, 'time_s': took,
                       'ids': [o['id'] for o in objs]})
        events.append({'op': what['name'], 'reason': what['reason'], 'inputs': inputs,
                       'outputs': [o['id'] for o in objs], 'params': what['params']})

    def line(pts) -> Dict[str, Any]:
        return {'type': 'line', 'coords': _line(pts, tolerance)}

    first = cfg.blade_loop_offset if cfg else 0.0
    spacing = plan.lane_spacing
    area_op = op('difference', 'the outline minus the obstacles, simplified',
                 simplify_tolerance=cfg.simplify_tolerance if cfg else None)
    area = _polygon_objects(plan.free_space, 'area', 'mowable', 'area', [], area_op)
    stage('area', 'Mowable area', area, area_op, [])
    area_ids = [o['id'] for o in area]

    # the loop levels, one object per ring: level-<k> its first, level-<k>.<n> the others
    levels, level_rings = [], {}
    level_op = op('inset', 'the area shrunk to where the blade runs on loop level k')
    for k, geom in enumerate(plan.levels):
        n = 0
        for poly in polygons(geom):
            for ring in [poly.exterior, *poly.interiors]:
                oid = f'level-{k}' if n == 0 else f'level-{k}.{n}'
                level_rings.setdefault(k, []).append((oid, ring))
                levels.append({'id': oid, 'stage': 'levels', 'kind': 'level', 'geometry': {
                    'type': 'polygon', 'coords': _ring_coords(ring)}, 'meta': {'level': k},
                    'parents': area_ids, 'op': op('inset', 'loop level', offset=round(first + k * spacing, 4))})
                n += 1
    stage('levels', 'Loop levels', levels, level_op, area_ids)

    remaining_op = op('difference', "the area minus the loops' strips, what the lanes mow")
    rest = _polygon_objects(plan.remaining, 'remaining', 'remaining', 'remaining', area_ids, remaining_op)
    region = _polygon_objects(plan.lane_region, 'remaining', 'lane_region', 'lane-region', area_ids,
                              op('inset', 'where the lanes lie', offset=round(
                                  first + max(0, (cfg.passes - cfg.lane_overlap_passes)) * spacing, 4)
                                 if cfg else None))
    narrow = _polygon_objects(plan.narrow, 'remaining', 'narrow', 'narrow', [o['id'] for o in levels],
                              op('middle', 'too narrow for lanes, a loop of its own')) \
        if plan.narrow is not None and not plan.narrow.is_empty else []
    stage('remaining', 'Left for the lanes', rest + region + narrow, remaining_op, area_ids)
    region_ids = [o['id'] for o in region]

    cells = []
    for pi, (angle, pass_cells) in enumerate(plan.cells):
        c, s = math.cos(angle), math.sin(angle)
        for ci, cell in enumerate(pass_cells):
            if not cell:
                continue
            ends = [(x * c - lane.y * s, x * s + lane.y * c) for lane in cell for x in (lane.x0, lane.x1)]
            hull = Polygon(ends).convex_hull if len(ends) >= 3 else None
            coords = _ring_coords(hull.exterior) if hull is not None and hull.geom_type == 'Polygon' else \
                [[_mm(x), _mm(y)] for x, y in ends]
            cells.append({'id': f'cell-{pi}-{ci}', 'stage': 'cells', 'kind': 'cell',
                          'geometry': {'type': 'polygon' if len(coords) >= 3 else 'line', 'coords': coords},
                          'meta': {'set': pi, 'angle_deg': round(math.degrees(angle), 2), 'lanes': len(cell),
                                   'bends': sum(len(lane.bends) for lane in cell)},
                          'parents': region_ids, 'op': op('split', 'lanes mowed in one zigzag (a row that splits or '
                                                                   'joins starts a new cell)',
                                                          spacing=round(spacing, 4))})
    stage('cells', 'Cells', cells, op('scan', 'parallel lanes, a lane spacing apart, grouped into cells',
                                      spacing=round(spacing, 4)), region_ids)

    # the loops and lanes as driven, from their segments
    driven: Dict[str, List[Tuple[float, float]]] = {}
    for seg in plan.segments:
        if seg.source and seg.kind in (PERIMETER, LANE):
            pts = [(p.x, p.y) for p in plan.poses[seg.start:seg.end + 1]]
            driven[seg.source] = driven.get(seg.source, [])[:-1] + pts if seg.source in driven else pts
    loops, lanes = [], []
    for name, origin in sorted(plan.origins.items(), key=lambda kv: (kv[0].split('-')[0],
                                                                      int(kv[0].split('-')[1]))):
        pts = driven.get(name)
        if not pts or len(pts) < 2:
            continue
        parent = origin.get('parent', '')
        start = Point(pts[0])
        made = parent
        if parent.startswith(('level-', 'middle-')) and level_rings.get(origin.get('level')):
            # the ring of its level it runs along (a ring halfway in: of the next level in)
            parent = min(level_rings[origin['level']], key=lambda r: r[1].distance(start))[0]
        elif parent == 'narrow' and narrow:
            parent = 'narrow-%d-0' % min(enumerate(polygons(plan.narrow)), key=lambda p: p[1].distance(start))[0]
        obj = {'id': name, 'stage': 'loops' if name.startswith('loop-') else 'lanes', 'kind': origin.get('kind', ''),
               'geometry': line(pts), 'meta': {k: v for k, v in origin.items() if k != 'parent'},
               'parents': [parent] if parent else []}
        if name.startswith('loop-'):
            obj['op'] = op('loop', 'along the edge of its level' if made.startswith('level-') else
                           'around a strip too narrow for lanes' if made == 'narrow' else 'a ring halfway in')
            loops.append(obj)
        else:
            obj['op'] = op('lane', 'a lane of its cell' + (', bending around an obstacle'
                                                          if origin.get('kind') == 'bent' else ''))
            lanes.append(obj)
    stage('loops', 'Perimeter loops', loops, op('loop', 'the rings of every level, closest first'),
          [o['id'] for o in levels])
    stage('lanes', 'Lanes', lanes, op('zigzag', 'every cell in a zigzag', lane_order=cfg.lane_order if cfg else None,
                                      route_order=cfg.route_order if cfg else None), [o['id'] for o in cells])

    route = []
    for i, seg in enumerate(plan.segments):
        pts = [(p.x, p.y) for p in plan.poses[seg.start:seg.end + 1]]
        reason = {PERIMETER: 'mows a loop', LANE: 'mows a lane', TURN: f'turns into the next lane ({seg.maneuver})',
                  TRANSIT: 'drives to what comes next' + (f' ({seg.maneuver})' if seg.maneuver else '')}[seg.kind]
        route.append({'id': f'seg-{i}', 'stage': 'route', 'kind': seg.kind, 'geometry': line(pts),
                      'meta': {'maneuver': seg.maneuver, 'reverse': seg.reverse, 'first_pose': seg.start,
                               'last_pose': seg.end, 'section': seg.section},
                      'parents': [seg.source] if seg.source else [], 'op': op('route', reason)})
    stage('route', 'Route', route, op('route', 'the loops and lanes joined by turns and drives'),
          [o['id'] for o in loops + lanes])
    if plan.poses:
        first_pose, last_pose = plan.poses[0], plan.poses[-1]
        stage('ends', 'Start and end', [
            {'id': 'start', 'stage': 'ends', 'kind': 'start', 'geometry': {
                'type': 'point', 'coords': [[_mm(first_pose.x), _mm(first_pose.y)]]},
             'meta': {'yaw': first_pose.yaw}, 'parents': ['seg-0'] if route else [], 'op': op('start', 'first pose')},
            {'id': 'end', 'stage': 'ends', 'kind': 'end', 'geometry': {
                'type': 'point', 'coords': [[_mm(last_pose.x), _mm(last_pose.y)]]},
             'meta': {'yaw': last_pose.yaw}, 'parents': [route[-1]['id']] if route else [],
             'op': op('end', 'last pose')}], op('ends', 'where the path starts and ends'), [])
    return stages, objects, events


def to_trace_json(plan: CoveragePlan, tolerance: float = SIMPLIFY) -> Dict[str, Any]:
    """{stages, lineage: {objects, events}}, see lineage()."""
    stages, objects, events = lineage(plan, tolerance)
    return {'stages': stages, 'lineage': {'objects': objects, 'events': events}}


def to_studio_payload(plan: CoveragePlan, area: Sequence[Sequence[float]],
                      obstacles: Sequence[Sequence[Sequence[float]]] = (),
                      nav_areas: Sequence[Sequence[Sequence[float]]] = (), params: Optional[Dict[str, Any]] = None,
                      ignored: Sequence[str] = (), tolerance: float = SIMPLIFY) -> Dict[str, Any]:
    """The result of ext.planner.plan as OpenMower's Planner Studio (xtech/openmower-app, /planner) shows
    it: stages with their geometry, a lineage of where each object comes from, timings and a report."""
    timings = plan.stats.get('timings', {}) if isinstance(plan.stats.get('timings'), dict) else {}
    stages, objects, events = lineage(plan, tolerance)
    report = check_plan(plan)
    stats = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in plan.stats.items() if k != 'timings'}
    stats['plan_time_s'] = round(plan.stats['planning_time'], 4)
    stats['stage_times'] = timings
    stats['fingerprint'] = fingerprint(plan)
    for key in ('mowed_m2', 'left_m2', 'double_mowed_pct'):
        if key in report:
            stats[key] = report[key]
    return {
        'angle_deg': math.degrees(plan.angle),
        'params': dict(params or {}),
        'stats': stats,
        'stage_times': timings,
        'report': {'hard': report['hard'], 'soft': report['soft'] + [
            f'parameter {name} is not used by the MowBite Planner' for name in ignored]},
        'stages': stages,
        'lineage': {'objects': objects, 'events': events},
        'domains': {
            'area': [[float(x), float(y)] for x, y in area],
            'obstacles': [[[float(x), float(y)] for x, y in o] for o in obstacles],
            'nav_areas': [[[float(x), float(y)] for x, y in o] for o in nav_areas],
            'expected_cover_m2': round(plan.free_space.area, 3),
        },
        'mowbite': to_openmower_json(plan, tolerance=tolerance),
    }
