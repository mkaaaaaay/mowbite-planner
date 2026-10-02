"""The plan in the shapes others want it in: OpenMower's paths (PlanPath, the mowing.plan RPC), the
Planner Studio payload, a fingerprint. No ROS, plain lists and dicts (JSON ready)."""

import hashlib
import math
import struct
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .geometry import polygons
from .mathutil import Pose2D
from .planner import LANE, PERIMETER, TRANSIT, TURN, CoveragePlan

# OpenMower counts the poses of a path in an int16 (current_path_index), longer paths are split
MAX_PATH_POSES = 30000
# what the mowing.plan RPC does to points it sends: simplified to 1 cm, rounded to mm
SIMPLIFY = 0.01

SEGMENT_TYPES = {PERIMETER: 0, LANE: 1, TURN: 2, TRANSIT: 3}


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


def to_openmower_json(plan: CoveragePlan, area_id: Optional[str] = None,
                      max_poses: int = MAX_PATH_POSES, tolerance: float = SIMPLIFY) -> Dict[str, Any]:
    """The result of OpenMower's mowing.plan RPC: {area_id, angle, paths: [{is_outline, points,
    pose_index}]}, points simplified (pose_index says which pose of the full path each one is) and rounded
    to mm. On top: every path's segments (kind, first and last pose, reverse, maneuver), the
    fingerprint, warnings and stats."""
    paths = []
    for is_outline, start, end in plan_pieces(plan, max_poses):
        pts = [(p.x, p.y) for p in plan.poses[start:end + 1]]
        keep = simplified_indices(pts, tolerance) if tolerance > 0 else list(range(len(pts)))
        segments = []
        for s in plan.segments:
            a, b = max(s.start, start), min(s.end, end)
            if a < b:
                segments.append({'type': s.kind, 'start': a - start, 'end': b - start, 'reverse': s.reverse,
                                 'maneuver': s.maneuver})
        paths.append({
            'is_outline': is_outline,
            'points': [[_mm(pts[k][0]), _mm(pts[k][1])] for k in keep],
            'pose_index': keep,
            'poses': end - start + 1,
            'segments': segments,
        })
    return {
        'area_id': area_id,
        'angle': plan.angle,
        'angles': plan.angles,
        'lane_spacing': plan.lane_spacing,
        'fingerprint': fingerprint(plan),
        'paths': paths,
        'warnings': list(plan.warnings),
        'stats': {k: (round(v, 4) if isinstance(v, float) else v) for k, v in plan.stats.items()},
    }


def to_poses_json(plan: CoveragePlan) -> Dict[str, Any]:
    """Every pose as [x, y, yaw] and the segments with indices into that list, unsimplified."""
    return {
        'poses': [[p.x, p.y, p.yaw] for p in plan.poses],
        'segments': [{'type': s.kind, 'start': s.start, 'end': s.end, 'reverse': s.reverse,
                      'maneuver': s.maneuver} for s in plan.segments],
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


def to_studio_payload(plan: CoveragePlan, area: Sequence[Sequence[float]],
                      obstacles: Sequence[Sequence[Sequence[float]]] = (),
                      nav_areas: Sequence[Sequence[Sequence[float]]] = (), params: Optional[Dict[str, Any]] = None,
                      ignored: Sequence[str] = (), tolerance: float = SIMPLIFY) -> Dict[str, Any]:
    """The result of ext.planner.plan as OpenMower's Planner Studio (xtech/openmower-app, /planner) shows
    it: stages with their geometry, a lineage of where each object comes from, timings and a report."""
    timings = plan.stats.get('timings', {}) if isinstance(plan.stats.get('timings'), dict) else {}
    objects: List[Dict[str, Any]] = []
    stages: List[Dict[str, Any]] = []

    def stage(key: str, name: str, objs: List[Dict[str, Any]]):
        objects.extend(objs)
        stages.append({'n': len(stages) + 1, 'key': key, 'name': name, 'time_s': timings.get(key, 0.0),
                       'ids': [o['id'] for o in objs]})

    op = lambda name, reason, **p: {'name': name, 'reason': reason, 'params': p}  # noqa: E731
    stage('area', 'Mowable area', _polygon_objects(plan.free_space, 'area', 'mowable', 'area', [],
                                                   op('difference', 'outline minus obstacles')))
    loops = []
    for i, ring in enumerate(plan.perimeter_loops):
        loops.append({'id': f'loop-{i}', 'stage': 'perimeter', 'kind': 'loop',
                      'geometry': {'type': 'line', 'coords': _line(ring, tolerance)},
                      'meta': {'index': i}, 'parents': ['area-0-0'],
                      'op': op('offset', 'loop along the edge', offset=plan.config.first_loop_offset
                               if plan.config else None)})
    stage('perimeter', 'Perimeter loops', loops)
    stage('remaining', 'Remaining area', _polygon_objects(plan.remaining, 'remaining', 'remaining', 'remaining',
                                                          [o['id'] for o in loops],
                                                          op('difference', 'area minus the loops\' strips')))
    lanes = [{'id': f'lane-{i}', 'stage': 'lanes', 'kind': 'lane',
              'geometry': {'type': 'line', 'coords': [[_mm(a[0]), _mm(a[1])], [_mm(b[0]), _mm(b[1])]]},
              'meta': {'index': i, 'length_m': round(math.hypot(b[0] - a[0], b[1] - a[1]), 3)},
              'parents': ['remaining-0-0'], 'op': op('scan', 'parallel lanes', spacing=plan.lane_spacing)}
             for i, (a, b) in enumerate(plan.lanes)]
    stage('lanes', 'Lanes', lanes)
    route = []
    for i, s in enumerate(plan.segments):
        pts = [(p.x, p.y) for p in plan.poses[s.start:s.end + 1]]
        route.append({'id': f'seg-{i}', 'stage': 'route', 'kind': s.kind,
                      'geometry': {'type': 'line', 'coords': _line(pts, tolerance)},
                      'meta': {'maneuver': s.maneuver, 'reverse': s.reverse, 'first_pose': s.start,
                               'last_pose': s.end},
                      'parents': [], 'op': op('route', s.maneuver or s.kind)})
    stage('route', 'Route', route)
    first, last = plan.poses[0], plan.poses[-1]
    stage('ends', 'Start and end', [
        {'id': 'start', 'stage': 'ends', 'kind': 'start', 'geometry': {'type': 'point', 'coords': [[_mm(first.x),
                                                                                                   _mm(first.y)]]},
         'meta': {'yaw': first.yaw}, 'parents': [], 'op': op('start', 'first pose')},
        {'id': 'end', 'stage': 'ends', 'kind': 'end', 'geometry': {'type': 'point', 'coords': [[_mm(last.x),
                                                                                               _mm(last.y)]]},
         'meta': {'yaw': last.yaw}, 'parents': [], 'op': op('end', 'last pose')}])

    stats = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in plan.stats.items() if k != 'timings'}
    stats['plan_time_s'] = round(plan.stats['planning_time'], 4)
    stats['stage_times'] = timings
    stats['fingerprint'] = fingerprint(plan)
    return {
        'angle_deg': math.degrees(plan.angle),
        'params': dict(params or {}),
        'stats': stats,
        'stage_times': timings,
        'report': {'hard': [], 'soft': list(plan.warnings) + [f'parameter {name} is not used by the MowBite '
                                                              'Planner' for name in ignored]},
        'stages': stages,
        'lineage': {'objects': objects, 'events': [
            {'op': s['key'], 'reason': s['name'], 'inputs': [], 'outputs': s['ids'], 'params': {}} for s in stages]},
        'domains': {
            'area': [[float(x), float(y)] for x, y in area],
            'obstacles': [[[float(x), float(y)] for x, y in o] for o in obstacles],
            'nav_areas': [[[float(x), float(y)] for x, y in o] for o in nav_areas],
            'expected_cover_m2': round(plan.free_space.area, 3),
        },
        'mowbite': to_openmower_json(plan, tolerance=tolerance),
    }
