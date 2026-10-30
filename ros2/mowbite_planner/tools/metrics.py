"""A plan's numbers for the ranking (bench.py adds them to simbench's records), from its poses and the map alone, so
every planner is counted the same way."""
import math
from typing import Any, Dict, List, Sequence

import numpy as np
from shapely.geometry import LineString, Polygon

from mowbite_planner.config import LANE, PERIMETER, TRANSIT, TURN
from mowbite_planner.geometry import polygons
from mowbite_planner.sweep import swept_area

import frozen

SPIN_R, SPIN_MIN = 0.05, math.radians(15.0)
PIR, PIR_REACH, ON_SPOT = 2 * math.pi, 0.6, 0.3
KINK_ANGLE, KINK_LEN = math.radians(45.0), 0.15
DIR_TOL = math.radians(5.0)
BAND = 0.30
PLACE = 0.5


def _xyh(plan) -> List[Sequence[float]]:
    return [(p.x, p.y, p.yaw) for p in plan.poses]


def _sections(plan):
    ends = list(plan.section_starts[1:]) + [len(plan.poses)]
    return list(zip(plan.section_starts, ends))


def spins_geo(P) -> int:
    n, run = 0, 0.0
    for a, b in zip(P, P[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        t = abs(math.remainder(b[2] - a[2], 2 * math.pi))
        if t > 1e-9 and (d < 1e-6 or d / t < SPIN_R):
            run += t
        else:
            n += run >= SPIN_MIN
            run = 0.0
    return n + (run >= SPIN_MIN)


def pirouettes(P, obstacle_points) -> Dict[str, Any]:
    out = {'spot': 0, 'obstacle': 0, 'nothing': 0, 'at': []}
    turned = [0.0]
    for a, b in zip(P, P[1:]):
        turned.append(turned[-1] + math.remainder(b[2] - a[2], 2 * math.pi))
    i = 0
    while i < len(P):
        j, hit = i, 0
        while j + 1 < len(P) and math.hypot(P[j + 1][0] - P[i][0], P[j + 1][1] - P[i][1]) <= PIR_REACH:
            j += 1
            if abs(turned[j] - turned[i]) >= PIR:
                hit = j
                break
        if hit:
            seg = P[i:hit + 1]
            driven = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(seg, seg[1:]))
            if driven < ON_SPOT:
                what = 'spot'
            else:
                ring = Polygon([q[:2] for q in seg]).buffer(0) if len(seg) >= 3 else Polygon()
                what = 'obstacle' if any(ring.contains(o) for o in obstacle_points) else 'nothing'
            out[what] += 1
            if what != 'obstacle':
                out['at'].append([round(P[i][0], 2), round(P[i][1], 2), what])
            i = hit + 1
        else:
            i += 1
    return out


def kinks(P) -> List[Sequence[float]]:
    hits, stretch = [], []

    def flush():
        if len(stretch) >= 2:
            s = np.array([t[0] for t in stretch])
            th = np.array([t[1] for t in stretch])
            cand = np.unique(np.concatenate([s, s - KINK_LEN]))
            cand = cand[(cand >= s[0]) & (cand + KINK_LEN <= s[-1])]
            if len(cand):
                dth = np.abs(np.interp(cand + KINK_LEN, s, th) - np.interp(cand, s, th))
                for c in cand[dth > KINK_ANGLE]:
                    k = min(int(np.searchsorted(s, c + 0.5 * KINK_LEN)), len(stretch) - 1)
                    hits.append((stretch[k][2], stretch[k][3]))
        stretch.clear()

    run, prev_h = 0.0, None
    for a, b in zip(P, P[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        t = abs(math.remainder(b[2] - a[2], 2 * math.pi))
        if d < 1e-9 or (t > 1e-9 and d / t < SPIN_R):  # (on the spot: counted as a spin, ends the stretch)
            flush()
            run, prev_h = 0.0, None
            continue
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        uh = stretch[-1][1] + math.remainder(h - prev_h, 2 * math.pi) if (stretch and prev_h is not None) else h
        stretch.append((run + 0.5 * d, uh, 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])))
        run += d
        prev_h = h
    flush()
    return _group(hits)


def _group(points, near=PLACE):
    out: List[List[float]] = []
    for x, y in points:
        for g in out:
            if math.hypot(g[0] - x, g[1] - y) < near:
                break
        else:
            out.append([round(x, 2), round(y, 2)])
    return out


def lanes(plan) -> Dict[str, Any]:
    angles = list(plan.angles or [plan.angle])
    total = along = 0.0
    runs, in_run = 0, False
    for seg in plan.segments:
        if seg.kind == LANE:
            pts = plan.poses[seg.start:seg.end + 1]
            for a, b in zip(pts, pts[1:]):
                d = math.hypot(b.x - a.x, b.y - a.y)
                if d < 1e-9:
                    continue
                h = math.atan2(b.y - a.y, b.x - a.x)
                total += d
                if any(abs(math.remainder(h - ang, math.pi)) <= DIR_TOL for ang in angles):
                    along += d
            if not in_run:
                runs += 1
                in_run = True
        elif seg.kind == TURN:
            continue
        else:
            in_run = False
    return {'lane_m': round(total, 1), 'lane_dir_share': round(along / total, 4) if total else None,
            'lane_runs': runs}


def dock(plan) -> Dict[str, Any]:
    """m before the first mowing pose (the drive from the dock); m of the shortest way from the last mowing pose to the
    end point the plan was asked for (plan._end_xy, set by bench.py), through its transit space."""
    first = last = None
    for seg in plan.segments:
        if seg.kind in (PERIMETER, LANE):
            first = seg.start if first is None else first
            last = seg.end
    if first is None:
        return {'dock_in_m': None, 'dock_out_m': None}
    din = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(plan.poses[:first], plan.poses[1:first + 1]))
    dout = None
    end = getattr(plan, '_end_xy', None)
    if end is not None and plan.transit_space is not None:
        from mowbite_planner.transit import TransitPlanner
        a = (plan.poses[last].x, plan.poses[last].y)
        try:
            way = TransitPlanner(plan.transit_space.buffer(0.05)).path(a, (float(end[0]), float(end[1])))
        except Exception:  # noqa: BLE001
            way = None
        dout = round(sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(way, way[1:])), 2) if way else \
            round(math.hypot(end[0] - a[0], end[1] - a[1]), 2)
    return {'dock_in_m': round(din, 2), 'dock_out_m': dout}


def rank_numbers(plan, obstacle_points=()) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    s = 0
    pir = {'spot': 0, 'obstacle': 0, 'nothing': 0, 'at': []}
    kk: List[List[float]] = []
    P = _xyh(plan)
    for a, b in _sections(plan):
        part = P[a:b]
        s += spins_geo(part)
        got = pirouettes(part, obstacle_points)
        for k in ('spot', 'obstacle', 'nothing'):
            pir[k] += got[k]
        pir['at'] += got['at']
        kk += kinks(part)
    out.update(spins_geo=s, pir_spot=pir['spot'], pir_obstacle=pir['obstacle'], pir_nothing=pir['nothing'],
               pir_at=pir['at'][:20], kinks=len(kk), kink_at=kk[:30])
    out.update(lanes(plan))
    out.update(dock(plan))
    return out


# against the lawn as drawn and the request ---------------------------------------------------------------------
TUFT = 1e-4  # m2
STRIP = 0.03
BANDS = {'tango': 0.50}  # m, by mower (long nose), else BAND
IDLE_CELL, IDLE_STEP, IDLE_NEW = 0.05, 0.2, 0.10
BODY_PLAN, BODY_DEEP, BODY_CONTACT = 0.01, 0.05, 0.02
TURN_DEG, TURN_WIN = math.radians(90.0), 0.3  # m each way (U-turns and corners, not the rings round trees)
OUT_TOL, DUP = 0.02, 1e-7
BODY_KEYS = ('mower_width', 'blade_offset', 'blade_ahead', 'robot_width', 'robot_front', 'robot_rear',
             'robot_outline', 'edge_margin', 'obstacle_margin', 'spin_margin')
AT_MAX = 20


def request(boundary, obstacles=(), config=None, start=None, end=None, drivable=(), keep_out=(), margins=(),
            mowed=None, reach_over=()) -> Dict[str, Any]:
    """plan_coverage's arguments by name."""
    from mowbite_planner import PlannerConfig
    return {'boundary': boundary, 'obstacles': list(obstacles), 'cfg': config or PlannerConfig(), 'start': start,
            'end': end, 'drivable': list(drivable), 'keep_out': list(keep_out), 'margins': list(margins),
            'reach_over': list(reach_over)}


class Ref:
    """What a plan is measured against, from the map and the request alone (frozen)."""

    def __init__(self, lawn, req: Dict[str, Any], mower: str = ''):
        from shapely.ops import unary_union
        self.lawn, self.cfg, self.end = lawn, req['cfg'], req['end']
        self.band = BANDS.get(mower, BAND)
        self.lines = unary_union([LineString(p.exterior.coords) for p in polygons(lawn)] +
                                 [LineString(r.coords) for p in polygons(lawn) for r in p.interiors])
        self.band_area = self.lines.buffer(self.band, 8).intersection(lawn)
        rings = [req['boundary']] + req['drivable'] + req['reach_over']
        away = req['obstacles'] + req['keep_out']
        allowed = unary_union([Polygon(r).buffer(0) for r in rings if len(r) >= 3])
        holes = [Polygon(r).buffer(0) for r in away if len(r) >= 3]
        self.allowed = allowed.difference(unary_union(holes)) if holes else allowed
        self.rm = self.r0 = None
        if self.cfg.body:
            margin_of = [self.cfg.obstacle_margin if m is None else m for m in req['margins']]
            margin_of += [self.cfg.obstacle_margin] * (len(away) - len(margin_of))
            self.rm = frozen.body_space(rings, away, margin_of, self.cfg)
            self.r0 = frozen.body_space(rings, away, [0.0] * len(away), self.cfg.with_overrides({'edge_margin': 0.0}))


def _as(plan, **kw):
    import dataclasses
    return dataclasses.replace(plan, **kw)


def _rest(lawn, mowed) -> List[Polygon]:
    left = lawn if mowed is None or mowed.is_empty else lawn.difference(mowed)
    return [p for p in polygons(left) if p.area >= TUFT]


def edge_numbers(ref: Ref, rest: List[Polygon]) -> Dict[str, Any]:
    if not rest:
        return {'strip_m': 0.0, 'band_left_m2': 0.0}
    from shapely.ops import unary_union
    left = unary_union(rest)
    wide = left.buffer(-0.5 * STRIP, 4)  # (what's left where a strip STRIP wide fits, grown back to the line)
    strip = 0.0 if wide.is_empty else ref.lines.intersection(wide.buffer(0.5 * STRIP + 0.002, 4)).length
    return {'strip_m': round(strip, 2), 'band_left_m2': round(left.intersection(ref.band_area).area, 4)}


def _sections_of(poses, starts):
    ends = list(starts[1:]) + [len(poses)]
    return [poses[a:b] for a, b in zip(starts, ends)]


def idle(plan, ref: Ref) -> Dict[str, Any]:
    """Rank 10 (and 8) from the blade's strip alone: the lawn on a grid of IDLE_CELL, the way in steps of IDLE_STEP."""
    import shapely
    from mowbite_planner.sweep import blade_points
    w = ref.cfg.mower_width
    x0, y0, x1, y1 = ref.lawn.bounds
    x0, y0 = x0 - w, y0 - w
    nx, ny = int((x1 + w - x0) / IDLE_CELL) + 1, int((y1 + w - y0) / IDLE_CELL) + 1
    gx, gy = np.meshgrid(x0 + (np.arange(nx) + 0.5) * IDLE_CELL, y0 + (np.arange(ny) + 0.5) * IDLE_CELL)
    lawn = shapely.contains_xy(ref.lawn, gx, gy)
    mowed = np.zeros_like(lawn)
    p = _as(plan, config=ref.cfg)
    steps = []  # (m, mows)
    ends = list(plan.section_starts[1:]) + [len(plan.poses)]
    for a, b in zip(plan.section_starts, ends):
        pts = blade_points(p, a, b - 1)
        k = 0
        while k < len(pts) - 1:
            j, d = k, 0.0
            while j < len(pts) - 1 and d < IDLE_STEP:
                d += math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1])
                j += 1
            (ax, ay), (bx, by) = pts[k], pts[j]
            k = j
            length = math.hypot(bx - ax, by - ay)
            if length < 1e-6:
                continue
            ux, uy = (bx - ax) / length, (by - ay) / length
            i0 = max(int((min(ax, bx) - 0.5 * w - x0) / IDLE_CELL), 0)
            i1 = min(int((max(ax, bx) + 0.5 * w - x0) / IDLE_CELL) + 1, nx)
            j0 = max(int((min(ay, by) - 0.5 * w - y0) / IDLE_CELL), 0)
            j1 = min(int((max(ay, by) + 0.5 * w - y0) / IDLE_CELL) + 1, ny)
            if i1 <= i0 or j1 <= j0:
                steps.append((d, False))
                continue
            cx, cy = gx[j0:j1, i0:i1] - ax, gy[j0:j1, i0:i1] - ay
            along, across = cx * ux + cy * uy, np.abs(cx * uy - cy * ux)
            strip = (along >= 0) & (along <= length) & (across <= 0.5 * w)
            n = int(strip.sum())
            here = strip & lawn[j0:j1, i0:i1]
            new = int((here & ~mowed[j0:j1, i0:i1]).sum())
            mowed[j0:j1, i0:i1] |= here
            steps.append((d, n > 0 and new >= IDLE_NEW * n))
    mows = [i for i, (_, m) in enumerate(steps) if m]
    if not mows:
        return {'idle_m': round(sum(d for d, _ in steps), 2), 'dock_in_geo_m': 0.0, 'dock_out_geo_m': 0.0}
    first, last = mows[0], mows[-1]
    return {'idle_m': round(sum(d for d, m in steps[first:last + 1] if not m), 2),
            'dock_in_geo_m': round(sum(d for d, _ in steps[:first]), 2),
            'dock_out_geo_m': round(sum(d for d, _ in steps[last + 1:]), 2)}


def _turning(poses) -> List[bool]:
    """Poses on a turn, from the way alone: the heading turns by TURN_DEG at least within TURN_WIN before and after."""
    s, h = [0.0], [poses[0].yaw] if poses else []
    for a, b in zip(poses, poses[1:]):
        s.append(s[-1] + math.hypot(b.x - a.x, b.y - a.y))
        h.append(h[-1] + math.remainder(b.yaw - a.yaw, 2 * math.pi))
    s, h = np.array(s), np.array(h)
    lo = np.searchsorted(s, s - TURN_WIN, 'left')
    hi = np.searchsorted(s, s + TURN_WIN, 'right') - 1
    out = []
    for i in range(len(poses)):
        part = h[lo[i]:hi[i] + 1]
        out.append(bool(len(part) and part.max() - part.min() >= TURN_DEG))
    return out


def _places(found, near=PLACE):
    out: List[List[float]] = []
    for d, (x, y) in sorted(found, key=lambda f: -f[0]):
        if all(math.hypot(x - px, y - py) > near for px, py, _ in out):
            out.append([round(x, 3), round(y, 3), round(d, 3)])
    return out


def plan_body(plan, ref: Ref) -> Dict[str, Any]:
    from mowbite_planner.footprint import Footprint
    if ref.rm is None:
        return {}
    fp = Footprint(ref.rm, ref.cfg.body_outline, ref.cfg.spin_margin)
    got = {}
    for key, geometric in (('rm_out', False), ('rm_turn_out', True)):
        found = []
        for part in _sections_of(plan.poses, plan.section_starts):
            turn = _turning(part) if geometric else False
            bad = fp.bad_poses(part, turn)
            if bad:
                on = [turn[i] for i in bad] if geometric else False
                found += [(d, xy) for d, xy in fp.out_by([(part[i - 1] if i else None, part[i]) for i in bad],
                                                         on_turn=on) if xy is not None and d > BODY_PLAN]
        places = _places(found)
        got[key] = len(places)
        got[key + '_at'] = places[:AT_MAX]
    return got


def rules(plan, ref: Ref) -> Dict[str, Any]:
    import shapely
    P = plan.poses
    xy = np.array([(p.x, p.y) for p in P]).reshape(-1, 2)
    zone = ref.allowed.buffer(OUT_TOL)
    out = int((~shapely.contains_xy(zone, xy[:, 0], xy[:, 1])).sum()) if len(xy) else 0
    back = dup = 0
    for part in _sections_of(P, plan.section_starts):
        for a, b in zip(part, part[1:]):
            d = math.hypot(b.x - a.x, b.y - a.y)
            if d < DUP:
                dup += 1
            elif d > 1e-3:  # (against the heading at both ends: a turn on the spot at b has b's yaw already)
                h = math.atan2(b.y - a.y, b.x - a.x)
                back += all(abs(math.remainder(h - y, 2 * math.pi)) > math.pi / 2 for y in (a.yaw, b.yaw))
    got = {'centre_out': out, 'backwards': back, 'reverse_segs': sum(1 for s in plan.segments if s.reverse),
           'dup_steps': dup}
    if ref.rm is not None and plan.body_space is not None:
        got['rm_diff_m2'] = round(plan.body_space.symmetric_difference(ref.rm).area, 4)
    if plan.config is not None:
        got['cfg_diff'] = [k for k in BODY_KEYS if getattr(plan.config, k) != getattr(ref.cfg, k)]
    return got


def reply(plan, area_id) -> Dict[str, Any]:
    import json
    from mowbite_planner.export import fingerprint, to_openmower_json, to_poses_json
    result = to_openmower_json(plan, area_id)
    result.update(to_poses_json(plan))
    return {'reply_bytes': len(json.dumps({'jsonrpc': '2.0', 'id': 1, 'result': result}, allow_nan=False)),
            'fp': fingerprint(plan)}


def off_left(plan, ref: Ref) -> Dict[str, Any]:
    """The lawn left where transit segments don't mow (a mower that drives them with the blade off)."""
    from shapely.ops import unary_union
    from mowbite_planner.sweep import blade_points
    p, w = _as(plan, config=ref.cfg), ref.cfg.mower_width
    strips = [LineString(blade_points(p, s.start, s.end)).buffer(0.5 * w, 4)
              for s in plan.segments if s.kind != TRANSIT and s.end > s.start]
    rest = _rest(ref.lawn, unary_union(strips) if strips else None)
    return {'off_left_m2': round(sum(q.area for q in rest), 4)}


def home(plan, ref: Ref) -> Dict[str, Any]:
    """m from the last pose to the end point asked for (the dock) through what's allowed: the drive home (rank 7)."""
    if ref.end is None or not plan.poses:
        return {'home_m': 0.0}
    from mowbite_planner.transit import TransitPlanner
    a, b = (plan.poses[-1].x, plan.poses[-1].y), (float(ref.end[0]), float(ref.end[1]))
    try:
        way = TransitPlanner(ref.allowed).path(a, b)
    except Exception:  # noqa: BLE001
        way = None
    if not way:
        way = [a, b]
    m = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(way, way[1:]))
    return {'home_m': round(m, 2)}


def plan_numbers(plan, ref: Ref, area_id=None) -> Dict[str, Any]:
    out: Dict[str, Any] = {'lawn_m2': round(ref.lawn.area, 3), 'lines_m': round(ref.lines.length, 2),
                           'band_m2': round(ref.band_area.area, 3)}
    rest = _rest(ref.lawn, swept_area(_as(plan, config=ref.cfg), ref.cfg.mower_width))
    left = sum(p.area for p in rest)
    out['lawn_left_m2'] = round(left, 4)
    out.update(edge_numbers(ref, rest))
    way = sum(math.hypot(b.x - a.x, b.y - a.y) for part in _sections_of(plan.poses, plan.section_starts)
              for a, b in zip(part, part[1:]))
    mowed = ref.lawn.area - left
    out['mow_factor'] = round(way * ref.cfg.mower_width / mowed - 1.0, 4) if mowed > 0 else None
    for name, f in (('off', lambda: off_left(plan, ref)), ('idle', lambda: idle(plan, ref)),
                    ('home', lambda: home(plan, ref)),
                    ('body', lambda: plan_body(plan, ref)),
                    ('rules', lambda: rules(plan, ref)), ('reply', lambda: reply(plan, area_id))):
        try:
            out.update(f())
        except Exception as e:  # noqa: BLE001
            out[name + '_error'] = f'{type(e).__name__}: {e}'
    return out


def sim_numbers(plan, samples, ref: Ref) -> Dict[str, Any]:
    from mowbite_planner.simulate import body_places, mowed_area
    out: Dict[str, Any] = {}
    rest = _rest(ref.lawn, mowed_area(_as(plan, free_space=ref.lawn, config=ref.cfg), samples))
    out['lawn_left_m2'] = round(sum(p.area for p in rest), 4)
    out.update(edge_numbers(ref, rest))
    if ref.rm is not None:
        places, _ = body_places(_as(plan, body_space=ref.rm, config=ref.cfg), samples)
        deep = [p for p in places if p[2] >= BODY_DEEP]
        out.update(rm_n5=len(deep), rm_worst=places[0][2] if places else 0.0,
                   rm_excess=round(sum(max(0.0, p[2] - BODY_CONTACT) for p in places), 4), rm_at=deep[:AT_MAX])
        places, _ = body_places(_as(plan, body_space=ref.r0, config=ref.cfg), samples)
        touch = [p for p in places if p[2] >= BODY_CONTACT]
        out.update(r0_n=len(touch), r0_worst=places[0][2] if places else 0.0, r0_at=touch[:AT_MAX])
    return out
