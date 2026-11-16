"""Where the mower may be, from the map and the settings alone: the mowing area, the spaces of its centre and body."""

import dataclasses
import hashlib
import math
from typing import Dict, List, Optional, Sequence, Tuple

from shapely import wkb
from shapely.geometry import LineString, Polygon
from shapely.ops import clip_by_rect, nearest_points

from .config import (_BODY_SPACE, _BODY_SPACES, _KEPT_LOCK, DRAWN_CENTRE, DRAWN_HARD, LARGER, MAX_BLADE_OFFSET,
                     ON_EDGE, RECORDED_KEEP, RECORDED_MITRE, RECORDED_OFF, RECORDED_SLACK, XY, PlannerConfig,
                     PlanningError, _keep)
from .footprint import TURN_STEP, Blade, Footprint
from .geometry import JOIN_MITRE, QUAD_SEGS, clean_polygon, inset, polygonal, polygons, rings, subtract, union
from .mathutil import normalize_angle


# m, coordinates further out are a mistake (or the wrong frame), not a lawn
MAX_COORDINATE = 1e7


# m, how far apart _edge_gaps closes a slot by the edge straight across
EDGE_STEP = 0.03


# m2, with the body checked (edges recorded) an obstacle of which less than this is left once the body's band round its
# line is taken off (smaller than the body is wide: it can't have been driven round with the body in it) was drawn: its
# line is its real edge, the body keeps its margin off all of it (_body_space)
DRAWN = 1e-4


def _check_coordinates(what: str, points: Sequence[Sequence[float]]) -> None:
    """Refuses points that are no numbers or lie absurdly far out (GEOS can take forever on those)."""
    for p in points:
        # false for nan too
        if not all(abs(c) <= MAX_COORDINATE for c in p[:2]):
            raise PlanningError(f'{what} has a point that is no number or more than {MAX_COORDINATE:.0f} m from '
                                f'the map origin: {list(p[:2])}')


def _edge_slot_count(boundary, obstacles, cfg: PlannerConfig, drivable, keep_out, margins, reach_over) -> int:
    """How many slots by the edge _plan_coverage closes (see _edge_gaps), worked out the same way before planning."""
    if not cfg.body:
        return 0
    area = clean_polygon(boundary)
    away_from = [clean_polygon(w) for w in list(obstacles) + list(keep_out)]
    margin_of = [cfg.obstacle_margin if m is None else m for m in margins]
    margin_of += [cfg.obstacle_margin] * (len(obstacles) + len(keep_out) - len(margin_of))
    return sum(1 for g in _edge_gaps(area, list(drivable) + list(reach_over), away_from, margin_of, cfg)
               if g.intersects(area))


def _gaps(shapes: List[object], margin_of: List[float], off: List[float], cfg: PlannerConfig,
          walls: Sequence[float] = ()) -> List[Polygon]:
    """The gaps between two obstacles or areas to keep out of where the body, facing straight across, has less than
    body_tolerance to spare with each one's margin kept (their real edges robot_width / 2 inside the lines, which lie
    about body_tolerance off where the mower was; hard: the lines): the hull round the parts of either closer than that
    to the other, less both. Not where the centre keeps off already (off: how far from each one's line). The loops go
    round both there, nothing drives in where the body would get through facing straight across at best. walls: each
    one's, see _walls."""
    walls = list(walls) or [0.0] * len(shapes)
    gaps = []
    for i, a in enumerate(shapes):
        for j in range(i + 1, len(shapes)):
            # (hard edges: the real edges are the lines, the body's whole width between them)
            b, width = shapes[j], margin_of[i] + margin_of[j] + cfg.body_tolerance + 2.0 * cfg.wall_half + \
                walls[i] + walls[j]
            if not a.is_empty and not b.is_empty and off[i] + off[j] <= a.distance(b) < width:
                near = union([a.intersection(b.buffer(width, QUAD_SEGS)), b.intersection(a.buffer(width, QUAD_SEGS))])
                gaps += [p for p in polygons(near.convex_hull.difference(union([a, b])))
                         if p.distance(a) < ON_EDGE and p.distance(b) < ON_EDGE]
    return gaps


def _edge_gaps(area, drivable: Sequence[Sequence[Sequence[float]]], shapes: List[object], margin_of: List[float],
               cfg: PlannerConfig, walls: Sequence[float] = (), wall: float = 0.0) -> List[Polygon]:
    """The slots between an obstacle or area to keep out of and the edge (where no area the mower may drive through
    lies beyond it: the edge's real edge robot_width / 2 out from its line, hard: the line) where the body, facing
    straight across, has less than body_tolerance to spare with the margins kept, like _gaps: all of the slot,
    straight across from the one to the other, as far as it stays narrower than that by half the body's width more
    (beside a closed bit it's a dead end the mower can't turn in). The loops go round the obstacle there as round one
    on the edge: a bed moved a little off the edge doesn't take the outermost loop through a slot by it. Also where it
    comes closer to the edge than the loops' centres keep off both (no way between them then): the wedges either side
    of where it does are such slots, the loops would run into them. walls: each one's, wall: the edge's, see _walls."""
    reach = union([area] + [clean_polygon(d) for d in drivable])
    beyond = reach.buffer(1.0, QUAD_SEGS).difference(reach)
    ys = [y for _, y in cfg.body_outline]
    gaps = []
    for a, m, w in zip(shapes, margin_of, list(walls) or [0.0] * len(shapes)):
        width = m + cfg.edge_margin + cfg.body_tolerance + 2.0 * cfg.wall_half + w + wall
        if a.is_empty or not a.distance(beyond) < width:
            continue
        across = width + 0.5 * (max(ys) - min(ys))
        bridges = []
        for p in polygons(a):
            n = max(8, int(p.exterior.length / EDGE_STEP))
            for k in range(n):
                q = p.exterior.interpolate(k * p.exterior.length / n)
                if q.distance(beyond) < across:
                    bridges.append(LineString([q, nearest_points(q, beyond)[1]]).buffer(EDGE_STEP, QUAD_SEGS))
        gaps += [g for g in polygons(union(bridges).difference(union([a, beyond])))
                 if g.distance(a) < ON_EDGE and g.distance(beyond) < ON_EDGE]
    return gaps


def _recorded(bodies: Sequence[Tuple[Sequence[Sequence[float]], Sequence[XY]]]) -> Dict[tuple, Tuple[XY, ...]]:
    """{ring: the body's corners, or DRAWN_HARD or DRAWN_CENTRE} of bodies ((ring, corners), ...), the rings as
    _body_space has them."""
    return {tuple((float(p[0]), float(p[1])) for p in ring):
            corners if isinstance(corners, str) else tuple((float(x), float(y)) for x, y in corners)
            for ring, corners in bodies}


def drawn_ring(bodies, ring) -> Optional[str]:
    """DRAWN_HARD or DRAWN_CENTRE where bodies (plan_coverage's) have ring as drawn, else None."""
    got = _recorded(bodies).get(tuple((float(p[0]), float(p[1])) for p in ring))
    return got if isinstance(got, str) else None


def _bands(rings, corners_of, own: Tuple[XY, ...]):
    """The rings and the bodies _driven takes of rings with each one's corners (corners_of, _recorded's): none of a
    ring drawn hard, a ring drawn as the centre track with this body (own) both ways."""
    out_rings, out_bodies = [], []
    for ring, corners in zip(rings, corners_of):
        if corners == DRAWN_HARD:
            continue
        if corners == DRAWN_CENTRE:
            out_rings += [ring, list(ring)[::-1]]
            out_bodies += [own, own]
        else:
            out_rings.append(ring)
            out_bodies.append(corners)
    return out_rings, out_bodies


def _walls(boundary, away_from: List[Sequence[Sequence[float]]], cfg: PlannerConfig,
           bodies) -> Tuple[float, List[float]]:
    """m the loops keep further off the outline, and off each obstacle or area to keep out of, recorded with a narrower
    body than this one (bodies): half the body's extra width, as with hard edges for one of none (wall_half); off one
    drawn hard half the body's width."""
    own = _recorded(bodies) if cfg.body and cfg.edges != 'hard' else {}
    side = max(abs(y) for _, y in cfg.body_outline) if own else 0.0

    def wall(ring) -> float:
        corners = own.get(tuple((float(p[0]), float(p[1])) for p in ring))
        if corners is None or corners == DRAWN_CENTRE:
            return 0.0
        return side if corners == DRAWN_HARD else max(0.0, side - max(abs(y) for _, y in corners))
    return wall(boundary), [wall(w) for w in away_from]


def _body_space(outlines: List[Sequence[Sequence[float]]], away_from: List[Sequence[Sequence[float]]],
                margin_of: List[float], cfg: PlannerConfig, blade: Optional[Blade] = None,
                bodies: Sequence[Tuple[Sequence[Sequence[float]], Sequence[XY]]] = ()) -> Tuple[object, Footprint]:
    """Where the body may be, and its Footprint there (with blade: the blade kept off the areas not to mow): where the
    edges really are, where the body was while its centre drove them (the outline and the drivable areas, the obstacles
    and areas to keep out of, each the way its points go), into an obstacle or an area to keep out of only along its own
    edge (hard edges: the lines themselves; one too small to have been driven round, of which that leaves less than
    DRAWN, its line), edge_margin and each one's margin off that. bodies ((ring, corners), ...): the body a ring was
    recorded with where it's another (the map's recorded_body), the Footprint's is this one's; DRAWN_HARD in place of
    the corners: the ring's line is its edge (as with hard edges, no other's band taken off an obstacle drawn so),
    DRAWN_CENTRE: the body was where this one's would be with its centre on the line either way. The last ones worked
    out are kept (_BODY_SPACES), the last one of this thread with its Footprint (_BODY_SPACE), the plan gets a copy."""
    outlines, away_from = ([[(float(p[0]), float(p[1])) for p in ring] for ring in rings]
                           for rings in (outlines, away_from))
    hard = cfg.edges == 'hard'
    key = (outlines, away_from, tuple(margin_of), cfg.edge_margin, cfg.body_outline, hard)
    own = {} if hard else _recorded(bodies)
    out_bodies, away_bodies = ([own.get(tuple(ring), cfg.body_outline) for ring in rings]
                               for rings in (outlines, away_from))
    if any(c != cfg.body_outline for c in out_bodies + away_bodies):
        key += (tuple(out_bodies), tuple(away_bodies))
    found = getattr(_BODY_SPACE, 'last', None)
    if found is None or found[0] != (key, cfg.spin_margin, blade and blade.key):
        shared = hashlib.sha1(repr(key).encode()).hexdigest()
        with _KEPT_LOCK:
            space = _BODY_SPACES.get(shared)
        if space is None:
            outer = union([clean_polygon(o) for o in outlines] +
                          ([] if hard else [_driven(*_bands(outlines, out_bodies, cfg.body_outline))]))
            band = None if hard else _driven(*_bands(away_from, away_bodies, cfg.body_outline))
            inner = []
            for w, corners in zip(away_from, away_bodies):
                w = clean_polygon(w)
                rest = w if band is None or corners == DRAWN_HARD else w.difference(band)
                inner.append(rest if rest.area >= DRAWN else w)
            space = subtract(inset(outer, cfg.edge_margin), [inset(w, -m) for w, m in zip(inner, margin_of)]).wkb
            _keep(_BODY_SPACES, shared, space)
        space = wkb.loads(space)
        found = _BODY_SPACE.last = ((key, cfg.spin_margin, blade and blade.key), space,
                                    Footprint(space, cfg.body_outline, cfg.spin_margin, blade=blade))
    return wkb.loads(found[1].wkb), found[2]


def recorded_space(outlines: List[Sequence[Sequence[float]]], away_from: List[Sequence[Sequence[float]]],
                   margin_of: List[float], cfg: PlannerConfig, bodies=()):
    """Where the body may be on the recorded loop (recorded_loop): where it was while the edges were recorded
    (_body_space without edge_margin, RECORDED_SLACK past it: on the line the body touches its edge), the obstacles'
    margins kept. recorded_loop_margin isn't taken off it: the loop runs that far inside its line, its body's corners
    at a bend of that less far in than where they were (a space that much smaller the loop itself wouldn't fit in).
    drive reverse: the body turned round went the other way. This thread's kept space stays as it was."""
    if cfg.drive == 'reverse':
        outlines, away_from = ([list(ring)[::-1] for ring in rings] for rings in (outlines, away_from))
        bodies = [(list(ring)[::-1], corners if isinstance(corners, str) else [(-x, -y) for x, y in corners])
                  for ring, corners in bodies]
    last = getattr(_BODY_SPACE, 'last', None)
    try:
        return _body_space(outlines, away_from, margin_of, dataclasses.replace(cfg, edge_margin=-RECORDED_SLACK),
                           None, bodies)[0]
    finally:
        _BODY_SPACE.last = last


def loop_footprint(space, cfg: PlannerConfig, blade: Optional[Blade] = None) -> Footprint:
    """The body's check on the recorded loop: in its space (recorded_space), turning on the spot there too (no
    spin_margin on top: the mower turned there while the line was recorded)."""
    return Footprint(space, cfg.body_outline, 0.0, blade=blade)


def recorded_larger(boundary, cfg: PlannerConfig, bodies) -> bool:
    """Whether the body is larger than the one the outline was recorded with (bodies: plan_coverage's; none for
    it: recorded with this one), as mapfile.body_hints has it: not in that one grown by LARGER."""
    corners = _recorded(bodies).get(tuple((float(p[0]), float(p[1])) for p in boundary))
    if corners is None or isinstance(corners, str):
        return False
    if cfg.drive == 'reverse':  # (cfg's body is turned round, see backing.settings)
        corners = [(-x, -y) for x, y in corners]
    return not Polygon(cfg.body_outline).within(Polygon(corners).buffer(LARGER, QUAD_SEGS))


def _recorded_loop(area, off, centre: float, cfg: PlannerConfig, space) -> 'Recorded':
    """The recorded loop of area (off: what the loops' centres keep `centre` off, as free has it; space: its
    recorded_space): the outline recorded_loop_margin in, round what's off like a first loop (further where the margin
    is more) and where the body fits across by what it keeps off in the area (RECORDED_KEEP), of each part its outer
    ring; the next loop a lane spacing in from it. A kink of the line turning away from the lawn stays one point moved
    in (RECORDED_MITRE): as an arc of the margin's radius round it the mower would turn on the spot at each of its
    points. Past that GEOS may cut the kink off closer than the margin: the arcs then."""
    m, half = cfg.recorded_loop_margin, 0.5 * cfg.mower_width
    inside = inset(area, m)
    if m:
        kinked = polygonal(area.buffer(-m, join_style=JOIN_MITRE, mitre_limit=RECORDED_MITRE))
        if inside.buffer(1e-4, 2).covers(kinked):
            inside = kinked
    if not off.is_empty:
        inside = polygonal(inside.difference(off.buffer(max(centre, m), QUAD_SEGS)))
    kept = polygonal(area.difference(space).buffer(-RECORDED_SLACK, 4))
    if not kept.is_empty:
        side = max(abs(y) for _, y in cfg.body_outline)
        inside = polygonal(inside.difference(kept.buffer(side + RECORDED_KEEP + RECORDED_SLACK, QUAD_SEGS)))
    found = [ring for p in polygons(inside) if p.area >= half * half for _, ring in rings(Polygon(p.exterior))]
    return Recorded(found, polygonal(inside.buffer(RECORDED_OFF, 4)), space, max(centre, m + cfg.lane_spacing))


def _driven(rings: Sequence[Sequence[Sequence[float]]], bodies: Sequence[Sequence[XY]]):
    """Where a body of each one's corners (bodies, counter-clockwise round its centre, see PlannerConfig.body_outline)
    was while the centre drove along each of the rings the way its points go: along every edge from where it was at the
    edge's start to where it was at its end (_rows), so past a corner and on the outside of a bend the front reaches
    out, and at every point turning on the spot from the heading of the edge before to the next one's (TURN_STEP
    apart)."""
    parts, rows_of = [], {}

    def body(x: float, y: float, h: float, points: Sequence[XY]) -> Polygon:
        c, s = math.cos(h), math.sin(h)
        return Polygon([(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in points])

    for ring, corners in zip(rings, bodies):
        rows = rows_of.get(corners)
        if rows is None:
            rows = rows_of[corners] = _rows(corners)
        heads = []  # every edge's heading and end
        for (ax, ay), (bx, by) in zip(ring, list(ring[1:]) + list(ring[:1])):
            length = math.hypot(bx - ax, by - ay)
            if length > 1e-9:
                heads.append((math.atan2(by - ay, bx - ax), bx, by))
                parts += [body(bx, by, heads[-1][0], [(dx - length * back, dy) for dx, dy, back in row])
                          for row in rows]
        for (h, x, y), (h1, _, _) in zip(heads, heads[1:] + heads[:1]):
            turn = normalize_angle(h1 - h)
            n = int(math.ceil(abs(turn) / TURN_STEP))
            parts += [body(x, y, h + turn * k / n, corners) for k in range(1, n)]
    return union(parts)


def _rows(corners: Sequence[XY]) -> List[List[Tuple[float, float, int]]]:
    """The body of these corners (counter-clockwise) in pieces every line along its heading crosses once (itself where
    it is like that, else cut along the heading where its edges turn from going left to going right or back), each as
    its corners (x, y, 1 on its rear side; the leftmost and rightmost on both sides): driving straight ahead it swept
    the polygon of a piece's corners where it got to, those on the rear side as far back as it drove."""
    ys = [y for _, y in corners]
    goes = [(b > a, b) for a, b in zip(ys, ys[1:] + ys[:1]) if b != a]  # (to the left, to where)
    turns = sorted({y for (left, y), (on, _) in zip(goes, goes[1:] + goes[:1]) if left != on})
    pieces = [list(corners)]
    if len(turns) > 2:
        x0, _, x1, _ = Polygon(corners).bounds
        pieces = [ring for a, b in zip(turns, turns[1:])
                  for _, ring in rings(clip_by_rect(Polygon(corners), x0 - 1.0, a, x1 + 1.0, b))]
    rows = []
    for piece in pieces:
        row: List[Tuple[float, float, int]] = []
        for i, (x, y) in enumerate(piece):
            into, out = y - piece[i - 1][1], piece[(i + 1) % len(piece)][1] - y
            if into == out == 0:  # (in the middle of an edge along it)
                continue
            if into >= 0 and out >= 0 or into <= 0 and out <= 0:
                row.append((x, y, int(into <= 0 and out <= 0)))
            else:
                row += [(x, y, 0), (x, y, 1)] if into > 0 else [(x, y, 1), (x, y, 0)]
        rows.append(row)
    return rows


@dataclasses.dataclass(frozen=True, eq=False)
class Recorded:
    """The recorded loop of a plan (recorded_loop)."""
    rings: List[List[XY]]  # the line it drives, a ring a part of the area (as rings() gives them)
    room: object  # where its centre may go: inside that line, RECORDED_OFF past it
    space: object  # where its body may be (recorded_space)
    first: float  # m from the outline in to the next loop's centre


@dataclasses.dataclass(frozen=True, eq=False)
class Space:
    """A plan's spaces (build_space), the same for the same map and settings."""
    area: object  # the outline
    free: object  # what's mowed: the area but the obstacles (with the body checked the gaps and slots it can't get in)
    robot_space: object  # where the centre drives the loops and lanes
    blade_space: object  # the blade's first loop's level
    drive_space: object  # robot_space, with the body checked body_tolerance further out (turns, drives)
    transit_space: object  # drives, through the drivable areas too
    body_space: object  # where the body may be (None without the body checked)
    slots: List[Polygon]  # the slots by the edge closed (_edge_gaps)
    first: float  # m from the outline in to the blade's first loop
    off: float  # m the centre is further in (blade_offset)
    unmowed: object = None  # the areas not to mow next to it, the blade keeps off them (None: none)
    recorded: Optional[Recorded] = None  # recorded_loop: the recorded loop
    recorded_off: str = ''  # why there's none though recorded_loop is set


def margins_of(boundary, obstacles, drivable, keep_out, reach_over, margins: Sequence[Optional[float]],
               cfg: PlannerConfig) -> List[float]:
    """The margins of the obstacles, then of the areas to keep out of. PlanningError where a point or margin is out
    of range."""
    _check_coordinates('the boundary', boundary)
    for o in obstacles:
        _check_coordinates('an obstacle', o)
    for d in drivable:
        _check_coordinates('a drivable area', d)
    for k in keep_out:
        _check_coordinates('a keep out area', k)
    for r in reach_over:
        _check_coordinates('an area to reach over into', r)
    if len(margins) > len(obstacles) + len(keep_out):
        raise PlanningError(f'{len(margins)} margins for {len(obstacles) + len(keep_out)} obstacles and areas to keep '
                            'out of')
    margin_of = [cfg.obstacle_margin if m is None else m for m in margins]
    margin_of += [cfg.obstacle_margin] * (len(obstacles) + len(keep_out) - len(margin_of))
    for m in margin_of:
        if isinstance(m, bool) or not isinstance(m, (int, float)) or not 0.0 <= m <= MAX_BLADE_OFFSET:
            raise PlanningError(f'a margin must be a number 0..{MAX_BLADE_OFFSET}, got {m!r}')
    return margin_of


def build_space(boundary, obstacles, drivable, keep_out, reach_over, margin_of: List[float], cfg: PlannerConfig,
                edge_slots: bool = True, unmowed=(), bodies=()) -> Tuple[Space, Optional[Footprint]]:
    """The spaces, and the body's Footprint in body_space (None without the body checked). edge_slots: the slots by the
    edge too narrow for the body closed (see _edge_gaps). unmowed (of reach_over): the blade keeps off them (the first
    loop half the mower width off their lines, the Footprint checks it with the body: footprint.Blade), nothing goes
    over them on the way. bodies: see _body_space."""
    half = 0.5 * cfg.mower_width
    # the blade's first loop, the mower's centre is `off` further in (blade_offset, the loops go round with the
    # blade on their outer side)
    first = cfg.blade_loop_offset
    off = abs(cfg.blade_offset)
    # (an outline recorded with a narrower body: that much further in, see _walls)
    wall, walls = _walls(boundary, list(obstacles) + list(keep_out), cfg, bodies)
    if wall > 0:
        first = max(first, cfg.edge_margin + cfg.simplify_tolerance + wall - off)
    centre = first + off

    area = clean_polygon(boundary)
    if area.is_empty:
        raise PlanningError('the boundary needs at least 3 points that enclose an area')
    # with the body checked the loops keep it an obstacle's margin (an area's to keep out of) off its real edge: the
    # obstacle that much bigger where that's further than they keep off the outline
    away_from = [clean_polygon(w) for w in list(obstacles) + list(keep_out)]
    keep = [m + cfg.simplify_tolerance + cfg.wall_half + w for m, w in zip(margin_of, walls)]
    grown = [w.buffer(k - centre, QUAD_SEGS) if cfg.body and k > centre + 1e-9 else w for w, k in zip(away_from, keep)]
    # (and the gaps between two too narrow for the body, see _gaps, or between one and the edge, see _edge_gaps)
    gaps = [g for g in _gaps(away_from, margin_of, [max(k, centre) for k in keep], cfg, walls)
            if g.intersects(area)] if cfg.body else []
    slots = [g for g in _edge_gaps(area, list(drivable) + list(reach_over), away_from, margin_of, cfg, walls, wall)
             if g.intersects(area)] if cfg.body and edge_slots else []
    blocked = union(grown[:len(obstacles)] + gaps + slots)
    free = polygonal(area.difference(blocked)) if not blocked.is_empty else area
    if cfg.simplify_tolerance > 0 and not free.is_empty:
        simple = free.simplify(cfg.simplify_tolerance, preserve_topology=True)
        # simplifying a very jagged outline can make it cross itself, then it stays as it is
        if simple.is_valid:
            free = polygonal(simple)
    if free.is_empty:
        raise PlanningError('the obstacles cover the whole area')
    # with the body checked the loops go round an area to keep out of like round an obstacle (see edges)
    if cfg.body and keep_out:
        free = polygonal(free.difference(union(grown[len(obstacles):])))
    # the blade off the areas not to mow next to this one: the first loop half the mower width off their lines
    unmowed_space = union(clean_polygon(u) for u in unmowed)
    if not unmowed_space.is_empty:
        free = polygonal(free.difference(unmowed_space.buffer(max(0.0, half - first), QUAD_SEGS)))
    # slivers smaller than the blade (where a noisy outline crosses itself) aren't parts of their own
    free = polygonal(union(p for p in polygons(free) if p.area >= half * half))
    if free.is_empty:
        raise PlanningError('the areas to keep out of cover the whole area')
    body_space, footprint = _body_space([boundary] + list(drivable) + list(reach_over),
                                        list(obstacles) + list(keep_out), margin_of, cfg,
                                        Blade.of(unmowed_space, cfg), bodies) if cfg.body else (None, None)
    robot_space = inset(free, centre)
    if robot_space.is_empty:
        raise PlanningError(f'the area is too narrow for a first loop {centre} m inside the outline')
    blade_space = robot_space if off == 0 else inset(free, first)

    # recorded edges are where the centre drove, give or take body_tolerance: with the body checked the centre may go
    # that far past the first loop as well (rounding a corner, a turn, a loop out), the body keeping the margins. The
    # loops and lanes still keep to robot_space
    drive_space = robot_space
    if cfg.body and cfg.body_tolerance > 0:
        drive_space = polygonal(robot_space.buffer(cfg.body_tolerance, QUAD_SEGS))
        if not unmowed_space.is_empty:  # (but no closer to an area not to mow than the first loop)
            drive_space = polygonal(drive_space.difference(unmowed_space.buffer(half, QUAD_SEGS)))

    transit_space = robot_space if cfg.body else drive_space
    extra_space = union(clean_polygon(d) for d in drivable)
    if not extra_space.is_empty:
        # drivable areas are recorded like outlines, the mower's centre may go anywhere in them, but not
        # closer to an obstacle (or an area to keep out of) than in the mowing area
        away = union(grown + [unmowed_space])
        keep_off = away.buffer(max(centre, half), QUAD_SEGS) if not away.is_empty else None
        if keep_off is not None:
            extra_space = extra_space.difference(keep_off)
        transit_space = polygonal(union([transit_space, extra_space]))
    recorded, why = None, cfg.recorded_off
    if cfg.recorded_loop and not why:
        if drawn_ring(bodies, boundary):
            why = 'the area is drawn, not recorded by driving along it'
        elif recorded_larger(boundary, cfg, bodies):
            why = 'the mower is larger than the one the area was recorded with'
        else:
            away = union(grown + gaps + slots + [unmowed_space.buffer(max(0.0, half - first), QUAD_SEGS)])
            recorded = _recorded_loop(area, away, centre, cfg, recorded_space(
                [boundary] + list(drivable) + list(reach_over), list(obstacles) + list(keep_out), margin_of, cfg,
                bodies))
            if not recorded.rings:
                recorded, why = None, 'the area is too small for it'
    return Space(area, free, robot_space, blade_space, drive_space, transit_space, body_space, slots, first,
                 off, None if unmowed_space.is_empty else unmowed_space, recorded, why), footprint
