"""Turns between neighbouring lanes, and driving along polylines with rounded corners.

A wheel-driven mower that spins 180 degrees on the spot tears up the lawn with its wheels. The turns here
keep moving while turning, on arcs of a minimum radius:

- U-turn: quarter arc, straight, quarter arc. Only possible when the lanes are at least two radii apart,
  which with a 0.19 m lane spacing means a radius of under 0.1 m.
- Bulb (omega) turn: a short swerve away from the next lane, a loop of more than half a circle, a swerve
  back. Forwards only, but needs about radius * (1 + 2 sin(theta)) of headland.
- K-turn: quarter arc forwards, a short straight backwards, quarter arc forwards. Needs only one radius
  of headland and keeps the radius, at the cost of a stop.
- Detour: quarter arc out of the lane, the shortest way through the free space along the edge, quarter
  arc into the next lane. For lanes far apart (lane_order skip) where the straight middle of a U-turn
  would leave the free space because the edge isn't square to the lanes.
- Pivot: turn on the spot by 90 degrees, cross over, turn on the spot by 90 degrees again. The last
  resort, still half the twist of a 180 degree spin.

Poses face the way the mower drives: on straight bits each one points at the next (OpenMower's planner
does the same), on arcs along the arc. No two poses share a position: OpenMower's FTC controller skips
such a pose, Nav2's regulated pure pursuit takes it for a cusp. A turn on the spot is a heading jump from
one pose to the next. On the backwards leg of a K-turn the poses face the way the mower faces, against the
way it drives (Nav2's convention, its controller finds the cusp from that).

Every turn is first built in a local frame: the mower leaves its lane at the origin heading +x, the next
lane lies to the left (+y) at distance d and is driven back towards -x, starting at (dx, d). A turn to
the right is the mirror image. The caller checks each candidate against the free space and takes the
first one that fits.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from .mathutil import Pose2D, normalize_angle

XY = Tuple[float, float]
Fits = Callable[[Sequence[XY]], bool]
Route = Callable[[XY, XY], Optional[List[XY]]]
# whether the mower's body stays where it may be at these poses (footprint.py), None: not checked
Body = Optional[Callable[[Sequence['Pose2D']], bool]]

# arcs get a pose at least every 10 degrees, whatever the waypoint spacing
MAX_ARC_STEP = math.radians(10.0)
# a bend sharper than this counts as a corner (for rounding the perimeter loops)
CORNER_ANGLE = math.radians(15.0)
# a detour that would still turn by more than this from one pose to the next isn't taken
DETOUR_MAX_JUMP = math.radians(45.0)
# poses closer than this are one position
SAME_POSITION = 1e-6
# m, the radii a corner gets where the body doesn't fit round it (more would leave a lot of the corner unmowed)
BODY_RADII = (0.25, 0.5, 1.0)
_EPS = 1e-9


@dataclass
class Piece:
    """Part of a maneuver driven in one direction."""

    poses: List[Pose2D]
    reverse: bool = False


@dataclass
class Maneuver:
    name: str  # 'u_turn', 'bulb', 'k_turn', 'detour' or 'pivot'
    radius: float
    pieces: List[Piece] = field(default_factory=list)

    def points(self) -> List[XY]:
        return [(p.x, p.y) for piece in self.pieces for p in piece.poses]

    def length(self) -> float:
        return sum(path_length(piece.poses) for piece in self.pieces)


def path_length(poses: Sequence[Pose2D]) -> float:
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:]))


def sample_straight(x0: float, y0: float, x1: float, y1: float, yaw: float, step: float) -> List[Pose2D]:
    """Poses from (x0, y0) to (x1, y1), at most `step` apart (step <= 0: just both ends), all facing yaw."""
    length = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(math.ceil(length / step - 1e-9))) if step > 0 else 1
    yaw = normalize_angle(yaw)
    return [Pose2D(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, yaw) for i in range(n + 1)]


def sample_arc(x: float, y: float, heading: float, radius: float, turn: float, step: float) -> List[Pose2D]:
    """Driving forwards on a circle from (x, y) facing `heading`, turning by `turn` (rad, > 0 left)."""
    side = 1.0 if turn >= 0.0 else -1.0
    # the centre lies `radius` to the left (or right) of the heading
    cx = x - side * radius * math.sin(heading)
    cy = y + side * radius * math.cos(heading)
    n = abs(turn) / MAX_ARC_STEP
    if step > 0:
        n = max(n, abs(turn) * radius / step)
    n = max(1, int(math.ceil(n - 1e-9)))
    out = []
    for i in range(n + 1):
        h = heading + turn * i / n
        out.append(Pose2D(cx + side * radius * math.sin(h), cy - side * radius * math.cos(h), normalize_angle(h)))
    return out


def same_position(a, b) -> bool:
    return abs(a[0] - b[0]) < SAME_POSITION and abs(a[1] - b[1]) < SAME_POSITION


def extend(out: List[Pose2D], poses: Sequence[Pose2D]) -> List[Pose2D]:
    """Appends poses to out (in place). Of two poses on the same position only the later one stays: it has
    the heading the mower leaves with."""
    for p in poses:
        if out and same_position(out[-1], p):
            out[-1] = p
        else:
            out.append(p)
    return out


def join(*parts: Sequence[Pose2D]) -> List[Pose2D]:
    """Concatenates pose lists like extend()."""
    out: List[Pose2D] = []
    for part in parts:
        extend(out, part)
    return out


# turns in the local frame --------------------------------------------------------------------------------


def _extensions(dx: float) -> Tuple[float, float]:
    # lanes end at different places when the edge runs at a slant. The turn happens level with the
    # longer of the two, the shorter one is carried on straight to get there (a) or after it (b)
    return max(dx, 0.0), max(-dx, 0.0)


def _lead_in(a: float, step: float) -> List[Pose2D]:
    return sample_straight(0.0, 0.0, a, 0.0, 0.0, step) if a > _EPS else []


def _lead_out(a: float, b: float, d: float, step: float) -> List[Pose2D]:
    return sample_straight(a, d, a - b, d, math.pi, step) if b > _EPS else []


def u_turn(d: float, dx: float, r: float, step: float) -> Optional[Maneuver]:
    if d < 2.0 * r - 1e-9:
        return None
    a, b = _extensions(dx)
    poses = join(
        _lead_in(a, step),
        sample_arc(a, 0.0, 0.0, r, math.pi / 2, step),  # ends at (a + r, r) facing +y
        sample_straight(a + r, r, a + r, d - r, math.pi / 2, step) if d - 2.0 * r > _EPS else [],
        sample_arc(a + r, d - r, math.pi / 2, r, math.pi / 2, step),  # ends at (a, d) facing -x
        _lead_out(a, b, d, step),
    )
    return Maneuver('u_turn', r, [Piece(poses)])


def bulb_turn(d: float, dx: float, r: float, step: float) -> Optional[Maneuver]:
    if d >= 2.0 * r or r <= _EPS:
        return None
    # swerve right by theta, loop left by pi + 2 theta, swerve right by theta. The loop's centre has to
    # lie halfway between the lanes: -r + 2 r cos(theta) = d / 2
    theta = math.acos((d + 2.0 * r) / (4.0 * r))
    a, b = _extensions(dx)
    first = sample_arc(a, 0.0, 0.0, r, -theta, step)
    loop = sample_arc(first[-1].x, first[-1].y, first[-1].yaw, r, math.pi + 2.0 * theta, step)
    last = sample_arc(loop[-1].x, loop[-1].y, loop[-1].yaw, r, -theta, step)
    # snap away the rounding of three chained arcs, it has to meet the lane exactly
    last[-1] = Pose2D(a, d, math.pi)
    poses = join(_lead_in(a, step), first, loop, last, _lead_out(a, b, d, step))
    return Maneuver('bulb', r, [Piece(poses)])


def k_turn(d: float, dx: float, r: float, step: float) -> Optional[Maneuver]:
    if d >= 2.0 * r or r <= _EPS:
        return None
    a, b = _extensions(dx)
    forward = join(_lead_in(a, step), sample_arc(a, 0.0, 0.0, r, math.pi / 2, step))
    # still facing +y, but backing up from y = r to y = d - r
    backward = sample_straight(a + r, r, a + r, d - r, math.pi / 2, step)
    forward2 = join(sample_arc(a + r, d - r, math.pi / 2, r, math.pi / 2, step), _lead_out(a, b, d, step))
    return Maneuver('k_turn', r, [Piece(forward), Piece(backward, reverse=True), Piece(forward2)])


def pivot_turn(d: float, dx: float, step: float) -> Maneuver:
    a, b = _extensions(dx)
    # facing the next lane from the corner on, facing back along it from the far corner on
    poses = join(
        _lead_in(a, step),
        sample_straight(a, 0.0, a, d, math.pi / 2, step),
        [Pose2D(a, d, math.pi)],
        _lead_out(a, b, d, step),
    )
    return Maneuver('pivot', 0.0, [Piece(poses)])


def _to_map(m: Maneuver, origin: Pose2D, side: float) -> Maneuver:
    c, s = math.cos(origin.yaw), math.sin(origin.yaw)
    pieces = []
    for piece in m.pieces:
        poses = []
        for p in piece.poses:
            ly = side * p.y
            poses.append(Pose2D(origin.x + p.x * c - ly * s, origin.y + p.x * s + ly * c,
                                normalize_angle(origin.yaw + side * p.yaw)))
        pieces.append(Piece(poses, piece.reverse))
    return Maneuver(m.name, m.radius, pieces)


def turn_radii(radius: float, d: float) -> List[float]:
    """The radii to try, largest first: the wanted one, smaller ones, and the half circle d / 2."""
    out: List[float] = []
    for r in (radius, 0.75 * radius, 0.5 * radius, 0.5 * d):
        if 1e-3 < r <= radius + 1e-9 and all(abs(r - o) > 1e-3 for o in out):
            out.append(r)
    return sorted(out, reverse=True)


def detour_turn(exit_pose: Pose2D, entry: XY, radius: float, step: float, fits: Fits,
                route: Route, body: Body = None) -> Optional[Maneuver]:
    """Quarter arc out of the lane towards the next one, route() between the arcs, quarter arc into the next
    lane. Only for lanes at least two radii apart, closer ones would need the middle to go backwards."""
    h = exit_pose.yaw
    vx, vy = entry[0] - exit_pose.x, entry[1] - exit_pose.y
    lateral = -vx * math.sin(h) + vy * math.cos(h)
    side = 1.0 if lateral >= 0.0 else -1.0
    for r in (radius, 0.75 * radius, 0.5 * radius):
        if r <= 1e-3 or abs(lateral) < 2.0 * r:
            continue
        out_arc = sample_arc(exit_pose.x, exit_pose.y, h, r, side * math.pi / 2, step)
        # the arc into the next lane, worked out backwards from its start: facing the other way and turning
        # the other way traces the same circle
        back = sample_arc(entry[0], entry[1], h, r, -side * math.pi / 2, step)
        in_arc = [Pose2D(p.x, p.y, normalize_angle(p.yaw + math.pi)) for p in reversed(back)]
        if not fits([(p.x, p.y) for p in out_arc]) or not fits([(p.x, p.y) for p in in_arc]):
            continue
        a, b = out_arc[-1], in_arc[0]
        middle = connect((a.x, a.y), a.yaw, (b.x, b.y), b.yaw, r, step, fits, route, body)
        if middle is None:
            continue
        poses = join(out_arc, middle, in_arc)
        # where the edge leaves no room for arcs it would still turn on the spot, a pivot does that shorter
        if max_heading_jump(poses) > DETOUR_MAX_JUMP or body is not None and not body([exit_pose] + poses):
            continue
        return Maneuver('detour', r, [Piece(poses)])
    return None


def arc_towards(a: XY, heading: float, target: XY, radius: float, step: float) -> Optional[List[Pose2D]]:
    """Forwards on a circle of `radius` from a (facing heading) until the mower faces target, turning towards
    it. None if target lies inside that circle."""
    s = 1.0 if math.cos(heading) * (target[1] - a[1]) - math.sin(heading) * (target[0] - a[0]) >= 0 else -1.0
    cx, cy = a[0] - s * radius * math.sin(heading), a[1] + s * radius * math.cos(heading)
    d = math.hypot(target[0] - cx, target[1] - cy)
    if d <= radius + 1e-6:
        return None
    # the tangent from target touches the circle acos(r / d) off the line from the centre to target
    tangent = math.atan2(target[1] - cy, target[0] - cx) - s * math.acos(radius / d)
    sweep = math.fmod(s * (tangent - math.atan2(a[1] - cy, a[0] - cx)), 2.0 * math.pi)
    sweep = sweep + 2.0 * math.pi if sweep < 0.0 else sweep
    return sample_arc(a[0], a[1], heading, radius, s * sweep, step)


def connect(a: XY, heading_a: Optional[float], b: XY, heading_b: Optional[float], radius: float, step: float,
            fits: Fits, route: Route, body: Body = None) -> Optional[List[Pose2D]]:
    """From a (facing heading_a) to b (arriving facing heading_b) the way route() finds. Where it fits it
    leaves on an arc that turns it towards the first bend and arrives on one from the last bend (a Dubins
    like start and end), the bends between are rounded: no turning on the spot at a and b. None if route()
    finds no way."""
    pts = route(a, b)
    if pts is None:
        return None
    pts = [tuple(p) for p in pts]

    def arc(start: XY, heading: float, toward: XY, back: bool = False) -> Optional[List[Pose2D]]:
        # the widest arc that fits, from where it ends the way to `toward` has to be free too. back: an arc
        # worked out backwards from the end, the mower faces the other way on it
        for r in (radius, 0.75 * radius, 0.5 * radius):
            poses = arc_towards(start, heading, toward, r, step) if r > 1e-3 else None
            if poses is not None and len(poses) > 2 and fits([(p.x, p.y) for p in poses]) and \
                    fits([(poses[-1].x, poses[-1].y), toward]) and \
                    (body is None or body([Pose2D(p.x, p.y, p.yaw + math.pi) for p in poses] if back else poses)):
                return poses
        return None

    leave: List[Pose2D] = []
    arrive: List[Pose2D] = []
    for _ in range(2):  # the two ends depend on each other when the way is one straight line
        if heading_a is not None:
            out = arc(a, heading_a, pts[1])
            if out is not None:
                leave, pts[0] = out, (out[-1].x, out[-1].y)
        if heading_b is not None:
            back = arc(b, heading_b + math.pi, pts[-2], True)
            if back is not None:
                arrive = [Pose2D(p.x, p.y, normalize_angle(p.yaw + math.pi)) for p in reversed(back)]
                pts[-1] = (back[-1].x, back[-1].y)
        if len(pts) > 2:
            break
    return join(leave, drive_polyline(pts, step, radius, fits, body=body), arrive)


def max_heading_jump(poses: Sequence[Pose2D]) -> float:
    return max((abs(normalize_angle(q.yaw - p.yaw)) for p, q in zip(poses, poses[1:])), default=0.0)


def plan_turn(exit_pose: Pose2D, entry: XY, radius: float, allow_reverse: bool, step: float,
              fits: Fits, route: Optional[Route] = None,
              types: Sequence[str] = ('u_turn', 'bulb', 'k_turn', 'detour', 'pivot'),
              body: Body = None) -> Optional[Maneuver]:
    """The gentlest turn from the end of a lane (exit_pose) into the next lane, which starts at `entry`
    and is driven the opposite way. route (shortest way between two points in the free space) allows
    detours. types: the kinds allowed, preferred first; larger radii always come before smaller ones,
    detours and pivots after all radii. body: the turn's poses have to pass it too. None if nothing allowed
    fits."""

    def ok(m: Maneuver) -> bool:
        return fits(m.points()) and (body is None or body([exit_pose] + [p for pc in m.pieces for p in pc.poses]))

    h = exit_pose.yaw
    vx, vy = entry[0] - exit_pose.x, entry[1] - exit_pose.y
    dx = vx * math.cos(h) + vy * math.sin(h)
    lateral = -vx * math.sin(h) + vy * math.cos(h)
    side = 1.0 if lateral >= 0.0 else -1.0
    d = abs(lateral)
    if d < 1e-6:
        return None
    types = [t for t in types if t != 'k_turn' or allow_reverse]
    shapes = {'u_turn': u_turn, 'bulb': bulb_turn, 'k_turn': k_turn}
    for r in turn_radii(radius, d):
        # a U-turn where the lanes are two radii apart, a bulb or a K-turn where they're closer
        for name in types:
            if name not in shapes or (name == 'u_turn') != (d >= 2.0 * r - 1e-9):
                continue
            local = shapes[name](d, dx, r, step)
            if local is None:
                continue
            m = _to_map(local, exit_pose, side)
            if ok(m):
                return m
    for name in types:
        if name == 'detour' and route is not None:
            m = detour_turn(exit_pose, entry, radius, step, fits, route, body)
            if m is not None:
                return m
        elif name == 'pivot':
            m = _to_map(pivot_turn(d, dx, step), exit_pose, side)
            if ok(m):
                return m
    return None


# polylines -------------------------------------------------------------------------------------------------


def _dedupe_xy(points: Sequence[XY]) -> List[XY]:
    out: List[XY] = []
    for p in points:
        if not out or not same_position(out[-1], p):
            out.append((float(p[0]), float(p[1])))
    return out


def inner_radius(rho: float, turn: float, spacing: float, width: float) -> float:
    """The largest corner radius for a loop `spacing` inside one whose corner (turning by `turn`) is
    rounded with rho, so that the strips the two mow (`width` wide) leave nothing between them in the corner.

    In the corner the strips are furthest apart on its bisector, where distances along the loops' straight
    parts grow by k = 1 / cos(turn / 2). The outer strip's inner edge rounds with rho - width / 2 around the
    outer arc's centre (or comes to a point there when rho is smaller than that), the inner strip's outer
    edge with the inner radius + width / 2 around the inner arc's centre.

    That holds where the outer loop's corner got rho. Where it is spread over a few short edges (a chamfer,
    a corner recorded point by point) the outer loop's arcs get less (an arc takes at most half of each edge
    next to it), while the inner loop, where those edges are gone, starts from rho: a tuft of a few cm2 can
    stay there."""
    k = 1.0 / max(1e-9, math.cos(0.5 * abs(turn)))
    half = 0.5 * width
    if k * spacing <= width + 1e-12:
        return rho  # strips close enough to stay together with the same radius
    if rho > half:
        return max(0.0, rho - (k * spacing - width) / (k - 1.0))
    return max(0.0, min(rho, (half * (1.0 + k) - k * spacing) / (k - 1.0)))


def drive_polyline(points: Sequence[XY], step: float, radius: float = 0.0, fits: Optional[Fits] = None,
                   turn_sign: int = 0, max_cut: Optional[float] = None, level: int = 0, spacing: float = 0.0,
                   width: float = 0.0, body: Body = None) -> List[Pose2D]:
    """Poses along a polyline, facing the way it goes.

    Corners are rounded with arcs of up to `radius` where `fits` accepts the arc (smaller ones are tried
    when it doesn't, then the corner stays sharp: its pose already faces the next segment). turn_sign 1
    only rounds left turns, -1 only right turns, 0 both. max_cut: how far an arc may pass inside the
    corner at most (a sharp corner rounded with the full radius would leave a lot of it unmowed). level: the
    polyline is the level-th of loops `spacing` apart, mowing `width` wide, rounded like that from the
    outermost one in: its corners get less radius (inner_radius), so no tuft stays between them (but
    for a few cm2 where the outer loop's corner is spread over short edges, see there). body: where the body
    doesn't fit round a corner (or turning on the spot at it), the BODY_RADII are tried as far as the segments
    allow, max_cut, level and turn_sign don't count then.
    """
    pts = _dedupe_xy(points)
    if len(pts) < 2:
        return [Pose2D(pts[0][0], pts[0][1], 0.0)] if pts else []
    n = len(pts)
    lengths = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(n - 1)]
    yaws = [math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0]) for i in range(n - 1)]

    # corner i joins segment i - 1 and segment i: (trim on both, arc poses)
    rounded = {}
    if radius > 0.0 and fits is not None:
        for i in range(1, n - 1):
            turn = normalize_angle(yaws[i] - yaws[i - 1])
            if abs(turn) < 1e-6 or abs(turn) > math.pi - 1e-3:
                continue
            if turn_sign and (turn > 0) != (turn_sign > 0):
                continue
            tan_half = math.tan(abs(turn) / 2.0)
            rho = radius
            if max_cut is not None:
                # the arc's middle is rho * (1 / cos(turn / 2) - 1) inside the corner
                rho = min(rho, max_cut / (1.0 / math.cos(abs(turn) / 2.0) - 1.0))
            for _ in range(level):
                rho = inner_radius(rho, turn, spacing, width)
            # at most half of each neighbouring segment, so neighbouring arcs never overlap
            rho = min(rho, 0.5 * min(lengths[i - 1], lengths[i]) / tan_half)
            for _ in range(3):
                if rho < 1e-3:
                    break
                trim = rho * tan_half
                ux, uy = math.cos(yaws[i - 1]), math.sin(yaws[i - 1])
                arc = sample_arc(pts[i][0] - trim * ux, pts[i][1] - trim * uy, yaws[i - 1], rho, turn, step)
                if fits([(p.x, p.y) for p in arc]):
                    rounded[i] = (trim, arc)
                    break
                rho *= 0.5
        if body is not None:
            for i in range(1, n - 1):
                turn = normalize_angle(yaws[i] - yaws[i - 1])
                # (a slight bend, of a recorded edge, needs no more room than driving straight on)
                if abs(turn) < CORNER_ANGLE or abs(turn) > math.pi - 1e-3:
                    continue
                corner = rounded[i][1] if i in rounded else [Pose2D(pts[i][0], pts[i][1], yaws[i - 1]),
                                                             Pose2D(pts[i][0], pts[i][1], yaws[i])]
                if body(corner):
                    continue
                tan_half = math.tan(abs(turn) / 2.0)
                most = 0.5 * min(lengths[i - 1], lengths[i]) / tan_half
                least = rounded[i][0] / tan_half if i in rounded else 0.0
                for rho in BODY_RADII:
                    if rho <= least + 1e-3 or rho > most:
                        continue
                    trim = rho * tan_half
                    ux, uy = math.cos(yaws[i - 1]), math.sin(yaws[i - 1])
                    arc = sample_arc(pts[i][0] - trim * ux, pts[i][1] - trim * uy, yaws[i - 1], rho, turn, step)
                    if (fits is None or fits([(p.x, p.y) for p in arc])) and body(arc):
                        rounded[i] = (trim, arc)
                        break

    out: List[Pose2D] = []
    for i in range(n - 1):
        ux, uy = math.cos(yaws[i]), math.sin(yaws[i])
        t0 = rounded[i][0] if i in rounded else 0.0
        t1 = rounded[i + 1][0] if i + 1 in rounded else 0.0
        a = (pts[i][0] + t0 * ux, pts[i][1] + t0 * uy)
        b = (pts[i + 1][0] - t1 * ux, pts[i + 1][1] - t1 * uy)
        extend(out, sample_straight(a[0], a[1], b[0], b[1], yaws[i], step))
        if i + 1 in rounded:
            extend(out, rounded[i + 1][1])
    return out


def closest_on_polyline(points: Sequence[XY], q: XY) -> Tuple[int, XY, float]:
    """(index of the segment, the closest point on it, its distance) of q to a polyline."""
    best = (0, (points[0][0], points[0][1]), math.inf)
    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        dx, dy = bx - ax, by - ay
        len2 = dx * dx + dy * dy
        t = 0.0 if len2 <= 0.0 else max(0.0, min(1.0, ((q[0] - ax) * dx + (q[1] - ay) * dy) / len2))
        px, py = ax + t * dx, ay + t * dy
        dist = math.hypot(q[0] - px, q[1] - py)
        if dist < best[2]:
            best = (i, (px, py), dist)
    return best
