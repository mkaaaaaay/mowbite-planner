"""A drive of its own from one pose to another through the drivable areas (the node's ~/plan_transit), for a robot that
drives the plans itself: from where it is to a plan's first pose, between its sections and areas, to the dock.

The way goes like a coverage plan's drives between its lanes and areas (mowbite_planner.drive's Drive: the shortest way
through the free space, leaving and arriving on arcs, the bends rounded, turning on the spot only where no arc fits or
with turn_on_spot), the body checked all along as on a plan (mowbite_planner.space's body space and Footprint). A start
or goal by the edge (the dock) may lie a little outside: the stretch from there straight to where the body fits isn't
checked, there's no turning on the spot on it.
"""

import dataclasses
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import Point
from shapely.ops import nearest_points

from mowbite_planner.config import XY, PlannerConfig, PlanningError
from mowbite_planner.drive import Drive
from mowbite_planner.footprint import SPIN, Footprint
from mowbite_planner.geometry import QUAD_SEGS, clean_polygon, inset, polygonal, union
from mowbite_planner.maneuvers import dubins, join, path_length, same_position, sample_arc, sample_straight
from mowbite_planner.mathutil import Pose2D, angle_diff, normalize_angle
from mowbite_planner.planner import _turning
from mowbite_planner.space import _body_space, _check_coordinates
from mowbite_planner.transit import TransitPlanner

# m, how far the start or the goal may lie outside of where the body fits by default
MAX_OUTSIDE = 0.5
# (m, rad): onto the line through the goal from that far beside it, or off the start's that far, on an S of two arcs
# turning by that much each way (as wide as that takes, none tighter than turn_radius)
MERGES = ((0.3, math.radians(25.0)), (0.3, math.radians(15.0)), (0.5, math.radians(25.0)))
# where no arc gets to the goal or away from the start: arriving or leaving facing this many other ways at most,
# turning on the spot there
SPIN_TRIES = 4


# (m straight on, ((radius, turn), ...) arcs): a way onto the goal or off the start, see plan_transit's drive
Bend = Tuple[float, Tuple[Tuple[float, float], ...]]


@dataclasses.dataclass
class Transit:
    poses: List[Pose2D]  # base_link, facing the next pose (the last one the way it arrives), forwards only
    length: float  # m
    spins: List[int]  # the poses where it turns on the spot (0: at the start, from the start's heading)
    start_outside: float  # m from the start on where the body isn't checked (the start lay outside)
    goal_outside: float  # m before the goal where it isn't (the goal lies outside)
    warnings: List[str]


def centre_space(drivable: Sequence[Sequence[Sequence[float]]], obstacles: Sequence[Sequence[Sequence[float]]],
                 cfg: PlannerConfig) -> Tuple[Any, Optional[Footprint]]:
    """Where the mower's centre may go on a transit (cfg as plan_transit takes it): with the body set where it fits
    sideways driving straight (and its Footprint there, see plan_transit), without it half the mower_width off the
    obstacles (and None). Raises PlanningError where that's nowhere."""
    areas = union(clean_polygon(d) for d in drivable)
    if areas.is_empty:
        raise PlanningError('no drivable areas')
    footprint: Optional[Footprint] = None
    if cfg.body:
        body_space, footprint = _body_space(list(drivable), list(obstacles), [cfg.obstacle_margin] * len(obstacles),
                                            cfg)
        space = polygonal(inset(body_space, footprint.half).intersection(areas))
    else:
        blocked = union(clean_polygon(o) for o in obstacles)
        keep = max(cfg.centre_offset, 0.5 * cfg.mower_width)
        space = polygonal(areas.difference(blocked.buffer(keep, QUAD_SEGS))) if not blocked.is_empty else areas
    if space.is_empty:
        raise PlanningError('the body fits nowhere in the drivable areas')
    return space, footprint


def plan_transit(start: Sequence[float], goal: Sequence[float], drivable: Sequence[Sequence[Sequence[float]]],
                 obstacles: Sequence[Sequence[Sequence[float]]] = (), config: Optional[PlannerConfig] = None,
                 goal_straight: float = 0.0, max_outside: float = MAX_OUTSIDE) -> Transit:
    """The drive from start (x, y, yaw) to goal ((x, y) or (x, y, yaw): arriving facing yaw) inside the drivable
    areas (mowing and navigation areas, [(x, y), ...] each) but the obstacles, all in one flat frame (map), the mower's
    centre (base_link). goal_straight: m straight on before the goal, facing its yaw (the dock). With the body set
    (robot_width, robot_front and robot_rear, or robot_outline) it keeps edge_margin off the areas' real edges,
    obstacle_margin off the obstacles and spin_margin more turning on the spot (see space._body_space), without it the
    centre keeps half the mower_width off the obstacles. Where the start or the goal lies outside (the centre, or the
    body sticking out), it drives straight from there to where it fits, max_outside m at most: straight ahead from the
    start (where that doesn't get there and the body fits turning at the start, towards the closest point of the
    areas), on the goal's heading to the goal (without one, from the closest point). The same request gives the same
    poses. An empty way where it's at the goal already (closer than waypoint_spacing, facing its way; with turn_on_spot
    whatever way it faces, turning there is up to whoever drives it). Raises PlanningError where there's no way (a
    goal by the edge no arc of turn_radius gets to facing its heading, where the body sticks out turning into it there
    too, says so), or the input is out of range."""
    cfg = config or PlannerConfig()
    cfg.validate()
    cfg = _turning(cfg)  # (as in plan_coverage)
    if len(start) < 3 or len(goal) < 2:
        raise PlanningError('the start needs x, y and yaw, the goal x and y')
    _check_coordinates('the start or goal', [start, goal])
    heading = goal[2] if len(goal) >= 3 and goal[2] is not None else None
    for what, v in (('the start yaw', start[2]), ('the goal yaw', heading), ('goal_straight', goal_straight),
                    ('max_outside', max_outside)):
        if v is not None and not math.isfinite(v):
            raise PlanningError(f'{what} must be a number, got {v!r}')
    if goal_straight < 0 or max_outside < 0:
        raise PlanningError('goal_straight and max_outside must be >= 0')
    if goal_straight > 0 and heading is None:
        raise PlanningError('goal_straight needs a goal yaw')
    for d in drivable:
        _check_coordinates('a drivable area', d)
    for o in obstacles:
        _check_coordinates('an obstacle', o)
    step = cfg.waypoint_spacing
    if not step > 0:
        raise PlanningError('a transit needs waypoint_spacing > 0')
    space, footprint = centre_space(drivable, obstacles, cfg)
    warnings: List[str] = [] if footprint is not None else [cfg.no_body or '']
    transit = TransitPlanner(space, cfg.transit_clearance, edge_distance=cfg.transit_edge_distance,
                             edge_cost=cfg.transit_edge_cost)

    def clear(p: XY, yaw: float) -> bool:
        return transit.is_free([p]) and (footprint is None or footprint.fits_pose(p[0], p[1], yaw))

    def first_clear(p: XY, yaw: float, sign: float) -> Optional[float]:
        # m from p (sign 1: ahead, -1: back) to the first place, waypoint_spacing apart, where the centre is in and the
        # body fits facing yaw, None if there's none within max_outside
        ux, uy = sign * math.cos(yaw), sign * math.sin(yaw)
        for k in range(int(max_outside / step + 1e-9) + 1):
            if clear((p[0] + k * step * ux, p[1] + k * step * uy), yaw):
                return k * step
        return None

    def closest(p: XY) -> Tuple[float, float]:
        q = nearest_points(space, Point(p))[0]
        return math.hypot(q.x - p[0], q.y - p[1]), math.atan2(q.y - p[1], q.x - p[0])

    # the start: where the body fits, straight ahead or towards the closest point (turning there, the body checked)
    s, s_yaw = (float(start[0]), float(start[1])), normalize_angle(float(start[2]))
    way_in, in_yaw = first_clear(s, s_yaw, 1.0), s_yaw
    if way_in is None:
        far, yaw = closest(s)
        if far > 0 and (footprint is None or footprint.fits_pose(s[0], s[1], s_yaw) and footprint.fits_turn(
                s[0], s[1], s_yaw, yaw)):
            way_in, in_yaw = first_clear(s, yaw, 1.0), yaw
        if way_in is None:
            where = f'{far:.2f} m outside the drivable areas' if far > 0 else \
                'where the body sticks out of the drivable areas, straight ahead too'
            raise PlanningError(f'the start ({s[0]:.2f}, {s[1]:.2f}) lies {where} (max_outside {max_outside} m)')
    entry = (s[0] + way_in * math.cos(in_yaw), s[1] + way_in * math.sin(in_yaw))

    # the goal: arriving straight on its heading (from the closest point without one), goal_straight at least
    g = (float(goal[0]), float(goal[1]))
    g_yaw = None if heading is None else normalize_angle(float(heading))
    way_out, out_yaw = 0.0, g_yaw
    if g_yaw is not None:
        way_out = first_clear(g, g_yaw, -1.0)
    elif not transit.is_free([g]):
        far, yaw = closest(g)
        out_yaw = normalize_angle(yaw + math.pi)
        way_out = first_clear(g, out_yaw, -1.0)
    if way_out is None:
        far, _ = closest(g)
        where = f'{far:.2f} m outside the drivable areas' if far > 0 else \
            'where the body sticks out of the drivable areas, on its heading before it too'
        raise PlanningError(f'the goal ({g[0]:.2f}, {g[1]:.2f}) lies {where} (max_outside {max_outside} m)')
    exit_ = (g[0] - way_out * math.cos(out_yaw), g[1] - way_out * math.sin(out_yaw)) if way_out > 0 else g
    straight = max(goal_straight, way_out)
    before = (g[0] - straight * math.cos(out_yaw), g[1] - straight * math.sin(out_yaw)) if straight > 0 else g

    if math.hypot(g[0] - s[0], g[1] - s[1]) < step:
        off = 0.0 if g_yaw is None else angle_diff(g_yaw, s_yaw)
        if off <= SPIN:
            return Transit([], 0.0, [], 0.0, 0.0, warnings + ['at the goal already'])
        if cfg.turn_on_spot:
            return Transit([], 0.0, [], 0.0, 0.0, warnings + [
                f'at the goal already, {math.degrees(off):.0f} deg off its heading: turning on the spot there is up to '
                'whoever drives it'])

    head = sample_straight(s[0], s[1], entry[0], entry[1], in_yaw, step)[:-1] if way_in > 0 else []
    tail = sample_straight(exit_[0], exit_[1], g[0], g[1], out_yaw, step)[1:] if way_out > 0 else []
    approach = sample_straight(before[0], before[1], exit_[0], exit_[1], out_yaw, step) \
        if straight > way_out else []
    if approach and not (transit.is_free([before, exit_]) and (footprint is None or footprint.all_fit(approach))):
        raise PlanningError(f'no room for {straight:.2f} m straight before the goal ({g[0]:.2f}, {g[1]:.2f})')
    before_first = head[-1] if head else Pose2D(s[0], s[1], s_yaw)

    def turns_in(poses: List[Pose2D], first: float, arrive: Optional[float]) -> int:  # (into arrive at the end too)
        yaws = [first] + [p.yaw for p in poses] + ([arrive] if arrive is not None else [])
        return sum(angle_diff(a, b) > SPIN for a, b in zip(yaws, yaws[1:]))

    def drive(tp: TransitPlanner, c: PlannerConfig, arrive: Optional[float], lead: Optional[Bend] = None,
              leave: Optional[Bend] = None, into: Optional[float] = None,
              away: Optional[float] = None) -> Optional[Tuple[List[Pose2D], list]]:
        # the way from entry to before through tp's space, arriving facing arrive, and the poses of it where the body
        # sticks out (from the pose before: turning on the spot at the first, from the start's heading or the way
        # in's). lead (m, arcs): arriving that much further back, straight on from there, before that on the arcs
        # ((radius, turn), ... in the order it drives them); leave (m, arcs): leaving straight ahead that far, then on
        # the arcs; into: arriving facing that way, turning on the spot there into arrive; away: turning on the spot at
        # entry first, leaving facing that way. None where there's no way
        to, to_yaw, on = before, arrive if into is None else into, approach
        if lead is not None:
            back = straight + lead[0]
            to = (g[0] - back * math.cos(arrive), g[1] - back * math.sin(arrive))
            on = sample_straight(to[0], to[1], exit_[0], exit_[1], arrive, step)
            for radius, turn in reversed(lead[1]):
                # (each arc driven backwards from where the next one starts, facing the other way on it)
                arc = sample_arc(on[0].x, on[0].y, on[0].yaw + math.pi, radius, -turn, step)
                on = join([Pose2D(p.x, p.y, normalize_angle(p.yaw + math.pi)) for p in reversed(arc)], on)
            to, to_yaw = (on[0].x, on[0].y), on[0].yaw
            if not tp.is_free([(p.x, p.y) for p in on] if lead[1] else [to, exit_]) or \
                    footprint is not None and not footprint.all_fit(on):
                return None
        frm, frm_yaw, out = entry, in_yaw if away is None else away, []
        if leave is not None:
            ahead = (entry[0] + leave[0] * math.cos(in_yaw), entry[1] + leave[0] * math.sin(in_yaw))
            out = sample_straight(entry[0], entry[1], ahead[0], ahead[1], in_yaw, step)
            for radius, turn in leave[1]:
                out = join(out, sample_arc(out[-1].x, out[-1].y, out[-1].yaw, radius, turn, step))
            if not tp.is_free([(p.x, p.y) for p in out]) or footprint is not None and not footprint.all_fit(out):
                return None
            frm, frm_yaw = (out[-1].x, out[-1].y), out[-1].yaw
        far = math.hypot(to[0] - frm[0], to[1] - frm[1]) > 1e-6
        way = Drive(tp, footprint, c).way(frm, frm_yaw, to, to_yaw) if far else []
        if not c.turn_on_spot and to_yaw is not None and c.turn_radius > 1e-3 and \
                (way is None or turns_in(way, frm_yaw, to_yaw)):
            # turning on the spot where Dubins' way straight there (the shortest first) fits: that one (a short way,
            # round once where it ends up next to where it started)
            for m in dubins(Pose2D(frm[0], frm[1], frm_yaw), Pose2D(to[0], to[1], to_yaw), c.turn_radius, step):
                dub = join(*[piece.poses for piece in m.pieces])
                if tp.is_free([(p.x, p.y) for p in dub]) and (footprint is None or footprint.poses_fit(dub)):
                    way = dub
                    break
        if way is None:
            return None
        way = join(out, way, on)
        if not way:  # (from where the body fits straight on to the goal)
            way = [Pose2D(entry[0], entry[1], in_yaw)]
        last = way[-1]
        if arrive is not None and angle_diff(last.yaw, arrive) > 1e-9:
            # (arriving another way, on the spot: turning there into the goal's heading)
            way[-1] = Pose2D(last.x, last.y, arrive)
        bad = [] if footprint is None else [way[i - 1] for i in footprint.bad_poses([before_first] + way) if i > 0]
        return way, bad

    # where the body sticks out on the way: keeping off the edges where it can (room to turn the nose round on the spot
    # there, see TransitPlanner.careful_path), with turn_on_spot on arcs; then arriving straight on, 2 turn_radius
    # (alongside the edge where the goal lies by one), to a goal without a heading facing a way the body fits there
    # (the closest to the way it arrived first); then arriving and leaving on arcs (see drive)
    tries: List[Tuple[TransitPlanner, PlannerConfig, Optional[float], Dict[str, Any]]] = [(transit, cfg, out_yaw, {})]
    r = cfg.turn_radius
    lead: Bend = (2.0 * r, ())
    if footprint is not None:
        wide = TransitPlanner(space, cfg.transit_clearance, edge_distance=max(cfg.transit_edge_distance,
                                                                              footprint.reach - footprint.half),
                              edge_cost=max(cfg.transit_edge_cost, 2.0))
        tries.append((wide, cfg, out_yaw, {}))
        if cfg.turn_on_spot:
            arcs = dataclasses.replace(cfg, turn_on_spot=False)
            tries += [(transit, arcs, out_yaw, {}), (wide, arcs, out_yaw, {})]

    def turned(p: XY, yaw: float, into: bool) -> List[float]:
        # headings 22.5 deg apart the body fits turning on the spot at p from into yaw (into) or from yaw into,
        # SPIN_TRIES of them at most: those with the most room straight before p (into) or after it first (up to 2
        # turn_radius, the centre in, the body fitting), then the closest to yaw
        def room(h: float) -> float:
            sign = -1.0 if into else 1.0
            for k in range(1, int(2.0 * r / step + 1e-9) + 1):
                q = (p[0] + sign * k * step * math.cos(h), p[1] + sign * k * step * math.sin(h))
                if not (transit.is_free([p, q]) and footprint.fits_pose(q[0], q[1], h)):
                    return (k - 1) * step
            return 2.0 * r

        yaws = [normalize_angle(yaw + i * math.pi / 8) for i in range(1, 16)]
        yaws = [h for h in yaws if footprint.fits_turn(p[0], p[1], *((h, yaw) if into else (yaw, h)))]
        return sorted(yaws, key=lambda h: (-room(h), angle_diff(h, yaw)))[:SPIN_TRIES]

    first_bad = None
    for k, (tp, c, arrive, more) in enumerate(tries):
        got = drive(tp, c, arrive, **more)
        if got is None:
            if k == 0:
                raise PlanningError(f'no way from ({s[0]:.2f}, {s[1]:.2f}) to ({g[0]:.2f}, {g[1]:.2f}) through the '
                                    'drivable areas')
            continue
        checked, bad = got
        if not bad:
            break
        if first_bad is None:
            first_bad = bad
            if out_yaw is not None:
                yaws = [out_yaw]
            else:
                came = checked[-1].yaw
                yaws = sorted((normalize_angle(came + i * math.pi / 8) for i in range(16)),
                              key=lambda h: angle_diff(h, came))
                yaws = [h for h in yaws if footprint.fits_pose(g[0], g[1], h)][:3]
            tries += [(t, cfg, h, {'lead': m}) for h in yaws for m in ((lead,) if out_yaw is not None else (None, lead))
                      for t in (transit, wide)]
            if r > 1e-3:
                # then arriving on an arc of turn_radius, straight on before the goal for a shorter way or none, and
                # leaving straight on or on such an arc; then onto the line through the goal (off it) from beside it
                # (alongside an edge, a gentle S, where an arc of turn_radius would swing the nose out over it); then
                # arriving another way and turning on the spot at the goal into its heading, turning on the spot at
                # the start and leaving another way (where that fits there, see turned): as a plan arrives at and
                # leaves its poses
                bends = [(m, ((r, side * a),)) for a in (0.25 * math.pi, 0.5 * math.pi) for side in (1.0, -1.0)
                         for m in (0.0, r)]
                merges = [(0.0, ((radius, side * a), (radius, -side * a))) for off, a in MERGES
                          for radius in (0.5 * off / (1.0 - math.cos(a)),) if radius >= r for side in (1.0, -1.0)]
                tries += [(t, cfg, h, {'lead': m}) for h in yaws for m in [(r, ()), (0.5 * r, ())] + bends
                          for t in (transit, wide)]
                tries += [(t, cfg, out_yaw, {'leave': m}) for m in [(r, ()), (2.0 * r, ())] + bends
                          for t in (transit, wide)]
                tries += [(t, cfg, h, {'lead': m}) for h in yaws for m in merges for t in (transit, wide)]
                tries += [(t, cfg, out_yaw, {'leave': m}) for m in merges for t in (transit, wide)]
                if out_yaw is not None:
                    tries += [(t, cfg, out_yaw, {'into': h}) for h in turned(before, out_yaw, True)
                              for t in (transit, wide)]
                tries += [(t, cfg, out_yaw, {'away': h}) for h in turned(entry, in_yaw, False) for t in (transit, wide)]
    else:
        if all(same_position((p.x, p.y), exit_) for p in first_bad):
            raise PlanningError(f'no way arrives at the goal ({g[0]:.2f}, {g[1]:.2f})' +
                                (' facing its heading' if heading is not None else '') +
                                ' with the body in: no arc of turn_radius gets there, turning on the spot there '
                                'sticks out')
        if all(same_position((p.x, p.y), entry) for p in first_bad):
            raise PlanningError(f'no way leaves the start ({s[0]:.2f}, {s[1]:.2f}) with the body in: no arc of '
                                'turn_radius gets away, turning on the spot there sticks out')
        where = ', '.join(f'({p.x:.2f}, {p.y:.2f})' for p in first_bad[:3])
        raise PlanningError(f'the body sticks out on the way at {len(first_bad)} poses: {where}')
    poses = join(head, checked, tail)
    if len(poses) < 2:  # (there, facing another way, with no room to go round)
        raise PlanningError(f'at the goal already, {math.degrees(angle_diff(out_yaw, s_yaw)):.0f} deg off its heading, '
                            'no arc of turn_radius turns it round there')
    if len(poses) < 3:  # (at least 3, as a plan's)
        a, b = poses[0], poses[-1]
        poses = [a, Pose2D(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), a.yaw), b]
    spins = [i for i in range(1, len(poses)) if angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN]
    if angle_diff(poses[0].yaw, s_yaw) > SPIN:
        spins.insert(0, 0)
    if spins and not cfg.turn_on_spot:
        warnings.append(f'turns on the spot at {len(spins)} places, no arc of turn_radius {cfg.turn_radius} m fits '
                        'there')
    start_out = math.hypot(entry[0] - s[0], entry[1] - s[1])
    goal_out = math.hypot(g[0] - exit_[0], g[1] - exit_[1])
    for what, out in (('start', start_out), ('goal', goal_out)):
        if out > 0:
            warnings.append(f'the {what} lies outside of where the body fits, {out:.2f} m straight to it unchecked')
    return Transit(poses, path_length(poses), spins, start_out, goal_out, warnings)
