"""The way onto the plan's first loop (rings.drive_ring): OpenMower's FTC takes a path on up to its max_goal_angle_error
off its first pose's heading, along the outermost loop driven as recorded the body turned that much sticks out; the
path starts before it with a way from where the body turns whatever way it faces, easing onto the loop."""

import math
from typing import List, Optional, Tuple

from shapely.geometry import Point

from .footprint import Footprint, swung
from .maneuvers import join, sample_straight
from .mathutil import Pose2D, angle_diff

# rad, m: OpenMower's FTC takes the path on up to its max_goal_angle_error (20 deg) off the first pose's heading: where
# the body turned START_SWING either way doesn't fit START_SWUNG on from the plan's first pose, the path starts before it
# (LEAD_IN)
START_SWING = math.radians(20.0)
START_SWUNG = 0.5
# m: where the body turned START_SWING doesn't fit there (the outermost loop along the recorded edge), the path starts
# with a way onto the loop: along the loop's last LEAD_LEN m, LEAD_IN off it on the mowable side there, getting onto it
# smoothly (the offset (3 - 2 u) u^2 of u from 1 to 0), turning LEAD_TURN at most a step; the first from where the body
# turns whatever way it faces, fitting turned START_SWING its first START_SWUNG and with the loop's check all along and
# onto the loop, else none
LEAD_IN = (0.3, 0.45, 0.6, 0.8)
LEAD_LEN = (2.0, 3.0, 4.0)
LEAD_TURN = math.radians(10.0)
# where none fits onto the loop where it starts, the loop starts at one of its poses LEAD_SPREAD m apart along it
# instead, the closest to where it would first, LEAD_TRIES at most
LEAD_SPREAD = 0.25
LEAD_TRIES = 40


def way_onto(route, poses: List[Pose2D], side: float, body: Footprint) -> Optional[List[Pose2D]]:
    """The way onto the plan's first loop (poses, the mowable side on the left with side 1, else on the right) from
    where the body turns whatever way it faces (see LEAD_IN), None where it fits turned START_SWING at its start
    or no such way fits (body: the loop's). [] where it fits turned START_SWING at its start."""
    fp, step = route.footprint, route.step
    swing = Footprint(fp.space, swung(fp.corners, START_SWING))
    at, k = 0.0, 1
    while k < len(poses) and at <= START_SWUNG:
        at += math.hypot(poses[k].x - poses[k - 1].x, poses[k].y - poses[k - 1].y)
        k += 1
    if all(swing.fit(poses[:k])):
        return []
    room = fp.turning.anywhere
    # (the loop backwards from its start: m back, the point, the normal to the mowable side)
    back, d = [], 0.0
    for j in range(len(poses) - 1, 0, -1):
        a, b = poses[j], poses[j - 1]
        back.append((d, a.x, a.y, -side * math.sin(a.yaw), side * math.cos(a.yaw)))
        d += math.hypot(a.x - b.x, a.y - b.y)
        if d > LEAD_LEN[-1] + step:
            break
    for length in LEAD_LEN:
        for off in LEAD_IN:
            pts = []
            for t, x, y, nx, ny in back:
                if t > length + 1e-9:
                    break
                u = t / length
                o = off * u * u * (3.0 - 2.0 * u)
                pts.append((x + o * nx, y + o * ny))
            pts = pts[::-1]
            if len(pts) < 3 or not room.contains(Point(pts[0])) or not route.area.is_free(pts):
                continue
            way = join(*(sample_straight(a[0], a[1], b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0]), step)
                         for a, b in zip(pts, pts[1:])))
            way[-1] = poses[0]  # (ending on the loop's first pose)
            if any(angle_diff(a.yaw, b.yaw) > LEAD_TURN for a, b in zip(way, way[1:])):
                continue
            j, at = 1, 0.0
            while j < len(way) and at <= START_SWUNG:
                at += math.hypot(way[j].x - way[j - 1].x, way[j].y - way[j - 1].y)
                j += 1
            if all(swing.fit(way[:j])) and body.poses_fit(way[:-1] + poses[:k]):
                return way
    return None


def lead_in(route, poses: List[Pose2D], side: float, body: Footprint) -> Tuple[List[Pose2D], Optional[List[Pose2D]]]:
    """(the first loop's poses, the way onto it or None): where none fits onto the loop where it starts (way_onto), the
    loop starting at one of its poses LEAD_SPREAD apart instead, the closest to where it would first (LEAD_TRIES). Only
    for the plan's first loop (part of it: checked like it, in the recorded space where it's driven so), closed (the way
    runs along its end)."""
    if route.started or route.segments or route.footprint is None or len(poses) <= 3 or \
            math.hypot(poses[0].x - poses[-1].x, poses[0].y - poses[-1].y) >= 1e-6:
        return poses, None
    lead = way_onto(route, poses, side, body)
    if lead is None:
        picks, at, last = [], 0.0, -math.inf
        for j in range(1, len(poses) - 1):
            at += math.hypot(poses[j].x - poses[j - 1].x, poses[j].y - poses[j - 1].y)
            if at - last >= LEAD_SPREAD:
                picks.append(j)
                last = at
        picks.sort(key=lambda j: (math.hypot(poses[j].x - poses[0].x, poses[j].y - poses[0].y), j))
        for j in picks[:LEAD_TRIES]:
            turned = poses[j:-1] + poses[:j] + [poses[j]]
            lead = way_onto(route, turned, side, body)
            if lead is not None:
                return turned, lead
    return poses, lead or None
