"""The loops' levels: the area shrunk loop by loop, from where the body fits across (a drawn obstacle, a passage or a
slot by the edge narrower than the body: the loops go round it where the body fits, body_free), on to the middle with
concentric, on into the parts narrower than a U-turn with narrow_parts 'loops'; where the lanes go and what's left for
them, the strips too narrow for lanes."""

import bisect
import dataclasses
import math
from typing import Any, Callable, List, Optional, Tuple

from shapely.geometry import LineString, Point, Polygon
from shapely.prepared import prep

from .config import MAX_ROWS, XY, Lane, PlannerConfig, PlanningError
from .geometry import QUAD_SEGS, inset, polygonal, polygons, rings, subtract, union
from .maneuvers import drive_polyline
from .mathutil import Pose2D
from .trace import _bad, repaired

# m: a loop keeps this much more off where the body would stick out across than the body's half width (the corners of
# its rings rounded, the mower a few cm off its path); what's cut off where it can't get to is opened by CUT_OPEN and
# kept where CUT_MIN m2 at least (along recorded edges the body's space is ragged, not narrower)
BODY_SLACK = 0.015
CUT_OPEN = 0.01
CUT_MIN = 0.002
# m, a part of a level whose ring is shorter than this gets no loop (a loop that small turns round within the 0.6 m of
# a pirouette): what it would mow is the completion stage's
RING_MIN = 2.0
# with the body checked the loops' levels come from where it fits (body_levels): its centre RING_EPS m further in than
# where the body sticks out across, between an island and the edge where it fits (else round both), there within
# RING_NEAR m of the island RING_GAP m further in where the gap has room for that, else as much as it has (the loop
# through the middle; mkay 31.10.); the first level's outline opened by RING_NOISE (a recorded edge's zigzag); further
# in, within RING_ROUND_NEAR m of the islands opened by RING_ROUND (no tip where an island joins the edge's loop or
# another's), closed and opened by RING_SEAM m where that ends (no spike the loop would drive out along and back); a
# part of a level further in smaller than RING_AREA m2 gets no loop (the lanes mow it); an island whose first loop
# would come closer to the edge's than a lane spacing and can't be driven gets none (the edge's goes between them) and
# one loop less, from the next level on its own (its first loop first off it on the lawn's side; where the body can't
# follow the edge's through the gap either, that gap is closed: round both); one that can be driven but the edge's loop
# can't past it (a slot it can't turn into from along the edge, the island's first loop RING_SLOT m along at least
# within RING_SHUT of what's beyond the edge's): the edge's goes round both (the island joined to what's beyond it where
# they're RING_SHUT apart), the island keeps its first loop, through the gap
# rounding a loop's corner may cut it by this share of the mower width at most, 90 degree corners keep the
# full perimeter_corner_radius, sharper ones get less
CORNER_CUT = 0.3
RING_EPS = 0.01
RING_GAP = 0.05
RING_NEAR = 1.0
RING_NOISE = 0.05
RING_ROUND = 0.1
RING_ROUND_NEAR = 0.5
RING_SEAM = 1e-3
RING_CLOSE = 0.3
RING_SHUT = 0.05
RING_SLOT = 1.0
RING_AREA = 1.0
# m2 of the first level outside drive_space it's cut back to that (the centre where the mower may drive)
RING_OUT = 1e-6

RING_QUAD = 16
# with the body checked, a strip too narrow for lanes gets one line down its middle in place of a loop round it where
# mowing along it leaves LINE_LEFT of it at most (mkay 31.10.: once through the middle, no loop); the line from the
# strip's ring LINE_STEP m apart (LINE_POINTS at most), LINE_MIN m long at least
LINE_LEFT = 0.05
LINE_STEP = 0.05
LINE_POINTS = 400
LINE_MIN = 0.5


@dataclasses.dataclass
class Owner:
    mark: XY  # a point inside the island (group) and all its loops
    lag: int  # the level of its first loop
    own: int  # its loops


@dataclasses.dataclass
class Rings:
    levels: list  # level k: free shrunk by the k-th loop's offset (level 0 the blade's)
    drive_levels: list  # what the route drives the loops of (see ring_levels)
    perimeter: int  # of drive_levels the loops along the edges
    middles: list  # ringed: a loop halfway in where a part ends inside level k
    narrow: Any  # strips too narrow for lanes but wider than the loops reach
    lane_region: Any  # where the lane centres lie
    remaining: Any  # free but what the loops along the edges mow
    ringed: bool  # all loops driven part by part (rings.Loops.concentric)
    count: int  # loops along the outline
    islands: int  # loops round the islands
    loop_spacing: float
    owners: Optional[List[Owner]] = None  # with body_levels: the islands' loops
    # (level, ring) of an island's loop on its own where the edge's goes round it (body_levels)
    own_rings: List[Tuple[int, List[XY]]] = dataclasses.field(default_factory=list)

    @property
    def most(self) -> int:
        return max([self.count, self.islands] + [o.lag + o.own for o in self.owners or ()]) if self.owners is not None \
            else max(self.count, self.islands)

    def driven(self, k: int, kind: str, ring: Optional[List[XY]] = None) -> bool:
        """Whether the ring of loop level k is driven: not past its side's number (with owners an island's ring from
        its lag on, as many as its own)."""
        return not left_out(self.count, self.islands, self.owners, self.most, kind, ring, k)

    def loop_rings(self, narrow) -> List[List[XY]]:
        """The rings of the loops driven (narrow: the strips too narrow for lanes that get one)."""
        return [ring for k, geom in enumerate(self.drive_levels) for kind, ring in rings(geom)
                if self.driven(k, kind, ring)] + [ring for _, ring in self.own_rings] + \
            [ring for geom in self.middles + [narrow] for _, ring in rings(geom)]


def left_out(count: int, islands: int, owners: Optional[List[Owner]], most: int, kind: str,
             ring: Optional[List[XY]], k: float) -> bool:
    """Whether the ring ('outline' or 'hole') of loop level k isn't driven: past its side's number, where it only says
    where that side's lanes begin; with owners an island's from its lag on, as many as its own (a ring round islands
    of more than one: while one of them has one)."""
    if k != int(k) or k >= most:
        return False
    if kind != 'hole':
        return k >= count
    if owners is None or ring is None:
        return k >= islands
    poly = Polygon(ring)
    mine = [o for o in owners if poly.contains(Point(o.mark))]
    return not any(o.lag <= k < o.lag + o.own for o in mine) if mine else k >= islands


def body_free(space, cfg: PlannerConfig) -> Tuple[Any, Any, Any]:
    """(free, robot_space, blade_space) of the loops: the space's but where a loop's centre (first in from free's edge)
    wouldn't have the body fit across, the body's half width and BODY_SLACK off the body's space (a drawn obstacle, a
    passage or a slot narrower than the body): the loops go round that. As the space has them without the body."""
    free, first, off = space.free, space.first, space.off
    if space.body_space is None:
        return free, space.robot_space, space.blade_space
    side = max(abs(y) for _, y in cfg.body_outline)
    if side <= first - BODY_SLACK:
        return free, space.robot_space, space.blade_space
    reach = inset(space.body_space, side - first + BODY_SLACK)
    cut = polygonal(free.difference(reach).buffer(-CUT_OPEN, 4).buffer(CUT_OPEN, 4))
    cut = polygonal(union(p for p in polygons(cut) if p.area >= CUT_MIN))
    if cut.is_empty:
        return free, space.robot_space, space.blade_space
    half = 0.5 * cfg.mower_width
    loops = polygonal(union(p for p in polygons(free.difference(cut)) if p.area >= half * half))
    if loops.is_empty:
        return free, space.robot_space, space.blade_space
    robot_space = inset(loops, first + off)
    return loops, robot_space, robot_space if off == 0 else inset(loops, first)


def _hair(geom, r: float = 1e-5):
    """geom closed and opened by a hair: GEOS can leave a crack or a spike without area where pieces are cut and
    joined (a loop would drive out along it and back)."""
    if geom.is_empty:
        return geom
    geom = polygonal(geom.buffer(r, 1, join_style=2).buffer(-r, 1, join_style=2))
    return polygonal(geom.buffer(-r, 1, join_style=2).buffer(r, 1, join_style=2))


def _filled(geom):
    """geom with its holes filled (itself where it has none)."""
    parts = list(polygons(geom))
    if not any(len(p.interiors) for p in parts):
        return geom
    return polygonal(union(Polygon(p.exterior) for p in parts))


def _opened(geom, r: float):
    return geom if r <= 0 or geom.is_empty else polygonal(geom.buffer(-r, RING_QUAD).buffer(r, RING_QUAD))


def _closed(geom, r: float):
    return geom if r <= 0 or geom.is_empty else polygonal(geom.buffer(r, RING_QUAD).buffer(-r, RING_QUAD))


def _drivable(island, footprint, cfg: PlannerConfig) -> bool:
    """Whether the loop round the island (its ring of the first level) can be driven with the body fitting all along,
    either way round, as rings.drive_ring drives it (corners rounded where they fit, the rest repaired, trace.py)."""
    if footprint is None:
        return False
    for poly in polygons(island):
        ring = list(poly.exterior.coords)[:-1]
        if not any(_drives(pts, footprint, cfg) for pts in (ring[::-1], ring)):
            return False
    return True


def _follows(edge, hole, footprint, cfg: PlannerConfig, room=None, kept: Optional[dict] = None,
             around=None) -> bool:
    """Whether the edge's loop (edge's outline ring) past the island in hole can be driven, one way round at least,
    the body fitting within RING_NEAR of it (as _drivable; room: where the centre may go repairing it; kept: each
    ring's poses left, for the next island; around: where the islands looked at are, only places there repaired)."""
    if footprint is None:
        return True
    kept = {} if kept is None else kept
    near = prep(hole.buffer(RING_NEAR, 4))
    for k, poly in enumerate(polygons(edge)):
        if poly.exterior.distance(hole) > RING_NEAR:
            continue
        if k not in kept:
            ring = list(poly.exterior.coords)[:-1]
            kept[k] = [_left(pts, footprint, cfg, room, around) for pts in (ring, ring[::-1])]
        if not any(not any(near.contains(Point(p.x, p.y)) for p in left) for left in kept[k]):
            return False
    return True


def _left(pts: List[XY], footprint, cfg: PlannerConfig, room=None, around=None) -> List[Pose2D]:
    """The poses of the loop round pts, driven as rings.drive_ring drives it, where the body still doesn't fit (around:
    of the places in there only)."""
    poses = drive_polyline(pts + pts[:1], cfg.waypoint_spacing, cfg.perimeter_corner_radius, None, 0,
                           CORNER_CUT * cfg.mower_width, 0, cfg.lane_spacing, cfg.mower_width, footprint.poses_fit)
    only = None if around is None else (lambda p: around.contains(Point(p.x, p.y)))
    poses, _, _ = repaired(poses, footprint, pts, False, cfg.waypoint_spacing, cfg.mower_width, cfg.blade_ahead, room,
                           only)
    return [poses[i] for i in _bad(poses, footprint) if only is None or only(poses[i])]


def _drives(pts: List[XY], footprint, cfg: PlannerConfig) -> bool:
    poses = drive_polyline(pts + pts[:1], cfg.waypoint_spacing, cfg.perimeter_corner_radius, None, 0,
                           CORNER_CUT * cfg.mower_width, 0, cfg.lane_spacing, cfg.mower_width, footprint.poses_fit)
    _, _, left = repaired(poses, footprint, pts, True, cfg.waypoint_spacing, cfg.mower_width, cfg.blade_ahead)
    return left == 0


def _gap_room(body_space, hole, side: float, shut: bool = False):
    """Within RING_NEAR of hole (the hole of where the body fits round an island by the edge) where the centre has less
    than RING_GAP to spare, or as much as the gap between them has (3 mm less than the most it stays a hole with).
    shut: so much that the gap is closed (RING_GAP, more where it has more room)."""
    near = hole.buffer(RING_NEAR, 4)
    local = body_space.intersection(near.buffer(side + RING_GAP + 0.5, 4))
    mark = hole.representative_point()

    def open_at(d: float) -> bool:
        return any(Polygon(r).contains(mark) for p in polygons(inset(local, side + d)) for r in p.interiors)

    lo, hi = RING_EPS, RING_GAP
    while shut and open_at(hi) and hi < RING_NEAR:
        hi *= 2.0
    if not open_at(hi) and not shut:
        for _ in range(5):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if open_at(mid) else (lo, mid)
        hi = max(RING_EPS, lo - 0.003)
    return polygonal(near.difference(inset(local, side + hi)))


def body_levels(space, cfg: PlannerConfig, footprint=None) -> Tuple[list, Any, List[Owner], list]:
    """With the body checked, the loops' levels from where it fits (levels 0 .. most - 1, the lanes' region after them,
    the islands' loops, (level, ring) of an island's first loop on its own where the edge's goes round it, see
    RING_SHUT): the centre RING_EPS further in than where the body sticks out across (fits); the edge's first
    level first in from the edge where that fits (RING_NOISE opened); the islands (a group of them where the body
    doesn't get between) first out from them, as far out as the body needs; level k the edge's first one k lane
    spacings in (its gaps by islands filled), less each island's grown that many lane spacings less its lag (each its
    own number, see RING_AREA for an island by the edge), round near the islands (RING_ROUND)."""
    side = max(abs(y) for _, y in cfg.body_outline)
    fits = inset(space.body_space, side + RING_EPS)
    first, step = space.first, cfg.lane_spacing
    count, islands = cfg.passes, cfg.island_passes
    by_part, owners, own_rings = [], [], []
    for part in polygons(space.free):
        shell = Polygon(part.exterior)
        whole = polygonal(union(Polygon(p.exterior) for p in polygons(fits.intersection(shell.buffer(0.5, QUAD_SEGS)))))
        here = polygonal(fits.intersection(whole))
        # (first in from the edge, nowhere closer: a recorded edge's zigzag only opened)
        # (first in from the edge, RING_EPS closer at most where the closing rounds an inside corner; off an obstacle
        # the loops' centres don't get past by the edge as robot_space keeps off it)
        edge = polygonal(_filled(inset(part, first)).intersection(whole))
        bound = polygonal(_filled(inset(part, first - RING_EPS)).intersection(whole))
        for _ in range(2):
            edge = _opened(polygonal(_closed(edge, RING_CLOSE).intersection(bound)), RING_NOISE)
        if edge.is_empty:
            continue
        rim = edge.boundary
        holes = [Polygon(r) for p in polygons(here) for r in p.interiors]
        mine = [Polygon(r) for r in part.interiors]
        grown, lag = [], []
        for h in holes:
            near = [i for i in mine if i.intersects(h) or h.contains(i)]
            q = polygonal(union([i.buffer(first, QUAD_SEGS) for i in near]))
            q = polygonal(_closed(polygonal(union([h, q])), RING_CLOSE).intersection(whole.buffer(-1e-3, 4)))
            if q.is_empty:
                continue
            grown.append((h, q))
            lag.append(1 if (q.intersects(rim) or q.distance(rim) < step) and not _drivable(q, footprint, cfg) else 0)
        edge, alone = _past_islands(edge, rim, grown, lag, whole, space, side, footprint, cfg)
        own_rings += alone
        own = [max(0, islands - g) for g in lag]
        owners += [Owner(q.representative_point().coords[0], g, n) for (_, q), g, n in zip(grown, lag, own)]
        by_part.append(_part_levels(edge, grown, lag, own, count, step, space))
    most = max((len(levels) for levels in by_part), default=1) - 1
    levels = [polygonal(union(lv[k] for lv in by_part if k < len(lv) - 1)) for k in range(most)]
    return levels, polygonal(union(lv[-1] for lv in by_part)), owners, own_rings


def _past_islands(edge, rim, grown: list, lag: List[int], whole, space, side: float, footprint,
                  cfg: PlannerConfig) -> Tuple[Any, List[Tuple[int, List[XY]]]]:
    """The edge's first level of a part (edge, its ring rim) past the islands by it (grown, lag: as body_levels has
    them; side: the body's half width): through the gaps, round both where the body can't follow it through one, round
    an island with a first loop of its own whose slot it can't be driven past (see RING_SHUT): (that level, the (level,
    ring) of those islands' first loops)."""
    step = cfg.lane_spacing
    own_rings = []
    # the edge's first loop through the gaps by islands: RING_GAP to spare where there's room for it; where the
    # body can't follow it through one either, that gap closed (the loop round both)
    by = [h for (h, _), g in zip(grown, lag) if g]
    # (islands by the edge with a first loop of their own: where the edge's can't be driven past one, the edge's
    # goes round both, the island joined to what's beyond it where they're RING_SHUT apart, and the island keeps
    # its first loop on its own, through the gap: own_rings)
    rimmed = [i for i, ((h, q), g) in enumerate(zip(grown, lag)) if not g and footprint is not None and
              (q.intersects(rim) or q.distance(rim) < step)]
    if by:
        # (there as far out as that leaves, not smoothed: the loop through the middle of a narrow gap)
        near = union(h.buffer(RING_NEAR, 4) for h in by)
        rest = edge.difference(near)

        def through(shut: List[int]):
            tight = [_gap_room(space.body_space, h, side, i in shut) for i, h in enumerate(by)]
            return polygonal(union([rest, polygonal(subtract(whole, tight)).intersection(
                near.intersection(space.drive_space))]))

        edge = through([])
        room = space.drive_space.buffer(1e-3, 4)  # (as rings.drive_ring repairs it)
        kept: dict = {}
        around = prep(near)
        shut = [i for i, h in enumerate(by) if not _follows(edge, h, footprint, cfg, room, kept, around)]
        if shut:
            edge = through(shut)
    if rimmed:
        # (a slot: the island's first loop RING_SLOT m along at least as close to what's beyond, not a pinch the
        # safety net gets the edge's past)
        beyond = polygonal(union(grown[i][0].buffer(RING_NEAR + RING_SHUT, 4) for i in rimmed).difference(edge))
        slots = [i for i in rimmed for kind, ring in rings(grown[i][1])[:1]
                 if LineString(ring + ring[:1]).intersection(beyond.buffer(RING_SHUT, 4)).length >= RING_SLOT]
        room = space.drive_space.buffer(1e-3, 4)
        around = prep(union(grown[i][0].buffer(RING_NEAR, 4) for i in slots)) if slots else None
        kept = {}
        off = [i for i in slots if not _follows(edge, grown[i][0], footprint, cfg, room, kept, around)]
        if off:
            beyond = beyond.intersection(union(grown[i][0].buffer(RING_NEAR + RING_SHUT, 4) for i in off))
            edge = polygonal(edge.difference(_closed(polygonal(union([grown[i][1] for i in off] + [beyond])),
                                                     RING_SHUT)))
            own_rings += [(0, ring[::-1]) for i in off for kind, ring in rings(grown[i][1])[:1]]
    return edge, own_rings


def _part_levels(edge, grown: list, lag: List[int], own: List[int], count: int, step: float, space) -> list:
    """A part's levels from its first one (edge) as body_levels has them, 0 .. most (the last its lanes' region): count
    loops along the edge, own ones round each island (grown) from its lag on, step apart."""
    most = max([count] + [g + n for g, n in zip(lag, own)])
    filled = _filled(edge)  # (from the first level on an island by the edge is cut on its own, see cut)
    levels = []
    for k in range(most + 1):
        outer = edge if k == 0 else inset(filled, min(k, count) * step)
        if outer.is_empty:
            levels += [Polygon()] * (most + 1 - k)  # (the lanes' region empty too)
            break
        cut = [q.buffer(min(k - g, n) * step, RING_QUAD) for (_, q), g, n in zip(grown, lag, own) if k >= g]
        # (an island by the edge before its first loop: the body still keeps out of it)
        cut += [h for (h, _), g in zip(grown, lag) if k < g]
        level = polygonal(outer.difference(union(cut))) if cut else outer
        if k == 0 and level.difference(space.drive_space).area > RING_OUT:
            # (round an obstacle the body gets close to, where it was while it was recorded, or a corner sharper
            # than the edge's loop rounds: the centre still where the mower may drive)
            level = polygonal(level.intersection(space.drive_space))
        if k and cut and k < most:  # (the lanes' region as it is: lanes go into a strip between them)
            # (near the islands round, elsewhere as it is; RING_SEAM where the two meet)
            near = union(cut).buffer(RING_ROUND_NEAR, 4)
            level = _hair(polygonal(union([_opened(level, RING_ROUND).intersection(near), level.difference(near)])),
                          RING_SEAM)
        levels.append(polygonal(union(p for p in polygons(level) if p.exterior.length >= RING_MIN and
                                      (k == 0 or p.area >= RING_AREA))))
    return levels


class _Shrink:
    """The area with its outline some loops in and its islands some loops out (obstacle_passes: the islands, the
    holes of the area, with loops of their own number)."""

    def __init__(self, free, blade_space, first: float, loop_spacing: float, count: int, islands: int) -> None:
        self.free, self.blade_space, self.first, self.spacing = free, blade_space, first, loop_spacing
        self.count, self.islands, self.most = count, islands, max(count, islands)
        self.shell = self.holes_in = None
        if islands != count:
            self.shell = polygonal(union(Polygon(p.exterior) for p in polygons(free)))
            self.holes_in = polygonal(self.shell.difference(free))

    def shrunk(self, out: int, isl: int):
        """The area with its outline out loops in and its islands isl loops out."""
        if out == isl:
            return self.blade_space if out == 0 else inset(self.free, self.first + out * self.spacing)
        outer = inset(self.shell, self.first + out * self.spacing)
        return outer if self.holes_in.is_empty or outer.is_empty else polygonal(outer.difference(
            self.holes_in.buffer(self.first + isl * self.spacing, QUAD_SEGS)))

    def level_at(self, k: int):
        """Level k: past its number a side stays where its lanes would begin (its rings there aren't driven, see
        Rings.driven)."""
        count, islands, most = self.count, self.islands, self.most
        return self.shrunk(min(k, count), min(k, islands)) if k < most else \
            self.shrunk(count + k - most, islands + k - most)


def ring_levels(space, cfg: PlannerConfig, took: Optional[Callable[[str], None]] = None, footprint=None) -> Rings:
    """The loops: level 0 runs perimeter_offset inside the edges (further in where the body wouldn't fit across, see
    body_free), concentric goes on until nothing is left (the last bit, narrower than a lane spacing, gets one more loop
    halfway in), narrow_parts 'loops' on into the parts narrower than a U-turn. Where the lanes go and what's left for
    them. took('perimeter') once the loops' levels are done."""
    first, off = space.first, space.off
    half, spacing = 0.5 * cfg.mower_width, cfg.lane_spacing
    concentric = cfg.fill_pattern == 'concentric'
    count, islands = cfg.passes, cfg.island_passes
    by_body = space.body_space is not None and not concentric and off == 0 and count > 0
    free, robot_space, blade_space = (space.free, space.robot_space, space.blade_space) if by_body else \
        body_free(space, cfg)
    most = max(count, islands)
    # concentric loops meet at the corners: at a right angle the corners of two loops are spacing * sqrt(2)
    # apart, which the blade only covers up to mower_width / sqrt(2)
    loop_spacing = min(spacing, cfg.mower_width / math.sqrt(2.0)) if concentric else spacing
    shrink = _Shrink(free, blade_space, first, loop_spacing, count, islands)
    owners, lanes, own_rings = None, None, []
    if by_body:
        # (where the body gets: the blade's kept off the areas not to mow driving the rings)
        levels, lanes, owners, own_rings = body_levels(space, cfg, footprint and footprint.plain)
        most = len(levels)
        drive_levels = list(levels)
    else:
        levels = []
        while concentric or len(levels) < most:
            level = shrink.level_at(len(levels))
            if level.is_empty:
                break
            levels.append(level)
            if len(levels) > MAX_ROWS:
                raise PlanningError('too many loops: the area is too big for this lane spacing')
        drive_levels = [_worth(g, k == 0) for k, g in enumerate(_island_levels(levels, shrink, robot_space, off,
                                                                               half))]
    perimeter_levels = drive_levels[:most]
    if took is not None:
        took('perimeter')
    rings_of = Rings(levels, drive_levels, len(perimeter_levels), [], Polygon(), Polygon(), Polygon(), concentric,
                     count, islands, loop_spacing, owners, own_rings)
    middles = [_middle(outer, inner, loop_spacing, half) for outer, inner in zip(levels, levels[1:] + [Polygon()])] \
        if concentric else []
    # what's left after the loops (kept for checks and plots, the lanes use the same edge directly)
    remaining = polygonal(subtract(free, [LineString(ring + [ring[0]]).buffer(half, QUAD_SEGS)
                                          for k, level in enumerate(perimeter_levels)
                                          for kind, ring in rings(level) if rings_of.driven(k, kind, ring)] +
                                   [LineString(ring + [ring[0]]).buffer(half, QUAD_SEGS) for _, ring in own_rings]))
    # (the lanes reach lane_overlap_passes loops further out, of the outline's and of the islands')
    lane_out, lane_isl = max(0, count - int(cfg.lane_overlap_passes)), max(0, islands - int(cfg.lane_overlap_passes))
    lane_passes = min(lane_out, lane_isl)
    lane_region = inset(free, first + lane_passes * spacing) if lane_out == lane_isl else \
        shrink.shrunk(lane_out, lane_isl)
    if lanes is not None:
        # (body_levels' level k has the edge's min(k, count) loops and each island's own at once: the side with the
        # most loops reaches lane_overlap_passes out, the other no further than its own)
        lane_passes = max(0, most - int(cfg.lane_overlap_passes))
        lane_region = lanes if lane_passes >= most else levels[lane_passes]
    # where the area is too narrow for lanes but wider than the loops reach (a strip of 0.8 to 1 m with the
    # defaults, or narrower ones where the loops stop earlier), what the loops leave in the middle gets a
    # loop of its own, halfway between the last loop there and the next one in (or where the lanes would be)
    towards = levels[:lane_passes] if lane_out == lane_isl or lanes is not None else \
        [shrink.shrunk(min(k, lane_out), min(k, lane_isl)) for k in range(max(lane_out, lane_isl))]
    narrow = Polygon() if concentric or cfg.skip_lanes else polygonal(union(
        _middle(outer, inner, spacing, half) for outer, inner in zip(towards, towards[1:] + [lane_region])))
    # the loops go on in where the lane region is narrower than a U-turn, the lanes only where it's wider: all the
    # loops are driven part by part like concentric's
    ringed = concentric
    if cfg.narrow_parts == 'loops' and not concentric and not cfg.skip_lanes and cfg.turn_radius > 0 and \
            len(levels) == most and not lane_region.is_empty:
        got = _narrow_loops(levels, shrink, lane_region, lane_passes, cfg, spacing, half, by_body)
        if got is not None:
            middles, lane_region = got
            narrow, ringed = Polygon(), True
    drive_levels += [_worth(g) for g in levels[len(drive_levels):]]
    middles = [_worth(g) for g in middles]
    return dataclasses.replace(rings_of, drive_levels=drive_levels, middles=middles, narrow=narrow,
                               lane_region=lane_region, remaining=remaining, ringed=ringed)


def centre_line(ring: List[XY], reach: float) -> Optional[List[XY]]:
    """The line down the middle of a thin piece (ring: its outline) from one end to the other, where mowing along it
    (reach either side) leaves LINE_LEFT of the piece at most; None where it's no such piece."""
    poly = Polygon(ring)
    if len(ring) < 3 or not poly.is_valid or poly.area <= 0:
        return None
    line = LineString(list(ring) + [ring[0]])
    m = min(LINE_POINTS, max(8, int(line.length / LINE_STEP)))
    m -= m % 2
    xy = [(q.x, q.y) for q in (line.interpolate(line.length * i / m) for i in range(m))]
    h = m // 2
    every = max(1, h // 50)

    def misfit(a: int) -> float:  # (from an end both sides round meet again at the other: a thin piece's)
        return max(math.hypot(xy[(a + i) % m][0] - xy[(a - i) % m][0], xy[(a + i) % m][1] - xy[(a - i) % m][1])
                   for i in range(0, h + 1, every))

    a = min(range(0, m, 2), key=misfit)
    mids = [(0.5 * (xy[(a + i) % m][0] + xy[(a - i) % m][0]), 0.5 * (xy[(a + i) % m][1] + xy[(a - i) % m][1]))
            for i in range(h + 1)]
    middle = LineString(mids)
    if middle.length < LINE_MIN or poly.difference(middle.buffer(reach, 4)).area > LINE_LEFT * poly.area:
        return None
    return list(middle.simplify(0.005).coords)


def _worth(geom, keep: bool = False):
    """geom but its parts whose ring is shorter than RING_MIN (keep: all of it where that leaves nothing)."""
    got = polygonal(union(p for p in polygons(geom) if p.exterior.length >= RING_MIN))
    return geom if keep and got.is_empty else got


def _island_levels(levels: list, shrink: _Shrink, robot_space, off: float, half: float) -> list:
    """What the route drives the loops of: the levels, but with more loops round the islands than along the outline
    those further ones each round its island alone (the island that much bigger). Where an island lies by the edge the
    level's ring runs into the outline's there: the loop is the strip from the loop before to this one then, as far as
    the outline's lanes begin, round that (along this one and back along the one before)."""
    drive_levels = list(levels)
    count, most, first, spacing = shrink.count, shrink.most, shrink.first, shrink.spacing
    if shrink.islands <= count or shrink.holes_in.is_empty:
        return drive_levels
    holes_in = shrink.holes_in
    inside = inset(shrink.shell, first + count * spacing)
    before = holes_in.buffer(first + (count - 1) * spacing, QUAD_SEGS) if count else holes_in
    for k in range(count, min(most, len(levels))):
        blobs = holes_in.buffer(first + k * spacing, QUAD_SEGS)
        strips = [Polygon(strip.exterior) for a in polygons(inside)
                  for strip in polygons(blobs.difference(before).intersection(a)) if strip.area >= half * half]
        if not k or off:
            # (no loop before the first one: by the edge there's none to go back along, the island gets none. With
            # the blade off to the side the centre runs `off` outside a strip's ring: not past the edge either)
            strips = [s for s in strips if robot_space.buffer(1e-6).covers(
                s.buffer(off, QUAD_SEGS).exterior if off else s.exterior)]
        drive_levels[k] = polygonal(inside.buffer(1.0, QUAD_SEGS).difference(union(strips)))
        before = blobs
    return drive_levels


def _middle(outer, inner, step: float, half: float):
    """Where the area ends inside a loop level (outer, loops step apart) and inner, the next level in, doesn't get
    there, what's left is narrower than a step: one more loop halfway in mows it (insets add up: that's the level shrunk
    by half a step, quicker than free shrunk again)."""
    half_in = inset(outer, 0.5 * step)
    if not half_in.is_empty and not inner.is_empty:
        half_in = half_in.difference(inner.buffer(step, QUAD_SEGS))
    return polygonal(union(p for p in polygons(half_in) if p.area >= half * half))


def _narrow_loops(levels: list, shrink: _Shrink, lane_region, lane_passes: int, cfg: PlannerConfig, spacing: float,
                  half: float, body: bool = False):
    """narrow_parts 'loops': the levels on in where the lane region is narrower than a U-turn (levels grows), the lanes
    only where it's wider: (middles, the lane region left), None where there's no such part. body: the levels from the
    body (body_levels), each further one the one before a spacing in."""
    r = cfg.turn_radius
    opened = lane_region.buffer(-r, QUAD_SEGS).buffer(r, QUAD_SEGS)
    # what that takes away: narrow parts, and the tips of corners (smaller than r * r), which keep their lanes
    cut = [p for p in polygons(lane_region.difference(opened)) if p.area >= r * r]
    if not cut:
        return None
    # (opened by a hair: GEOS can leave a spike without area there, the first row of lanes would go by it)
    # (opened by a hair: GEOS can leave a spike without area there, the first row of lanes would go by it)
    wide = polygonal(subtract(lane_region, cut).buffer(-1e-6, 1, join_style=2).buffer(1e-6, 1, join_style=2))
    # bits left between narrow parts too small for more than a few lanes go to the loops as well
    wide = polygonal(union(p for p in polygons(wide) if p.area >= (4.0 * r) ** 2))
    keep_off = None if wide.is_empty else wide.buffer(half, QUAD_SEGS)  # the lanes mow that far round it
    while len(levels) < MAX_ROWS:
        level = _hair(inset(levels[-1], spacing)) if body else shrink.level_at(len(levels))
        if keep_off is not None and not level.is_empty:
            level = polygonal(union(p for p in polygons(level.difference(keep_off)) if p.area >= half * half))
        if level.is_empty:
            break
        levels.append(level)
    middles = [_middle(outer, union([inner, wide]) if k + 1 >= lane_passes else inner, spacing, half)
               for k, (outer, inner) in enumerate(zip(levels, levels[1:] + [Polygon()]))]
    return middles, wide


def narrow_left(narrow, passes, remaining, half: float):
    """Of the strips too narrow for lanes those the lanes leave something of (more than a cm2): a narrow piece needs no
    loop where the lanes already mow what the loops leave around it (they reach on into a strip along them, see
    scan_lanes). The lanes of each set by row, so only those next to a piece are looked at."""
    by_row = [(a, sorted((lane.y, x0, x1) for cell in cells for lane in cell for x0, x1 in lane.pieces()))
              for a, cells in passes]

    def left(piece) -> float:
        strip = piece.buffer(half, QUAD_SEGS)
        near = []
        for a, lanes in by_row:
            c, s = math.cos(a), math.sin(a)
            xs, ys = zip(*[(x * c + y * s, -x * s + y * c) for x, y in strip.exterior.coords])
            first = bisect.bisect_left(lanes, (min(ys) - half,))
            for y, x0, x1 in lanes[first:bisect.bisect_right(lanes, (max(ys) + half, math.inf))]:
                if x0 < max(xs) + half and x1 > min(xs) - half:
                    line = LineString(Lane(0, x0, x1, y).ends(a))
                    if line.distance(strip) < half:
                        near.append(line.buffer(half, QUAD_SEGS))
        return subtract(strip.intersection(remaining), near).area

    return polygonal(union(p for p in polygons(narrow) if left(p) > 1e-4))  # more than a cm2
