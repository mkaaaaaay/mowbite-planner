"""lane_spacing_mode 'auto': what it compares the lane spacings by. The loops and lanes alone (quick, no turns, no
drives), whether what's left is a strip between lanes, the places in spin_margin of a plan."""
import math
from typing import List, Optional, Sequence

from shapely.geometry import LineString

from .config import LEFT_TUFT, PERIMETER, TRACK, XY, CoveragePlan, Lane
from .drive import turning
from .footprint import Footprint
from .geometry import polygons, subtract
from .sweep import swept_area

# lane_spacing_mode 'auto', the places in spin_margin it compares (turn_places): m the body sticks in at least, m
# apart, m a step turning on the spot is at most (as the measuring tools count them, mkay 31.10.)
TURN_OUT, TURN_NEAR, TURN_SPOT = 0.01, 0.5, 0.005


def left_alone(free, loop_rings: List[List[XY]], passes, half: float) -> float:
    """(for lane_spacing_mode 'auto') m2 the loops (their rings) and the lanes alone leave, no turns, no drives, the
    blade TRACK narrower each side: quick to compare."""
    half -= TRACK
    bands = [LineString(ring + [ring[0]]).buffer(half, 4) for ring in loop_rings]
    bands += [LineString(Lane(0, x0, x1, lane.y).ends(a)).buffer(half, 4) for a, cells in passes for cell in cells
              for lane in cell for x0, x1 in lane.pieces()]
    return sum(p.area for p in polygons(subtract(free, bands)) if p.area >= LEFT_TUFT)


def left_whole(plan: CoveragePlan) -> float:
    """m2 the plan leaves with the blade as wide as it is (pieces of LEFT_TUFT at least, the way simpler by 1 cm)."""
    return sum(p.area for p in polygons(plan.free_space.difference(swept_area(plan, plan.config.mower_width, 0.01)))
               if p.area >= LEFT_TUFT)


def strip_between_lanes(piece, angles: Sequence[float]) -> bool:
    """Whether a piece left unmowed is a strip along the lanes (of one of the angles): three times as long as wide."""
    for a in angles:
        c, s = math.cos(a), math.sin(a)
        pts = list(piece.exterior.coords)
        along = [x * c + y * s for x, y in pts]
        across = [-x * s + y * c for x, y in pts]
        if max(along) - min(along) > 3.0 * (max(across) - min(across)):
            return True
    return False


def turn_places(plan: CoveragePlan, fp: Optional[Footprint]) -> int:
    """(for lane_spacing_mode 'auto') the places, TURN_NEAR apart, where the body sticks more than TURN_OUT into
    spin_margin where the path turns (drive.turning) or turns on the spot: on the loops only turning on the spot (mkay
    31.10.), none on the outermost one driven as recorded and the safety net's ways along it."""
    n, P = len(plan.poses), plan.poses
    if fp is None or not n:
        return 0
    loops = {name for name, o in (plan.origins or {}).items()
             if o.get('kind') == 'outline' and o.get('parent') == 'level-0'}
    outer, ringish = [False] * n, [False] * n
    for seg in plan.segments:
        if seg.kind == PERIMETER and seg.source in loops:
            outer[seg.start:seg.end + 1] = [True] * (seg.end + 1 - seg.start)
        if (seg.kind == PERIMETER or seg.maneuver == 'ring') and seg.maneuver != 'mop_up':
            ringish[seg.start:seg.end + 1] = [True] * (seg.end + 1 - seg.start)
    for seg in plan.segments:
        if seg.maneuver == 'body' and outer[seg.start] and outer[seg.end]:
            outer[seg.start:seg.end + 1] = [True] * (seg.end + 1 - seg.start)

    def spot(i: int) -> bool:
        return any(0 <= j < n and math.hypot(P[j].x - P[i].x, P[j].y - P[i].y) < TURN_SPOT and
                   abs(math.remainder(P[j].yaw - P[i].yaw, 2.0 * math.pi)) > 1e-6 for j in (i - 1, i + 1))

    found = []
    for a, b in zip(plan.section_starts, list(plan.section_starts[1:]) + [n]):
        part = P[a:b]
        turn = turning(part).tolist()
        bad = [i for i in fp.bad_poses(part, turn) if not outer[a + i] and (not ringish[a + i] or spot(a + i))]
        for d, xy in fp.out_by([(part[i - 1] if i else None, part[i]) for i in bad],
                               on_turn=[turn[i] for i in bad]) if bad else ():
            if xy is not None and d > TURN_OUT:
                found.append((d, xy))
    places: List[XY] = []
    for _, (x, y) in sorted(found, key=lambda f: -f[0]):
        if all(math.hypot(x - px, y - py) > TURN_NEAR for px, py in places):
            places.append((x, y))
    return len(places)
