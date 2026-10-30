"""mkay's ranking as one table of rates: what a choice may cost in one rank for a gain in another."""

import bisect
import dataclasses
import math
from typing import List, Optional, Sequence

from .report import drive_time

# below this two options leave the same unmowed (m2, the noise of working out areas)
AREA_EPS = 5e-4
# seconds of driving each is worth: a m2 left unmowed (1 cm2 = 6 s), a turn on the spot, a loop (pirouette), a hard
# angle, and what a s of the way to or from the dock and of a drive with the blade on mowing nothing count on top
S_PER_M2 = 60000.0
SPIN_S = 60.0
LOOP_S = 300.0
KINK_S = 10.0
DOCK_EXTRA = 0.1
TRANSIT_EXTRA = 0.05
# m2, a narrower lane spacing (or the slots by the edge closed) has to leave this much less unmowed to be taken
LEFT_GAIN = 0.05
# s a m further from the end point weighs where the path is to end there (the dock): it ends as close to it as it gets
END_S = 1000.0


def allowed_left(spins: int = 0, loops: int = 0) -> float:
    """m2 more a choice may leave unmowed for that many turns on the spot and loops less (EARLIER_LEFT, ROUND_LEFT)."""
    return (SPIN_S * spins + LOOP_S * loops) / S_PER_M2


EARLIER_LEFT = allowed_left(spins=1)
ROUND_LEFT = allowed_left(loops=1)


@dataclasses.dataclass(frozen=True)
class Cost:
    left_m2: float = 0.0  # rank 1
    edge_m2: float = 0.0  # rank 2: of left_m2 what's in the band along the edges
    loops: int = 0  # rank 3
    spins: int = 0  # rank 3
    kinks: int = 0  # rank 5
    time_s: float = 0.0  # rank 7
    dock_s: float = 0.0  # rank 8, of time_s
    transit_s: float = 0.0  # rank 10, of time_s

    def __add__(self, o: 'Cost') -> 'Cost':
        return Cost(*(a + b for a, b in zip(dataclasses.astuple(self), dataclasses.astuple(o))))


def seconds(c: Cost) -> float:
    """The route optimizer's cost."""
    return (S_PER_M2 * c.left_m2 + LOOP_S * c.loops + SPIN_S * c.spins + KINK_S * c.kinks + c.time_s +
            DOCK_EXTRA * c.dock_s + TRANSIT_EXTRA * c.transit_s)


def better(a: Cost, b: Cost) -> Optional[bool]:
    """Whether a beats b in a local choice, None where they're equal. The area first: the one leaving more may still
    win by the turns on the spot and loops it saves (allowed_left), by the rates then, its area counted. Within
    AREA_EPS less along the edges wins, then the rates without the area."""
    more, less = (a, b) if a.left_m2 >= b.left_m2 else (b, a)
    extra = more.left_m2 - less.left_m2
    if extra > AREA_EPS + allowed_left(max(0, less.spins - more.spins), max(0, less.loops - more.loops)):
        return a is less
    if extra > AREA_EPS:
        sa, sb = seconds(a), seconds(b)
    elif abs(a.edge_m2 - b.edge_m2) > 1e-9:
        return a.edge_m2 < b.edge_m2
    else:
        sa, sb = (seconds(dataclasses.replace(c, left_m2=0.0)) for c in (a, b))
    if abs(sa - sb) > 1e-9:
        return sa < sb
    return None if extra <= AREA_EPS else a is less


# a stretch of path about as the ranking counts it (tools/metrics.py): turning on the spot is turning SPIN_MIN at least
# with less than SPIN_R m of driving a rad, a loop a full turn within LOOP_REACH m, a hard angle more than KINK_ANGLE
# within KINK_LEN m of driving (KINK_APART m apart, turning on the spot aside)
SPIN_R, SPIN_MIN = 0.05, math.radians(15.0)
LOOP_REACH = 0.6
KINK_ANGLE, KINK_LEN, KINK_APART = math.radians(45.0), 0.15, 0.5


def counted(poses: Sequence, transit_s: float = 0.0) -> Cost:
    """What driving poses (with .x, .y, .yaw) costs (left_m2 0), transit_s of it on drives mowing nothing."""
    return Cost(loops=_loops(poses), spins=_spins(poses), kinks=_kinks(poses), time_s=drive_time(poses)['s'],
                transit_s=transit_s)


def _steps(poses: Sequence):
    for a, b in zip(poses, poses[1:]):
        yield math.hypot(b.x - a.x, b.y - a.y), math.remainder(b.yaw - a.yaw, 2.0 * math.pi)


def _on_spot(d: float, t: float) -> bool:
    return abs(t) > 1e-9 and (d < 1e-6 or d / abs(t) < SPIN_R)


def _spins(poses: Sequence) -> int:
    n, run = 0, 0.0
    for d, t in _steps(poses):
        if _on_spot(d, t):
            run += abs(t)
        else:
            n, run = n + (run >= SPIN_MIN), 0.0
    return n + (run >= SPIN_MIN)


def _loops(poses: Sequence) -> int:
    turned = [0.0]
    for _, t in _steps(poses):
        turned.append(turned[-1] + t)
    if sum(abs(b - a) for a, b in zip(turned, turned[1:])) < 2.0 * math.pi:
        return 0  # (none turns that far all together)
    n, i = 0, 0
    while i < len(poses):
        j, hit = i, 0
        while j + 1 < len(poses) and math.hypot(poses[j + 1].x - poses[i].x, poses[j + 1].y - poses[i].y) <= LOOP_REACH:
            j += 1
            if abs(turned[j] - turned[i]) >= 2.0 * math.pi:
                hit = j
                break
        n, i = (n + 1, hit + 1) if hit else (n, i + 1)
    return n


def _kinks(poses: Sequence) -> int:
    n, last, run, prev = 0, -math.inf, 0.0, None
    at: List[float] = []  # (m driven to each step of the stretch and its heading, unwrapped)
    heading: List[float] = []
    for (d, t), a, b in zip(_steps(poses), poses, poses[1:]):
        if d < 1e-9 or _on_spot(d, t):
            at, heading, prev = [], [], None
            continue
        h = math.atan2(b.y - a.y, b.x - a.x)
        run += d
        heading.append(h if prev is None else heading[-1] + math.remainder(h - prev, 2.0 * math.pi))
        at.append(run)
        prev = h
        k = bisect.bisect_left(at, run - KINK_LEN)
        if max(heading[k:]) - min(heading[k:]) > KINK_ANGLE and run - last > KINK_APART:
            n, last = n + 1, run
    return n
