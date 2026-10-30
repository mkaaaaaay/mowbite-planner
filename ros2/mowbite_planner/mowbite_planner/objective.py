"""mkay's ranking as one table of rates: what a choice may cost in one rank for a gain in another."""

import dataclasses
from typing import Optional

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
