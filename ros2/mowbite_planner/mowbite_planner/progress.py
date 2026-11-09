"""How far a planning got (plan_coverage's progress), for whoever waits for it: planner.plan's progress topic on
OpenMower, the bridge on MowBite OS. Only to show: done and of never go back, so one bar does. Where it's stopped from
outside (stoppable): a planning of this thread ends with Stopped at its next step (check)."""

import contextlib
import threading
from typing import Callable, Iterator, Optional

# progress(stage, done, of): stage one of STAGES, done of of steps
Report = Callable[[str, int, int], None]
# angles: begun (the lanes' angle first), spacing: a lane spacing planned through (lane_spacing_mode auto), route: the
# path put together, mop_up: what it leaves mowed where it's worth it, done
STAGES = ('angles', 'spacing', 'route', 'net', 'mop_up', 'done')
# the stop of the planning a thread runs (stoppable)
_STOP = threading.local()


class Stopped(BaseException):
    """A planning stopped from outside (stoppable): no plan. Not an Exception: nothing on the way takes it for a
    planning's failure."""


@contextlib.contextmanager
def stoppable(stop: Callable[[], bool]) -> Iterator[None]:
    """The plannings of this thread end with Stopped at their next step (check) once stop() is true."""
    before = getattr(_STOP, 'stop', None)
    _STOP.stop = stop
    try:
        yield
    finally:
        _STOP.stop = before


def stopping() -> bool:
    """Whether the planning of this thread is to stop (stoppable)."""
    stop = getattr(_STOP, 'stop', None)
    return stop is not None and stop()


def check() -> None:
    """Stopped where the planning of this thread is to stop (stoppable)."""
    if stopping():
        raise Stopped('stopped')


class Progress:
    """report(stage, done, of) as a planning goes on: done never going back, of the same all along."""

    def __init__(self, report: Optional[Report], of: int) -> None:
        self.report, self.of, self.done = report, max(1, int(of)), 0

    def at(self, stage: str, done: Optional[int] = None) -> None:
        """At stage with done steps (without: one more than before), at most of. Stopped where it's to stop (but done:
        the plan's there)."""
        if stage != 'done':
            check()
        self.done = min(self.of, max(self.done, self.done + 1 if done is None else int(done)))
        if self.report is not None:
            self.report(stage, self.done, self.of)
