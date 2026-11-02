"""How far a planning got (plan_coverage's progress), for whoever waits for it: planner.plan's progress topic on
OpenMower, the bridge on MowBite OS. Only to show: done and of never go back, so one bar does."""

from typing import Callable, Optional

# progress(stage, done, of): stage one of STAGES, done of of steps
Report = Callable[[str, int, int], None]
# angles: begun (the lanes' angle first), spacing: a lane spacing planned through (lane_spacing_mode auto), route: the
# path put together, mop_up: what it leaves mowed where it's worth it, done
STAGES = ('angles', 'spacing', 'route', 'net', 'mop_up', 'done')


class Progress:
    """report(stage, done, of) as a planning goes on: done never going back, of the same all along."""

    def __init__(self, report: Optional[Report], of: int) -> None:
        self.report, self.of, self.done = report, max(1, int(of)), 0

    def at(self, stage: str, done: Optional[int] = None) -> None:
        """At stage with done steps (without: one more than before), at most of."""
        self.done = min(self.of, max(self.done, self.done + 1 if done is None else int(done)))
        if self.report is not None:
            self.report(stage, self.done, self.of)
