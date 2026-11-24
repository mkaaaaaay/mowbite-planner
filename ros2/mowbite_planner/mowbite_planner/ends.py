"""Where mower_logic gets from a section's end to the next one's start itself (no arrives, see starts.py): where it'd
turn there with no room for the body (sections close together, a turn _fit_body refused: Review 19), the end and the
start left out as far as it takes for that way to fit (sweep.fits_over) (mkay 24.11.: no turn where the body has no room, unmowed rather
than a contact; with back_out a section ends backing up first, starts.backed_out)."""

import dataclasses
import heapq
import math
from typing import Iterator, List, Optional, Tuple

from .bodyfit import unchecked_said
from .config import CoveragePlan
from .mathutil import Pose2D
from .starts import BACK_OUT_MAX, _at, _bounds, _rebuilt
from .sweep import SECTION_NEAR, fits_over, plan_footprint, swept_area

# poses tried OVER_STEP m of path apart for where a section ends and where the next one starts, those with BACK_OUT_MAX
# m of either left out at most first, then any further on OVER_FAR apart; OVER_TRIES ways of each a place at most
OVER_STEP = 1.0
OVER_FAR = 1.0
OVER_TRIES = 20000


def left_ends(plan: CoveragePlan) -> CoveragePlan:
    """Without arrives (mower_logic gets from where a section ends to where the next one starts itself, sweep.fits_over):
    where a section starts close to where the one before ends (sweep.SECTION_NEAR) and the way between doesn't fit,
    the end of the one before and the start of the next left out as far as it takes for the way between them to fit:
    of the poses OVER_STEP m apart the pair leaving the least out, BACK_OUT_MAX m of either at most first, else any
    further on (OVER_FAR apart: the rest of the section, the next ones' too); a section ending backing up (backed_out)
    ends there. Where none fits (OVER_TRIES each), as it was: the report finds it. stats['left_ends']: {'places': [[x, y, m left
    out]], 'stuck': [[x, y]], 'unmowed_m2'}, _fit_body's unchecked places it settles taken off its warning."""
    fp = plan_footprint(plan, blade=False)
    bounds = _bounds(plan) if fp is not None and plan.poses else []
    if len(bounds) < 2:
        return plan
    poses, n = plan.poses, len(plan.poses)
    back = [False] * n  # (got to backing up)
    for seg in plan.segments:
        if seg.reverse:
            back[seg.start + 1:seg.end + 1] = [True] * (seg.end - seg.start)
    starts = {a for a, _ in bounds}

    def hop(i: int) -> float:  # m from the pose before within its section
        return 0.0 if i in starts else math.hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y)

    cut: List[Tuple[int, int]] = []
    places, stuck, settled = [], [], []  # type: Tuple[List[list], List[list], List[list]]
    low = 0  # (the first pose a section may still end at: none before a cut)
    for (first, e), (s, _) in zip(bounds, bounds[1:]):
        if s < low or e < low:
            continue
        a, b = poses[e], poses[s]
        if math.hypot(b.x - a.x, b.y - a.y) > SECTION_NEAR or fits_over(fp, a, b, back[e]):
            continue
        # ((m left out, pose) where it may end, where the next may start: two poses of a section kept at least)
        ends, d = [(0.0, e)], 0.0
        if not back[e]:
            for i in range(e - 1, max(first, low), -1):
                d += hop(i + 1)
                if d >= ends[-1][0] + OVER_STEP:
                    ends.append((d, i))
        begins, d = [(0.0, s)], 0.0
        for j in range(s + 1, n - 1):
            d += hop(j)
            if (d >= begins[-1][0] + OVER_STEP or j in starts) and j + 1 not in starts:
                begins.append((d, j))
        got = _pair(fp, poses, back, ends, begins)
        if got is None:
            stuck.append(_at(b))
            continue
        i, j = got
        cut += [(i + 1, e + 1), (s, j)]
        places.append(_at(b) + [round(sum(hop(k) for k in range(i + 1, e + 1)) +
                                      sum(hop(k) for k in range(s + 1, j + 1)), 2)])
        settled.append(_at(b))
        low = j
    if not places and not stuck:
        return plan
    report = {'places': places, 'stuck': stuck, 'unmowed_m2': 0.0}
    warnings = list(plan.warnings)
    fitted = plan.stats.get('body_fit')
    if fitted and fitted.get('unchecked') and settled:
        left = [p for p in fitted['unchecked'] if p not in settled]
        warnings = [w for w in warnings if w != unchecked_said(fitted['unchecked'])] + \
            ([unchecked_said(left)] if left else [])
        fitted = dict(fitted, unchecked=left)
    out = dataclasses.replace(plan, stats=dict(plan.stats, left_ends=report,
                                               **({'body_fit': fitted} if fitted else {})), warnings=warnings)
    if cut:
        out = _rebuilt(out, [c for c in cut if c[0] < c[1]], {})
        width = plan.config.mower_width if plan.config is not None else 0.0
        if width > 0:
            report['unmowed_m2'] = round(swept_area(plan, width).difference(swept_area(out, width)).area, 3)
        out.warnings.append(f'{len(places)} places where mower_logic would turn with no room for the body between '
                            f'sections: their ends and starts left out, {sum(m for _, _, m in places):.1f} m of the '
                            f'path, {report["unmowed_m2"]:.2f} m2 more unmowed: at ' +
                            ', '.join(f'({x:.2f}, {y:.2f})' for x, y, _ in places))
    if stuck:
        out.warnings.append(f'{len(stuck)} places where mower_logic turns between sections with no room for the body '
                            'and leaving out more doesn\'t help: at ' + ', '.join(f'({x:.2f}, {y:.2f})'
                                                                                     for x, y in stuck))
    return out


def _pair(fp, poses: List[Pose2D], back: List[bool], ends: List[Tuple[float, int]],
          begins: List[Tuple[float, int]]) -> Optional[Tuple[int, int]]:
    """(where it ends, where the next starts) of ends and begins ((m left out, pose) each) the way between fits,
    leaving the least out: BACK_OUT_MAX m of either at most first, else any OVER_FAR apart (OVER_TRIES of each)."""
    for near in (True, False):
        xs = [x for x in ends if x[0] <= BACK_OUT_MAX] if near else _every(ends, OVER_FAR)
        ys = [y for y in begins if y[0] <= BACK_OUT_MAX] if near else _every(begins, OVER_FAR)
        tried = 0
        for i, j in _least(xs, ys):
            (da, x), (db, y) = xs[i], ys[j]
            if not near and da <= BACK_OUT_MAX and db <= BACK_OUT_MAX:
                continue  # (tried before)
            if tried == OVER_TRIES:
                break
            tried += 1
            if fits_over(fp, poses[x], poses[y], back[x]):
                return x, y
    return None


def _every(xs: List[Tuple[float, int]], apart: float) -> List[Tuple[float, int]]:
    """Of xs (m, pose: m growing) the first and then those apart m on from the last kept."""
    out = xs[:1]
    for x in xs[1:]:
        if x[0] >= out[-1][0] + apart:
            out.append(x)
    return out


def _least(xs: List[Tuple[float, int]], ys: List[Tuple[float, int]]) -> Iterator[Tuple[int, int]]:
    """(k, m) of xs and ys (each by its first, growing), the sum of their firsts growing."""
    if not xs or not ys:
        return
    heap, seen = [(xs[0][0] + ys[0][0], 0, 0)], {(0, 0)}
    while heap:
        _, k, m = heapq.heappop(heap)
        yield k, m
        for nk, nm in ((k + 1, m), (k, m + 1)):
            if nk < len(xs) and nm < len(ys) and (nk, nm) not in seen:
                seen.add((nk, nm))
                heapq.heappush(heap, (xs[nk][0] + ys[nm][0], nk, nm))
