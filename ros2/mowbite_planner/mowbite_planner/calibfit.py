"""The calibration's fits (mowbite_planner.calib): runs, stands and turns found in the odometry, and the estimates
against RTK fixed (the antenna's points): the real way to the odometry's (sv), the real turn to the odometry's on the
spot and in arcs (k_spin, k_arc), the antenna (a), the GNSS delay (d), the gyro's bias and scale. Only numpy."""

import math
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from .calibread import CalibError, Recording

# odometry: speeds smoothed over SMOOTH s, a gap over GAP s breaks runs and turns. Straight: v >= RUN_V (m/s) and
# |w| <= RUN_CURVE * v; a run reaches RUN_EDGE s at most into slow driving straight on or a stand (|w| <= CALM_W),
# runs RUN_JOIN s apart turning less than JOIN_TURN are one, used from RUN_USE m, fitted in pieces of CHUNK_M m at
# most, each bent BEND rad/m at most more than the odometry says
SMOOTH = 1.0
GAP = 0.3
RUN_V = 0.1
RUN_CURVE = 1.0 / 3.0
RUN_EDGE = 5.0
CALM_W = 0.05
RUN_JOIN = 1.0
JOIN_TURN = 0.2
RUN_USE = 1.0
CHUNK_M = 10.0
BEND = 0.5
# a stand: |v| and |w| under STILL for STAND_S s at least, STAND_TRIM s off each end (the program's: its first
# PHASE_SETTLE s off too)
STILL = 0.01
STAND_S = 2.0
STAND_TRIM = 0.5
PHASE_SETTLE = 1.0
# headings: from the first or last HEADING_M m of a run, HEADING_POINTS points at least (a piece's too). Turns:
# TURN_MIN rad at least within TURN_S s, not back more than TURN_BACK; on the spot under SPIN_R m driven a rad, tight
# (the antenna) under TIGHT_R; the open ones (a run on one side only) end TURN_SETTLE s after the rotation does, their
# antenna fit takes TURN_POINTS points at least
HEADING_M = 1.5
HEADING_POINTS = 10
TURN_MIN = math.pi / 2
TURN_S = 60.0
TURN_BACK = 0.3
SPIN_R = 0.1
TIGHT_R = 0.5
TURN_SETTLE = 2.0
TURN_POINTS = 20
# RTK: fixed with acc <= RTK_ACC m. GNSS delay (s) searched within DELAY; the points of a piece are those from DELAY[1]
# s after it starts to -DELAY[0] s before it ends, the same for every delay tried
RTK_ACC = 0.05
DELAY = (-0.1, 0.5)
# the delay is searched with this many runs and turns at most (spread over the recording)
DELAY_PIECES = 40


def rot(phi: float) -> np.ndarray:
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s], [s, c]])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def turned(a: np.ndarray, th: np.ndarray) -> np.ndarray:
    """The body vector a turned by each heading of th (n x 2)."""
    c, s = np.cos(th), np.sin(th)
    return np.column_stack([a[0] * c - a[1] * s, a[0] * s + a[1] * c])


def cumulative(t: np.ndarray, x: np.ndarray, hold: float = 0.5) -> np.ndarray:
    """The integral of a held value (each sample until the next, hold s at most)."""
    if t.size < 2:
        return np.zeros(t.size)
    return np.concatenate([[0.0], np.cumsum(x[:-1] * np.minimum(np.diff(t), hold))])


def _spans(mask: np.ndarray) -> List[Tuple[int, int]]:
    """The runs of True in mask, (first, last) index."""
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1))


def rigid(model: np.ndarray, points: np.ndarray) -> Tuple[float, np.ndarray, float]:
    """The turn phi and shift c putting model onto points best (both n x 2): (phi, c, the squares left)."""
    mm, pm = model.mean(axis=0), points.mean(axis=0)
    a, b = model - mm, points - pm
    phi = math.atan2(float((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum()), float((a * b).sum()))
    r = rot(phi)
    c = pm - r @ mm
    return phi, c, float(((model @ r.T + c - points) ** 2).sum())


def bent(points: np.ndarray, model: Callable[[float], Tuple[np.ndarray, np.ndarray]],
         a: np.ndarray) -> Optional[Tuple[float, float, float]]:
    """points = c + R(phi) (s * body + a turned by th) best, (body, th) = model(bend): the odometry's track bent bend
    rad/m more along the way. Gauss-Newton from the rigid fit: (s, bend, the squares left), None: it doesn't settle."""
    body, th = model(0.0)
    phi, c, _ = rigid(body + turned(a, th), points)
    s, bend, n, eps = 1.0, 0.0, len(points), 1e-4
    jac = np.zeros((2 * n, 5))
    jac[0::2, 0] = 1.0
    jac[1::2, 1] = 1.0
    for _ in range(10):
        r = rot(phi)
        body, th = model(bend)
        now = s * body + turned(a, th)
        body2, th2 = model(bend + eps)
        left = now @ r.T + c - points
        cols = (now @ rot(phi + math.pi / 2).T, body @ r.T, (s * body2 + turned(a, th2) - now) @ r.T / eps)
        for i, col in enumerate(cols):
            jac[0::2, 2 + i], jac[1::2, 2 + i] = col[:, 0], col[:, 1]
        step = np.linalg.lstsq(jac, -left.ravel(), rcond=None)[0]
        c, phi, s, bend = c + step[:2], phi + step[2], s + step[3], bend + step[4]
        if abs(bend) > BEND:
            return None
        if np.abs(step[2:]).max() < 1e-9:
            break
    body, th = model(bend)
    return s, bend, float((((s * body + turned(a, th)) @ rot(phi).T + c - points) ** 2).sum())


class Odometry:
    """The wheels' way at times t: S the distance driven (m, backwards counts down) and Th the rotation (rad, left +),
    summed up from the start with the basis; v and w smoothed over SMOOTH s; gaps [(t0, t1)] without odometry (over GAP
    s, S and Th a guess across)."""

    def __init__(self, rec: Recording) -> None:
        tpm, wd = rec.ticks_per_m, rec.wheel_distance
        ticks, twist = rec.columns['ticks'], rec.columns['twist']
        source = ticks if ticks['t'].size >= 2 else twist
        t = source['t']
        if t.size < 2:
            raise CalibError('no odometry: neither ticks nor twist lines')
        keep = np.concatenate([[True], np.diff(t) > 0])
        t = t[keep]
        dt = np.diff(t)
        if source is ticks:
            left, right = ticks['l'][keep], ticks['r'][keep]
            S, Th = (left + right) / (2 * tpm), (right - left) / (tpm * wd)
        else:
            step = np.where(dt > GAP, np.median(dt), dt)
            S = np.concatenate([[0.0], np.cumsum(twist['v'][keep][1:] * step)])
            Th = np.concatenate([[0.0], np.cumsum(twist['w'][keep][1:] * step)])
        self.t, self.S, self.Th = t, S - S[0], Th - Th[0]
        self.gaps = [(t[i], t[i + 1]) for i in np.flatnonzero(dt > GAP)]
        half = SMOOTH / 2
        self.v = (self.dist(t + half) - self.dist(t - half)) / SMOOTH
        self.w = (self.turn(t + half) - self.turn(t - half)) / SMOOTH

    def dist(self, at):
        return np.interp(at, self.t, self.S)

    def turn(self, at):
        return np.interp(at, self.t, self.Th)

    def broken(self, t0: float, t1: float) -> bool:
        return any(a < t1 and b > t0 for a, b in self.gaps)

    def inside(self, t0: float, t1: float) -> np.ndarray:
        """Times from t0 to t1: both and the samples between."""
        i0, i1 = np.searchsorted(self.t, t0, side='right'), np.searchsorted(self.t, t1, side='left')
        return np.concatenate([[t0], self.t[i0:i1], [t1]])

    def at_dist(self, t0: float, t1: float, s: float) -> float:
        """The time from t0 to t1 at which S got to s (S held from going back)."""
        times = self.inside(t0, t1)
        S = np.maximum.accumulate(self.dist(times))
        k = int(np.searchsorted(S, s))
        return float(times[min(max(k, 0), len(times) - 1)])


class Turn:
    """A turn from t0 to t1: the runs before and after it (index, None: none), the odometry's rotation omega, the way
    driven in it; true: its real rotation (from the headings of both runs, else from the gyro), fit: the antenna's
    (vector, squares left)."""

    def __init__(self, t0: float, t1: float, before: Optional[int], after: Optional[int], omega: float,
                 way: float) -> None:
        self.t0, self.t1, self.before, self.after, self.omega, self.way = t0, t1, before, after, omega, way
        self.left = omega > 0
        self.spin = way < SPIN_R * abs(omega)
        self.tight = way <= TIGHT_R * abs(omega)
        self.true = None
        self.fit = None


class RunFit:
    """A run fitted: scale (real way / the odometry's), length (the odometry's), rss and points, bends (rad/m more
    than the odometry's, of its first and last piece)."""

    def __init__(self, scale: float, length: float, rss: float, points: int, bends: Tuple[float, float]) -> None:
        self.scale, self.length, self.rss, self.points, self.bends = scale, length, rss, points, bends


def _phase_spans(rec: Recording) -> List[Tuple[str, float, float]]:
    """The program's phases that ended: [(name, start, end)] in time order."""
    begun, spans = {}, []
    for t, name, edge in rec.phases:
        if edge == 'start':
            begun[name] = t
        elif name in begun:
            spans.append((name, begun.pop(name), t))
    return sorted(spans, key=lambda s: s[1])


def _program(spans, odo: Odometry):
    """The program's lines (into the stands around them, the start and the stop with them) and stands."""
    runs, stands = [], []
    for k, (name, start, end) in enumerate(spans):
        if name == 'line':
            before = max([e for n, _, e in spans[:k] if n != 'stand'] or [start - 1.0])
            after = min([s for n, s, _ in spans[k + 1:] if n != 'stand'] or [end + 1.0])
            runs.append((max(start - 1.0, before), min(end + 1.0, after)))
        elif name == 'stand' and end - start >= STAND_S + PHASE_SETTLE + STAND_TRIM:
            stands.append((start + PHASE_SETTLE, end - STAND_TRIM))
    runs = [r for r in runs if not odo.broken(*r) and odo.dist(r[1]) - odo.dist(r[0]) >= RUN_USE]
    return runs, [s for s in stands if not odo.broken(*s)]


def segments(rec: Recording, odo: Odometry) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    """Runs and stands [(t0, t1)]: the program's lines and stands, else found in the odometry."""
    spans = _phase_spans(rec) if rec.mode == 'program' else []
    if any(name == 'line' for name, _, _ in spans):
        return _program(spans, odo)
    t, v, w, n = odo.t, odo.v, odo.w, len(odo.t)
    straight = (v >= RUN_V) & (np.abs(w) <= RUN_CURVE * v)
    calm = (v > -STILL) & (np.abs(w) <= CALM_W)
    late = np.concatenate([[False], np.diff(t) > GAP])
    runs = []
    for i0, i1 in _spans(straight):
        a, b = i0, i1
        while a > 0 and calm[a - 1] and not late[a] and t[i0] - t[a - 1] <= RUN_EDGE:
            a -= 1
        while b < n - 1 and calm[b + 1] and not late[b + 1] and t[b + 1] - t[i1] <= RUN_EDGE:
            b += 1
        if runs and (t[a] <= runs[-1][1] or (t[a] - runs[-1][1] <= RUN_JOIN and not odo.broken(runs[-1][1], t[a]) and
                                             abs(odo.turn(t[a]) - odo.turn(runs[-1][1])) <= JOIN_TURN)):
            runs[-1] = (runs[-1][0], max(runs[-1][1], t[b]))
        else:
            runs.append((t[a], t[b]))
    pieces = []
    for t0, t1 in runs:  # (split where odometry is missing)
        start = t0
        for a, b in [(a, b) for a, b in odo.gaps if t0 < b and a < t1]:
            pieces.append((start, a))
            start = b
        pieces.append((start, t1))
    runs = [r for r in pieces if r[1] > r[0] and odo.dist(r[1]) - odo.dist(r[0]) >= RUN_USE]
    still = (np.abs(v) < STILL) & (np.abs(w) < STILL)
    stands = [(t[i0] + STAND_TRIM, t[i1] - STAND_TRIM) for i0, i1 in _spans(still)
              if t[i1] - t[i0] >= STAND_S + 2 * STAND_TRIM]
    return runs, [s for s in stands if not odo.broken(*s)]


def _turn(odo: Odometry, t0: float, t1: float, before: Optional[int], after: Optional[int]) -> Optional[Turn]:
    if t1 - t0 > TURN_S or t1 <= t0 or odo.broken(t0, t1):
        return None
    times = odo.inside(t0, t1)
    th = odo.turn(times)
    omega = float(th[-1] - th[0])
    if abs(omega) < TURN_MIN or float(th.max() - th.min()) > abs(omega) + TURN_BACK:
        return None
    return Turn(t0, t1, before, after, omega, float(np.abs(np.diff(odo.dist(times))).sum()))


def _settled(odo: Odometry, t0: float, t1: float, forward: bool) -> Optional[float]:
    """Where the rotation from t0 (forward) or towards t1 stops, TURN_SETTLE s on."""
    times = odo.inside(t0, t1)
    moving = np.flatnonzero(np.abs(odo.turn(times + SMOOTH / 2) - odo.turn(times - SMOOTH / 2)) > STILL * SMOOTH)
    if not moving.size:
        return None
    return min(t1, times[moving[-1]] + TURN_SETTLE) if forward else max(t0, times[moving[0]] - TURN_SETTLE)


def turns(odo: Odometry, runs: List[Tuple[float, float]]) -> List[Turn]:
    """The turns between runs, and the tight ones with a run on one side only (after the last before a long break,
    before the first after one)."""
    found, between = [], set()
    for k in range(len(runs) - 1):
        turn = _turn(odo, runs[k][1], runs[k + 1][0], k, k + 1)
        if turn is not None:
            found.append(turn)
            between.update((('after', k), ('before', k + 1)))
    start, end = float(odo.t[0]), float(odo.t[-1])
    for k, (ta, tb) in enumerate(runs):
        if ('after', k) not in between:
            limit = min([runs[k + 1][0] if k + 1 < len(runs) else end, tb + TURN_S] +
                        [a for a, _ in odo.gaps if a > tb])
            t1 = _settled(odo, tb, limit, True)
            turn = _turn(odo, tb, t1, k, None) if t1 is not None else None
            if turn is not None and turn.tight:
                found.append(turn)
        if ('before', k) not in between:
            limit = max([runs[k - 1][1] if k else start, ta - TURN_S] + [b for _, b in odo.gaps if b < ta])
            t0 = _settled(odo, limit, ta, False)
            turn = _turn(odo, t0, ta, None, k) if t0 is not None else None
            if turn is not None and turn.tight:
                found.append(turn)
    return sorted(found, key=lambda turn: turn.t0)


def ratio(group: Sequence[Turn]) -> Optional[float]:
    """The real rotation to the odometry's: each way's (true / omega summed) and their mean."""
    ways = [[t for t in group if t.left == left] for left in (True, False)]
    got = [sum(abs(t.true) for t in w) / sum(abs(t.omega) for t in w) for w in ways if w]
    return sum(got) / len(got) if got else None


def antenna_mean(group: Sequence[Turn]) -> Optional[np.ndarray]:
    """The antenna's mean from turns' fits (weighted by their rotation)."""
    if not group:
        return None
    weights = np.array([abs(t.true if t.true is not None else t.omega) for t in group])
    return (np.array([t.fit[0] for t in group]) * weights[:, None]).sum(axis=0) / weights.sum()


class Fit:
    """The estimates against RTK: sv, k_spin, k_arc, a (m, body frame), d (s); heads: the headings at each run's start
    and end, scales: the runs' fits (RunFit, None: none), turns: their real rotation (between two runs from the
    headings, else the gyro's: its bias from the stands, its scale, gyro / real, from those) and antenna fits."""

    def __init__(self, rec: Recording, odo: Odometry, runs: List[Tuple[float, float]],
                 stands: List[Tuple[float, float]]) -> None:
        gps = rec.columns['gps']
        good = (rec.fixes == 0) & (gps['acc'] <= RTK_ACC) if gps['t'].size else np.zeros(0, dtype=bool)
        self.tg, self.P = gps['t'][good], np.column_stack([gps['x'][good], gps['y'][good]])
        self.odo, self.runs, self.stands, self.turns = odo, runs, stands, turns(odo, runs)
        self.sv, self.k_spin, self.k_arc, self.d = 1.0, 1.0, 1.0, 0.1
        self.a = np.array([rec.params['antenna_offset_x'], rec.params['antenna_offset_y']])
        self.heads = [(None, None)] * len(runs)
        self.scales = [None] * len(runs)
        imu = rec.columns['imu']
        self.gyro_bias = self.gyro_scale = self.gyro = None
        if imu['t'].size >= 2 and stands:
            inside = np.zeros(imu['t'].size, dtype=bool)
            for t0, t1 in stands:
                inside[np.searchsorted(imu['t'], t0):np.searchsorted(imu['t'], t1)] = True
            if inside.sum() >= 20:
                self.gyro_bias = float(imu['gz'][inside].mean())
                self.gyro = (imu['t'], cumulative(imu['t'], imu['gz'] - self.gyro_bias, hold=0.1))

    def points(self, t0: float, t1: float) -> Tuple[np.ndarray, np.ndarray]:
        """The RTK points of a piece from t0 to t1: their times on the odometry's clock, positions."""
        j0 = np.searchsorted(self.tg, t0 + DELAY[1], side='left')
        j1 = np.searchsorted(self.tg, t1 + DELAY[0], side='right')
        return self.tg[j0:j1] - self.d, self.P[j0:j1]

    def window(self, t0: float, t1: float, ref: float, rate: float, base: float = 0.0, scale: float = 1.0,
               least: int = HEADING_POINTS):
        """A piece from t0 to t1: its RTK points and model(bend) -> (body, heading) at them, the body's track from t0
        (the odometry's way times scale), heading base + rate * (Th - Th(ref)) + bend * (S - S(ref)). None: under least
        points."""
        odo = self.odo
        tau, P = self.points(t0, t1)
        if len(P) < least:
            return None
        times = odo.inside(t0, t1)
        S, Th = odo.dist(times), odo.turn(times)
        q, way, step = Th - float(odo.turn(ref)), S - float(odo.dist(ref)), np.diff(S) * scale

        def model(bend):
            th = base + rate * q + bend * way
            mid = (th[:-1] + th[1:]) / 2
            x = np.concatenate([[0.0], np.cumsum(step * np.cos(mid))])
            y = np.concatenate([[0.0], np.cumsum(step * np.sin(mid))])
            body = np.column_stack([np.interp(tau, times, x), np.interp(tau, times, y)])
            return body, np.interp(tau, times, th)

        return P, model

    def run_fit(self, run: Tuple[float, float]) -> Optional[RunFit]:
        """A run's real way / the odometry's and how much more it bends, in pieces of CHUNK_M m at most."""
        ta, tb = run
        odo = self.odo
        s0, s1 = float(odo.dist(ta)), float(odo.dist(tb))
        count = max(1, int(math.ceil((s1 - s0) / CHUNK_M)))
        cuts = [ta] + [odo.at_dist(ta, tb, s0 + (s1 - s0) * i / count) for i in range(1, count)] + [tb]
        grown = length = rss = 0.0
        points, bends = 0, []
        for t0, t1 in zip(cuts, cuts[1:]):
            way = float(odo.dist(t1) - odo.dist(t0))
            got = self.window(t0, t1, t0, self.k_arc) if way >= RUN_USE / 2 else None
            fitted = bent(got[0], got[1], self.a) if got is not None else None
            bends.append(fitted[1] if fitted else 0.0)
            if fitted:
                grown, length, rss = grown + fitted[0] * way, length + way, rss + fitted[2]
                points += len(got[0])
        return RunFit(grown / length, length, rss, points, (bends[0], bends[-1])) if length else None

    def heading(self, k: int, end: bool) -> Optional[float]:
        """The body's heading at the end (else the start) of run k, from its last (first) HEADING_M m."""
        (ta, tb), odo, fitted = self.runs[k], self.odo, self.scales[k]
        if end:
            t0, t1, ref = odo.at_dist(ta, tb, float(odo.dist(tb)) - HEADING_M), tb, tb
        else:
            t0, t1, ref = ta, odo.at_dist(ta, tb, float(odo.dist(ta)) + HEADING_M), ta
        got = self.window(t0, t1, ref, self.k_arc, scale=self.sv) if t1 - t0 >= 1.0 else None
        if got is None:
            return None
        body, th = got[1](fitted.bends[1 if end else 0] if fitted else 0.0)
        model = body + turned(self.a, th)
        if float(np.hypot(*(model - model.mean(axis=0)).T).max()) < 0.3:
            return None
        return wrap(rigid(model, got[0])[0])

    def k_turn(self, turn: Turn) -> float:
        return self.k_spin if turn.spin else self.k_arc

    def turn_true(self, turn: Turn) -> Optional[float]:
        """A turn's real rotation from the headings of the runs on both sides."""
        if turn.before is None or turn.after is None:
            return None
        h0, h1 = self.heads[turn.before][1], self.heads[turn.after][0]
        if h0 is None or h1 is None:
            return None
        base = h1 - h0
        return base + 2 * math.pi * round((self.k_turn(turn) * turn.omega - base) / (2 * math.pi))

    def gyro_turn(self, turn: Turn) -> Optional[float]:
        """What the gyro turned in a turn (bias off)."""
        if self.gyro is None:
            return None
        t, summed = self.gyro
        return float(np.interp(turn.t1, t, summed) - np.interp(turn.t0, t, summed))

    def turn_fit(self, turn: Turn) -> Optional[Tuple[np.ndarray, float, int]]:
        """The antenna from a tight turn's points: (vector, squares left, points)."""
        if not turn.tight:
            return None
        rate = turn.true / turn.omega if turn.true is not None else self.k_turn(turn)
        if turn.before is not None and self.heads[turn.before][1] is not None:
            ref, base = turn.t0, self.heads[turn.before][1]
        elif turn.after is not None and self.heads[turn.after][0] is not None:
            ref, base = turn.t1, self.heads[turn.after][0]
        else:
            return None
        got = self.window(turn.t0, turn.t1, ref, rate, base, self.sv, TURN_POINTS)
        if got is None:
            return None
        P, (body, th) = got[0], got[1](0.0)
        if float(th.max() - th.min()) < TURN_MIN:
            return None
        design = np.zeros((2 * len(P), 4))
        design[0::2, 0] = 1.0
        design[1::2, 1] = 1.0
        c, s = np.cos(th), np.sin(th)
        design[0::2, 2], design[0::2, 3] = c, -s
        design[1::2, 2], design[1::2, 3] = s, c
        rhs = (P - body).ravel()
        x = np.linalg.lstsq(design, rhs, rcond=None)[0]
        return x[2:], float(((design @ x - rhs) ** 2).sum()), len(P)

    def turn_scales(self) -> None:
        for turn in self.turns:
            turn.true = self.turn_true(turn)
        known = [(abs(t.true), abs(self.gyro_turn(t))) for t in self.turns if t.true is not None and self.gyro]
        self.gyro_scale = sum(g for _, g in known) / sum(r for r, _ in known) if len(known) >= 2 else None
        for turn in self.turns:
            if turn.true is None and self.gyro_scale and turn.tight:
                real = self.gyro_turn(turn) / self.gyro_scale
                if real * turn.omega > 0:
                    turn.true = real
        spin = ratio([t for t in self.turns if t.true is not None and t.spin])
        arc = ratio([t for t in self.turns if t.true is not None and not t.spin])
        self.k_spin = spin or arc or 1.0
        self.k_arc = arc or spin or 1.0

    def cost(self, d: float) -> float:
        self.d = d
        few = [pieces[::max(1, len(pieces) // DELAY_PIECES)] for pieces in (self.runs, self.turns)]
        fitted = [self.run_fit(r) for r in few[0]] + [self.turn_fit(t) for t in few[1]]
        return sum(f.rss if isinstance(f, RunFit) else f[1] for f in fitted if f is not None)

    def search(self, lo: float, hi: float, step: float) -> float:
        grid = np.arange(lo, hi + step / 2, step)
        costs = [self.cost(float(d)) for d in grid]
        if not max(costs):  # (nothing to fit it with)
            return self.d
        i = int(np.argmin(costs))
        best = float(grid[i])
        if 0 < i < len(grid) - 1:  # (the parabola through the best and its neighbours)
            a, b, c = costs[i - 1], costs[i], costs[i + 1]
            if a - 2 * b + c > 0:
                best += 0.5 * step * (a - c) / (a - 2 * b + c)
        return best

    def solve(self) -> None:
        """Three passes: the delay searched (the first coarse, runs only; the second fine, the turns too), then the
        runs, their headings, the turns' rotation and the antenna each time."""
        for p in range(3):
            if p == 0:
                self.d = self.search(DELAY[0], DELAY[1], 0.05)
            elif p == 1:
                self.d = self.search(self.d - 0.05, self.d + 0.05, 0.01)
            self.scales = [self.run_fit(r) for r in self.runs]
            got = [s for s in self.scales if s is not None]
            if got:
                self.sv = sum(s.scale * s.length for s in got) / sum(s.length for s in got)
            self.heads = [(self.heading(k, False), self.heading(k, True)) for k in range(len(self.runs))]
            self.turn_scales()
            for turn in self.turns:
                turn.fit = self.turn_fit(turn)
            sides = [antenna_mean([t for t in self.turns if t.fit is not None and t.left == left])
                     for left in (True, False)]
            if sides[0] is not None and sides[1] is not None:
                self.a = (sides[0] + sides[1]) / 2
