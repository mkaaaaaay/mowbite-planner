"""Calibration from a recorded drive (format mowbite-calib/1): ticks_per_m, wheel_distance_m, antenna_offset_x and
antenna_offset_y, on MowBite OS imu_roll and imu_pitch too, each a proposal next to the old value with its sigma,
quality and whether it's fit to take (ok). Nothing here writes them anywhere.

    python3 -m mowbite_planner.calib [--interim] <recording>   prints the result as JSON (a recording it can't read:
                                                               {"error": why} and exit code 1)

The recording (calibread.py): from OpenMower's ROS 1 node a drive by joystick or a mow (passive), from MowBite OS its
calibration program (phase marks). Only numpy, no shapely: any thread will do.

Against RTK fixed (the antenna's points), with the odometry's track shaped by the guesses so far (calibfit.py):
  - straight runs (over 1 s v >= 0.1 m/s and |w| <= v / 3; the program's lines): the odometry's track turned, moved,
    scaled and bent onto the points gives the real way to the odometry's, ticks_per_m = basis / that. The GNSS delay
    is the one the runs and turns fit best with.
  - the heading at the start and the end of each run (its first and last 1.5 m turned and moved onto the points), a
    turn of 90 deg at least between two runs: the real turn to the odometry's, on the spot and in arcs apart (each
    way's mean), wheel_distance_m = basis * runs' / turns' (arcs where there are enough of them). A turn with a run on
    one side only: the gyro's turn, its scale from the others.
  - the antenna in each tight turn: its points against the body's track (the heading spread like the odometry's
    turn), left and right turns averaged, their difference counts into sigma (a pivot off the axle).
  - imu_roll/pitch (the IMU's tilt on level ground, atan2(ay, az) and atan2(-ax, hypot(ay, az)) of gravity): the
    stands; where the mower stood on one place in two headings the slope drops out.
sigma: bootstrap over runs and turns. quality good: sigma <= target / 4, fair: <= target / 2, else poor and no value,
nor one without the least data (MIN). ok: a value within its fixed limits (VALUES).
"""

import json
import math
import sys
from typing import List, Optional

import numpy as np

from .calibfit import CHUNK_M, RUN_V, Fit, Odometry, antenna_mean, cumulative, ratio, rot, segments, wrap
from .calibread import FORMAT, PARAMS, CalibError, Recording, read

__all__ = ['FORMAT', 'MIN', 'VALUES', 'CalibError', 'Recording', 'evaluate', 'read']

# name: unit, target (what sigma is measured against: of the value or in its unit), relative, the fixed limits, a change
# from old that warns (as target)
VALUES = {
    'ticks_per_m': ('ticks/m', 0.01, True, (100.0, 10000.0), 0.10),
    'wheel_distance_m': ('m', 0.02, True, (0.15, 0.80), 0.05),
    'antenna_offset_x': ('m', 0.02, False, (-0.60, 0.60), 0.05),
    'antenna_offset_y': ('m', 0.02, False, (-0.30, 0.30), 0.05),
    'imu_roll': ('rad', math.radians(1.0), False, (-0.087, 0.087), None),
    'imu_pitch': ('rad', math.radians(1.0), False, (-0.087, 0.087), None),
}
DIGITS = {'ticks_per_m': 1, 'wheel_distance_m': 4, 'antenna_offset_x': 3, 'antenna_offset_y': 3, 'imu_roll': 4,
          'imu_pitch': 4}
# sigma at least (of the value or in its unit): what the data can't show (RTK's slow drift, the ground)
FLOOR = {'ticks_per_m': 0.001, 'wheel_distance_m': 0.002, 'antenna_offset_x': 0.003, 'antenna_offset_y': 0.003,
         'imu_roll': math.radians(0.1), 'imu_pitch': math.radians(0.1)}
# the least data by mode: runs of run_m m at least (runs of them, m in all), turns between runs each way (turns of them,
# rotation full turns), the antenna's tight turns each way (full turns, two turns at least), stands (s, for the IMU)
MIN = {
    'joystick': {'run_m': 5.0, 'runs': 4, 'm': 20.0, 'turns': 3, 'rotation': 2.5, 'antenna': 2.0, 'stand_s': 10.0},
    'mow': {'run_m': 2.0, 'runs': 8, 'm': 50.0, 'turns': 20, 'rotation': 0.0, 'antenna': 2.0, 'stand_s': 10.0},
    'program': {'run_m': 1.5, 'runs': 4, 'm': 7.0, 'turns': 1, 'rotation': 1.5, 'antenna': 1.5, 'stand_s': 10.0},
}
# bootstrap draws and its seed; a wheel difference that warns, arcs vs on the spot that warn, a tilt of the IMU that
# warns on OpenMower; places within PLACE_R m are one, two headings PLACE_TURN apart at one tell the slope from the tilt
BOOT = 200
SEED = 1
LR_WARN = 0.02
SPIN_ARC_WARN = 0.02
IMU_WARN = math.radians(1.5)
PLACE_R = 0.5
PLACE_TURN = math.pi / 3


def _boot(rng: np.random.RandomState, n: int) -> np.ndarray:
    return rng.randint(0, n, size=(BOOT, n))


def _spread(draws: np.ndarray, n: int) -> float:
    """The bootstrap's sigma, few draws from n made up for (n / (n - 1))."""
    if n < 2:
        return math.inf
    return float(np.std(draws, ddof=1) * math.sqrt(n / (n - 1.0)))


def _entry(name: str, value: Optional[float], sigma: Optional[float], old, why: Optional[str]) -> dict:
    unit, target, relative, (low, high), large = VALUES[name]
    entry = {'name': name, 'value': None, 'unit': unit, 'old': old, 'sigma': None, 'quality': 'poor', 'ok': False,
             'warning': why or 'no estimate'}
    if value is None or sigma is None or not math.isfinite(sigma):
        return entry
    sigma = max(sigma, FLOOR[name] * (abs(value) if relative else 1.0))
    yard = target * abs(value) if relative else target
    digits = DIGITS[name]
    entry.update(sigma=round(sigma, digits + 1), warning=None,
                 quality='good' if sigma <= yard / 4 else 'fair' if sigma <= yard / 2 else 'poor')
    if entry['quality'] == 'poor':
        entry['warning'] = 'too uncertain: %.*f +- %.*f %s, needs +- %.*f' % (
            digits, value, digits + 1, sigma, unit, digits + 1, yard / 2)
        return entry
    entry['value'] = round(value, digits)
    if not low <= value <= high:
        entry['warning'] = 'out of range: %.*f %s, plausible %g to %g' % (digits, value, unit, low, high)
        return entry
    entry['ok'] = True
    if large is not None and isinstance(old, (int, float)) and not isinstance(old, bool):
        change = abs(value / old - 1) if relative and old else abs(value - old)
        if change > large:
            entry['warning'] = 'large change from %g: %s' % (old, '%.1f %%' % (100 * change) if relative else
                                                              '%.3f %s' % (change, unit))
    return entry


def _ticks(rec: Recording, fit: Fit, need: dict, rng: np.random.RandomState):
    """ticks_per_m: ((value, sigma) or why not, the bootstrap's sv draws, runs)."""
    used = [s for s in fit.scales if s is not None]
    counted = [s.length * fit.sv for s in used if s.length * fit.sv >= need['run_m']]
    if len(counted) < need['runs'] or sum(counted) < need['m']:
        return 'too few straight runs: %d of %d (%.0f m at least), %.1f of %.0f m' % (
            len(counted), need['runs'], need['run_m'], sum(counted), need['m']), None, 0
    s, length = np.array([[u.scale, u.length] for u in used]).T
    pick = _boot(rng, len(s))
    draws = (s[pick] * length[pick]).sum(axis=1) / length[pick].sum(axis=1)
    return (rec.ticks_per_m / fit.sv, _spread(rec.ticks_per_m / draws, len(s))), draws, len(s)


def _wheels(rec: Recording, fit: Fit, need: dict, rng: np.random.RandomState, sv_draws, runs: int,
            warnings: List[str]):
    """wheel_distance_m: (value, sigma) or why not; from arcs where they're enough, else on the spot, else both."""
    known = [t for t in fit.turns if t.true is not None]
    groups = [('arcs', [t for t in known if not t.spin]), ('on the spot', [t for t in known if t.spin]),
              ('both', known)]

    def enough(group):
        for left in (True, False):
            way = [t for t in group if t.left == left]
            if len(way) < need['turns'] or sum(abs(t.true) for t in way) / (2 * math.pi) < need['rotation']:
                return False
        return True

    chosen = next(((name, group) for name, group in groups if enough(group)), None)
    if chosen is None:
        count = [len([t for t in known if t.left == left]) for left in (True, False)]
        return 'too few turns between runs: %d left, %d right, of %d each way' % (count[0], count[1], need['turns'])
    ways = [[t for t in chosen[1] if t.left == left] for left in (True, False)]
    draws = []
    for way in ways:
        true, omega = np.abs([t.true for t in way]), np.abs([t.omega for t in way])
        pick = _boot(rng, len(way))
        draws.append(true[pick].sum(axis=1) / omega[pick].sum(axis=1))
    arc, spin = [ratio(group) if len(group) >= 2 else None for _, group in groups[:2]]
    if spin and arc and abs(spin / arc - 1) > SPIN_ARC_WARN:
        warnings.append('wheel distance on the spot %.4f m, in arcs %.4f m (%.1f %% apart), proposed: %s' % (
            rec.wheel_distance * fit.sv / spin, rec.wheel_distance * fit.sv / arc, 100 * abs(arc / spin - 1),
            chosen[0]))
    return (rec.wheel_distance * fit.sv / ratio(chosen[1]),
            _spread(rec.wheel_distance * sv_draws / ((draws[0] + draws[1]) / 2), min([runs] + [len(w) for w in ways])))


def _antenna(fit: Fit, need: dict, rng: np.random.RandomState) -> dict:
    """antenna_offset_x/y: (value, sigma) or why not, from tight turns each way, both ways averaged."""
    fitted = [[t for t in fit.turns if t.fit is not None and t.left == left] for left in (True, False)]
    turned = [sum(abs(t.true if t.true is not None else t.omega) for t in way) / (2 * math.pi) for way in fitted]
    if min(len(way) for way in fitted) < 2 or min(turned) < need['antenna']:
        why = 'too few tight turns: %.1f left, %.1f right (full turns, %.1f each way and two turns at least)' % (
            turned[0], turned[1], need['antenna'])
        return {'antenna_offset_x': why, 'antenna_offset_y': why}
    sides, draws = [], []
    for way in fitted:
        vectors = np.array([t.fit[0] for t in way])
        weights = np.array([abs(t.true if t.true is not None else t.omega) for t in way])
        pick = _boot(rng, len(way))
        draws.append((vectors[pick] * weights[pick][:, :, None]).sum(axis=1) / weights[pick].sum(axis=1)[:, None])
        sides.append(antenna_mean(way))
    a, mean, n = (sides[0] + sides[1]) / 2, (draws[0] + draws[1]) / 2, min(len(way) for way in fitted)
    # (left and right apart: a pivot off the axle, half of it at least)
    return {name: (float(a[i]), max(_spread(mean[:, i], n), abs(float(sides[0][i] - sides[1][i])) / 2))
            for i, name in enumerate(('antenna_offset_x', 'antenna_offset_y'))}


def _stand_heading(fit: Fit, t0: float, t1: float) -> Optional[float]:
    """The heading in a stand: from the nearest time it's known at (a run's start or end, an end of a turn whose real
    rotation is known) without turning in between."""
    odo, mid = fit.odo, (t0 + t1) / 2
    known = []
    for (ta, tb), (h0, h1) in zip(fit.runs, fit.heads):
        known += [(ta, h0), (tb, h1)]
    for turn in fit.turns:
        if turn.true is not None and turn.before is not None and fit.heads[turn.before][1] is not None:
            known.append((turn.t1, fit.heads[turn.before][1] + turn.true))
        elif turn.true is not None and turn.after is not None and fit.heads[turn.after][0] is not None:
            known.append((turn.t0, fit.heads[turn.after][0] - turn.true))
    best = None
    for at, heading in known:
        turned = float(odo.turn(mid) - odo.turn(at))
        if heading is not None and abs(turned) < 0.1 and (best is None or abs(at - mid) < best[0]):
            best = (abs(at - mid), heading + fit.k_arc * turned)
    return None if best is None else wrap(best[1])


def _gravity(rec: Recording, fit: Fit, stands):
    """The IMU's tilt on level ground from the stands with a heading: (roll, pitch, sigma, slope rad) or why not."""
    imu, heads = rec.columns['imu'], []
    for t0, t1 in stands:
        i0, i1 = np.searchsorted(imu['t'], t0), np.searchsorted(imu['t'], t1)
        heading = _stand_heading(fit, t0, t1)
        _, P = fit.points(t0, t1)
        if i1 - i0 < 20 or heading is None or len(P) < 5:
            continue
        a = [float(imu[n][i0:i1].mean()) for n in ('ax', 'ay', 'az')]
        heads.append((P.mean(axis=0) - rot(heading) @ fit.a, heading, math.atan2(a[1], a[2]),
                      math.atan2(-a[0], math.hypot(a[1], a[2]))))
    places = []
    for h in heads:
        near = [p for p in places if float(np.hypot(*(p[0][0] - h[0]))) <= PLACE_R]
        if near:
            near[0].append(h)
        else:
            places.append([h])
    places = [p for p in places if max(abs(wrap(a[1] - b[1])) for a in p for b in p) >= PLACE_TURN]
    if not places:
        return 'no place stood on in two headings (%d stands with a heading)' % len(heads)
    rows, rhs, width = [], [], 2 + 2 * len(places)
    for k, place in enumerate(places):
        for _, h, roll, pitch in place:
            # roll = mount + the ground's rise across, pitch = mount - its rise along (gx, gy at the place)
            for row, (i, gx, gy), got in (([0.0] * width, (0, -math.sin(h), math.cos(h)), roll),
                                          ([0.0] * width, (1, -math.cos(h), -math.sin(h)), pitch)):
                row[i], row[2 + 2 * k], row[3 + 2 * k] = 1.0, gx, gy
                rows.append(row)
                rhs.append(got)
    A, b = np.array(rows), np.array(rhs)
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    dof = len(b) - A.shape[1]
    noise = max(math.sqrt(float(((A @ x - b) ** 2).sum()) / dof) if dof > 0 else 0.0, FLOOR['imu_roll'])
    cov = np.linalg.pinv(A.T @ A) * noise ** 2
    slope = float(np.mean([math.hypot(x[2 + 2 * k], x[3 + 2 * k]) for k in range(len(places))]))
    return float(x[0]), float(x[1]), math.sqrt(max(cov[0, 0], cov[1, 1])), slope


def _tilt(rec: Recording, fit: Fit, stands, need: dict, info: dict, warnings: List[str]) -> dict:
    """imu_roll/pitch: (value, sigma) or why not; on OpenMower a warning where it sits tilted."""
    stood = sum(t1 - t0 for t0, t1 in stands)
    if stood < need['stand_s']:
        got = 'too little standing: %.0f of %.0f s' % (stood, need['stand_s'])
    elif rec.columns['imu']['t'].size == 0:
        got = 'no imu lines'
    elif rec.imu_frame not in (None, 'base_link'):
        got = 'IMU data not in base_link (%s)' % rec.imu_frame
    else:
        got = _gravity(rec, fit, stands)
    if not isinstance(got, tuple):
        return {'imu_roll': got, 'imu_pitch': got}
    roll, pitch, sigma, slope = got
    info['slope_deg'] = round(math.degrees(slope), 2)
    if rec.source == 'openmower' and sigma <= VALUES['imu_roll'][1] / 2 and max(abs(roll), abs(pitch)) > IMU_WARN:
        warnings.append('the IMU sits tilted: roll %.1f deg, pitch %.1f deg (OpenMower has no setting for it)' % (
            math.degrees(roll), math.degrees(pitch)))
    return {'imu_roll': (roll, sigma), 'imu_pitch': (pitch, sigma)}


def evaluate(path, interim: bool = False) -> dict:
    """The calibration from a recording (a path or a Recording): {ready, version, based_on: {m, turns (full turns),
    runs}, warnings, values: [{name, value, unit, old, sigma, quality, ok, warning}], info: {gyro_scale, gyro_bias,
    gnss_delay_s, rtk_spread_m, slope_deg, speed_ratio, turn_ratio}}. interim: one still recorded (no warning that it
    stops in the middle). CalibError: no recording that can be read."""
    rec = path if isinstance(path, Recording) else read(path)
    warnings = []
    if rec.end is None and not interim:
        warnings.append('the recording stops in the middle (no end line): evaluated up to there')
    elif rec.end is not None and rec.end['why'] == 'error':
        warnings.append('the recording ended with an error%s' % (': ' + rec.end['reason'] if rec.end['reason'] else ''))
    elif rec.end is not None and rec.end['why'] in ('time', 'size'):
        warnings.append('the recording ended at its %s limit' % rec.end['why'])
    names = list(PARAMS) + (['imu_roll', 'imu_pitch'] if rec.source == 'mowbite-os' else [])
    old = dict(rec.params, imu_roll=rec.imu_old[0], imu_pitch=rec.imu_old[1])
    info = dict.fromkeys(('gyro_scale', 'gyro_bias', 'gnss_delay_s', 'rtk_spread_m', 'slope_deg', 'speed_ratio',
                          'turn_ratio'))
    result = {'ready': False, 'version': FORMAT, 'based_on': {'m': 0.0, 'turns': 0.0, 'runs': 0},
              'warnings': warnings, 'info': info}
    try:
        odo = Odometry(rec)
    except CalibError as e:
        warnings.append(str(e))
        return dict(result, values=[_entry(n, None, None, old[n], 'no odometry') for n in names])
    runs, stands = segments(rec, odo)
    fit = Fit(rec, odo, runs, stands)
    if len(fit.tg) < 20:
        warnings.append('no RTK fixed positions (%d)' % len(fit.tg))
        return dict(result, values=[_entry(n, None, None, old[n], 'no RTK fixed') for n in names])
    fit.solve()
    need, rng = MIN[rec.mode], np.random.RandomState(SEED)
    estimates = {}
    estimates['ticks_per_m'], sv_draws, counted = _ticks(rec, fit, need, rng)
    estimates['wheel_distance_m'] = estimates['ticks_per_m'] if sv_draws is None else \
        _wheels(rec, fit, need, rng, sv_draws, counted, warnings)
    estimates.update(_antenna(fit, need, rng))
    estimates.update(_tilt(rec, fit, stands, need, info, warnings))
    values = [_entry(n, *(estimates[n] if isinstance(estimates[n], tuple) else (None, None)), old[n],
                     None if isinstance(estimates[n], tuple) else estimates[n]) for n in names]
    wanted = [v for v in values if not v['name'].startswith('imu_') or rec.imu_frame in (None, 'base_link')]
    _info(rec, fit, info)
    _warn(rec, odo, fit, warnings)
    used = [s for s in fit.scales if s is not None]
    turned = sum(abs(t.true if t.true is not None else t.omega) for t in fit.turns
                 if t.true is not None or t.fit is not None)
    return dict(result, ready=all(v['ok'] for v in wanted), values=values,
                based_on={'m': round(float(sum(s.length for s in used) * fit.sv), 1),
                          'turns': round(turned / (2 * math.pi), 1), 'runs': len(used)})


def _info(rec: Recording, fit: Fit, info: dict) -> None:
    info['gnss_delay_s'] = round(fit.d, 3)
    if fit.gyro_bias is not None:
        info['gyro_bias'] = round(fit.gyro_bias, 5)
    if fit.gyro_scale:
        info['gyro_scale'] = round(fit.gyro_scale, 4)
    spread = []
    for t0, t1 in fit.stands:
        _, P = fit.points(t0, t1)
        if len(P) >= 10:
            spread.append(((P - P.mean(axis=0)) ** 2).sum(axis=1))
    if spread:
        info['rtk_spread_m'] = round(math.sqrt(float(np.concatenate(spread).mean())), 4)
    cmd = rec.columns['cmd']
    if cmd['t'].size < 2:
        return
    v, w = cumulative(cmd['t'], cmd['v']), cumulative(cmd['t'], cmd['w'])

    def asked(summed, t0, t1):
        return float(np.interp(t1, cmd['t'], summed) - np.interp(t0, cmd['t'], summed))

    pairs = [(fit.sv * got.length, asked(v, *run)) for run, got in zip(fit.runs, fit.scales) if got is not None]
    if sum(c for _, c in pairs) > 1.0:
        info['speed_ratio'] = round(sum(r for r, _ in pairs) / sum(c for _, c in pairs), 3)
    known = [t for t in fit.turns if t.true is not None]
    turned = sum(abs(asked(w, t.t0, t.t1)) for t in known)
    if turned > 1.0:
        info['turn_ratio'] = round(sum(abs(t.true) for t in known) / turned, 3)


def _warn(rec: Recording, odo: Odometry, fit: Fit, warnings: List[str]) -> None:
    """Left and right apart (a: the wheels turn apart, b: one wheel smaller), driving mostly curved, RTK not fixed."""
    wd = rec.wheel_distance * fit.sv / fit.k_arc
    true = odom = way = asked = 0.0
    cmd = rec.columns['cmd']
    w = cumulative(cmd['t'], cmd['w']) if cmd['t'].size >= 2 else None
    for run, (h0, h1), got in zip(fit.runs, fit.heads, fit.scales):
        if h0 is None or h1 is None or got is None or got.length > 2 * CHUNK_M:
            continue
        true += wrap(h1 - h0)
        odom += fit.k_arc * float(odo.turn(run[1]) - odo.turn(run[0]))
        way += fit.sv * got.length
        if w is not None:
            asked += float(np.interp(run[1], cmd['t'], w) - np.interp(run[0], cmd['t'], w))
    if way >= 5.0:
        b = (true - odom) / way * wd
        if abs(b) >= LR_WARN:
            warnings.append('left/right: the %s wheel is in effect %.1f %% smaller (diameter, wear, dirt), straight on '
                            'the odometry it turns %s; OpenMower has one ticks_per_m for both' % (
                                'left' if b > 0 else 'right', 100 * abs(b), 'left' if b > 0 else 'right'))
        a = (odom - fit.k_arc * asked) / way * wd
        if w is not None and abs(a) >= LR_WARN:
            warnings.append('left/right: the %s wheel turns %.1f %% faster than asked (motor, gearbox, control)' %
                            ('right' if a > 0 else 'left', 100 * abs(a)))
    forward = float(np.clip(np.diff(odo.S), 0, None)[odo.v[1:] >= RUN_V].sum())
    straight = sum(got.length for got in fit.scales if got is not None)
    if forward >= 10.0 and straight < 0.5 * forward:
        warnings.append('only %.0f %% of the driving was straight (a radius of 3 m or more): the mower pulls to one '
                        'side or was steered' % (100 * straight / forward))
    moving = odo.t[(np.abs(odo.v) > 0.05) | (np.abs(odo.w) > 0.05)]
    if moving.size:
        fixed = np.isin(np.unique(np.floor(moving * 2)), np.unique(np.floor((fit.tg - fit.d) * 2)))
        if fixed.mean() < 0.8:
            warnings.append('RTK fixed only %.0f %% of the time moving' % (100 * fixed.mean()))


def main(argv: Optional[List[str]] = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    paths = [a for a in args if not a.startswith('--')]
    if len(paths) != 1 or any(a != '--interim' for a in args if a.startswith('--')):
        print('usage: python3 -m mowbite_planner.calib [--interim] <recording>', file=sys.stderr)
        return 2
    try:
        result = evaluate(paths[0], interim='--interim' in args)
    except CalibError as e:
        print(json.dumps({'error': str(e)}))
        return 1
    print(json.dumps(result, indent=1, allow_nan=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
