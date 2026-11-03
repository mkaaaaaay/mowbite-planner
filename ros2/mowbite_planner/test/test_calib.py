"""The calibration (mowbite_planner.calib) on drives of the simulator (calibsim.py): its targets over several seeds
(ticks_per_m within 1 %, wheel_distance_m within 2 %, the antenna within 2 cm, the IMU within 1 deg), no value
without the least data, the warnings, and the reader's limits."""

import gzip
import json
import math
import os
import re
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), HERE]

import calibsim  # noqa: E402
from mowbite_planner import calib, calibfit, calibread, calibrecord  # noqa: E402
from mowbite_planner.calib import CalibError, evaluate  # noqa: E402


def values(result):
    return {v['name']: v for v in result['values']}


def errors(mower, result, spin=True):
    """Each value's error against the simulator's truth: relative for ticks and wheel distance, else in its unit;
    None where there's no value."""
    truth = {'ticks_per_m': mower.true_ticks, 'wheel_distance_m': mower.true_wheel_distance(spin),
             'antenna_offset_x': mower.antenna[0], 'antenna_offset_y': mower.antenna[1], 'imu_roll': mower.imu[0],
             'imu_pitch': mower.imu[1]}
    out = {}
    for name, v in values(result).items():
        if v['value'] is None:
            out[name] = None
        elif name in ('ticks_per_m', 'wheel_distance_m'):
            out[name] = abs(v['value'] / truth[name] - 1)
        else:
            out[name] = abs(v['value'] - truth[name])
    return out


TARGETS = {'ticks_per_m': 0.01, 'wheel_distance_m': 0.02, 'antenna_offset_x': 0.02, 'antenna_offset_y': 0.02,
           'imu_roll': math.radians(1.0), 'imu_pitch': math.radians(1.0)}


def within(results, always=('ticks_per_m', 'wheel_distance_m')):
    """Every value there is within its target, those in always there in every run, the rest in all but one."""
    missing = {}
    for errs in results:
        for name, err in errs.items():
            if err is None:
                missing[name] = missing.get(name, 0) + 1
            else:
                assert err < TARGETS[name], (name, err)
    assert not [n for n in always if n in missing], missing
    assert all(n <= 1 for n in missing.values()), missing


def run(tmp_path, mower, **kw):
    return evaluate(mower.recording(str(tmp_path / ('%s-%d.jsonl.gz' % (mower.mode, mower.seed))), **kw))


def test_program_targets(tmp_path):
    """MowBite OS's program, slip, delay, a wandering pivot, gyro off, sloping ground: all six values."""
    got = []
    for seed in range(4):
        m = calibsim.program(seed)
        result = run(tmp_path, m)
        got.append(errors(m, result))
        assert result['version'] == calib.FORMAT and result['based_on']['runs'] == 4
        assert set(values(result)) == set(TARGETS)
        info = result['info']
        assert abs(info['gnss_delay_s'] - m.delay) < 0.03 and abs(info['gyro_scale'] / m.gyro_scale - 1) < 0.005
        assert abs(info['gyro_bias'] - m.gyro_bias) < 0.001 and info['slope_deg'] > 0
    within(got, always=('ticks_per_m', 'wheel_distance_m', 'imu_roll', 'imu_pitch'))


def test_joystick_targets(tmp_path):
    """OpenMower by joystick: half the speed asked for, the stick only pushed on."""
    got = []
    for seed in range(4):
        m = calibsim.joystick(seed, odometry='ticks' if seed == 3 else 'twist')
        result = run(tmp_path, m)
        got.append(errors(m, result))
        assert set(values(result)) == set(calib.VALUES) - {'imu_roll', 'imu_pitch'}
        assert abs(result['info']['speed_ratio'] / (m.speed * m.basis[0] / m.true_ticks) - 1) < 0.05
    within(got)


def test_mow_targets(tmp_path):
    """A mow's lanes and U-turns in arcs: the wheel distance in arcs, everything good."""
    m = calibsim.mow(0, lane_m=3.0)
    result = run(tmp_path, m)
    within([errors(m, result, spin=False)], always=list(TARGETS)[:4])
    assert result['ready'] and all(v['quality'] == 'good' for v in result['values'])
    assert result['based_on']['runs'] >= 40 and result['based_on']['turns'] >= 20


def test_wheels_apart(tmp_path):
    """One wheel 4 % bigger: the runs bend, the values hold, a warning says which and how much."""
    for diff, side in ((0.04, 'left'), (-0.04, 'right')):
        m = calibsim.joystick(0, wheel_diff=diff)
        result = run(tmp_path, m)
        within([errors(m, result)], always=list(TARGETS)[:4])
        warned = [w for w in result['warnings'] if w.startswith('left/right: the %s wheel' % side)]
        assert len(warned) == 1 and abs(float(re.search(r'([0-9.]+) % smaller', warned[0])[1]) - 4.0) < 0.5, warned


def test_on_the_spot_and_arcs_apart(tmp_path):
    """Turns on the spot slipping more than arcs: both wheel distances in a warning."""
    m = calibsim.mow(1, lane_m=3.0, on_spot=range(0, 43, 2), spin_slip=0.06, arc_slip=0.01)
    result = run(tmp_path, m)
    warned = [w for w in result['warnings'] if w.startswith('wheel distance on the spot')]
    assert warned and 'proposed: both' in warned[0]


def test_too_few_data_no_value(tmp_path):
    """One run and one turn: no value, why in each, not ready."""
    m = calibsim.joystick(0, spins=(1,), run_m=5.0)
    result = run(tmp_path, m)
    assert not result['ready']
    for v in result['values']:
        assert v['value'] is None and not v['ok'] and v['warning'].startswith('too few')
    assert values(result)['ticks_per_m']['warning'].startswith('too few straight runs: 1 of 4')


def test_cut_off(tmp_path):
    """Cut off in the middle of the stream (the writer gone): read up to there, said so; interim: not said."""
    m = calibsim.program(0)
    path = m.recording(str(tmp_path / 'cut.jsonl.gz'), end=None, cut=60000)
    rec = calibread.read(path)
    assert rec.cut and rec.end is None
    result = evaluate(path)
    assert any(w.startswith('the recording stops in the middle') for w in result['warnings'])
    assert not [w for w in evaluate(path, interim=True)['warnings'] if w.startswith('the recording stops')]


def test_imu_frame_not_base_link(tmp_path):
    m = calibsim.program(0, imu_frame='imu_link')
    v = values(run(tmp_path, m))
    assert v['imu_roll']['value'] is None and 'not in base_link' in v['imu_roll']['warning']
    assert v['ticks_per_m']['ok']


def test_quality_and_limits():
    entry = calib._entry('ticks_per_m', 2000.0, 4.0, 2025.0, None)
    assert entry['quality'] == 'good' and entry['ok'] and entry['value'] == 2000.0 and entry['warning'] is None
    assert calib._entry('ticks_per_m', 2000.0, 8.0, 2025.0, None)['quality'] == 'fair'
    poor = calib._entry('ticks_per_m', 2000.0, 12.0, 2025.0, None)
    assert poor['quality'] == 'poor' and poor['value'] is None and poor['warning'].startswith('too uncertain')
    assert calib._entry('ticks_per_m', 2500.0, 4.0, 2025.0, None)['warning'].startswith('large change from 2025')
    out = calib._entry('antenna_offset_y', 0.35, 0.002, 0.0, None)
    assert out['value'] == 0.35 and not out['ok'] and out['warning'].startswith('out of range')


HEADER = {'format': 'mowbite-calib/1', 'source': 'openmower', 'mode': 'joystick', 'started': 1.7e9,
          'params': {'ticks_per_m': 2025.0, 'wheel_distance_m': 0.3556, 'antenna_offset_x': -0.1,
                     'antenna_offset_y': 0.0}}


def written(tmp_path, *lines, header=HEADER, raw=None):
    text = raw if raw is not None else '\n'.join([json.dumps(header)] + list(lines)) + '\n'
    path = tmp_path / ('rec%d.jsonl.gz' % len(list(tmp_path.iterdir())))
    path.write_bytes(gzip.compress(text.encode() if isinstance(text, str) else text))
    return str(path)


@pytest.mark.parametrize('line, why', [
    ('{"t": 1, "k": "twist", "v": NaN, "w": 0}', 'not a number'),
    ('{"t": 1, "k": "twist", "v": Infinity, "w": 0}', 'not a number'),
    ('{"t": 1, "k": "twist", "v": 1e999, "w": 0}', 'out of range'),
    ('{"t": 1, "k": "twist", "v": 0.1}', 'w is not a number'),
    ('{"t": 1, "k": "twist", "v": true, "w": 0}', 'v is not a number'),
    ('{"t": 1, "k": "gps", "x": 0, "y": 0, "z": 0, "acc": 0.01, "fix": "rtk"}', 'fix must be one of'),
    ('{"t": 1, "k": "imu", "gx": {"a": 1}}', 'nested'),
    ('[1, 2]', 'not an object'),
    ('{"t": 1, "k": "twist", "v": 0, "w": 0, "x": "' + 'a' * 5000 + '"}', 'longer than'),
    ('{"t": 1, "k": "phase", "name": "dance", "edge": "start"}', 'phase name or edge'),
    ('{"t": 1}', 'no kind'),
    ('{"t": 12345678901234567890, "k": "cmd", "v": 0, "w": 0}', 'integer too long'),
    ('[' * 2000 + ']' * 2000, 'line 2'),
])
def test_reader_refuses(tmp_path, line, why):
    with pytest.raises(CalibError, match=why):
        calibread.read(written(tmp_path, line))


@pytest.mark.parametrize('header, why', [
    (dict(HEADER, format='other/1'), 'not a mowbite-calib/1 recording'),
    (dict(HEADER, mode='drive'), 'mode must be one of'),
    (dict(HEADER, params={'ticks_per_m': 2025.0}), 'wheel_distance_m is not a number'),
    (dict(HEADER, basis={'ticks_per_m': 0, 'wheel_distance_m': 0.4}), 'ticks_per_m must be over 0'),
    (dict(HEADER, program={'a': {'b': {'c': 1}}}), 'nested deeper than 3'),
])
def test_reader_refuses_header(tmp_path, header, why):
    with pytest.raises(CalibError, match=why):
        calibread.read(written(tmp_path, header=header))


def test_reader_limits_and_order(tmp_path, monkeypatch):
    path = written(tmp_path, '{"t": 2, "k": "cmd", "v": 0, "w": 0}', '{"t": 1, "k": "cmd", "v": 1, "w": 0}',
                   '{"t": 3, "k": "news", "text": "skipped"}', '{"t": 4, "k": "end", "why": "stop"}')
    rec = calibread.read(path)
    assert list(rec.columns['cmd']['t']) == [1.0, 2.0] and list(rec.columns['cmd']['v']) == [1.0, 0.0]
    assert rec.skipped == 1 and rec.end == {'t': 4.0, 'why': 'stop', 'reason': None} and not rec.cut
    with pytest.raises(CalibError, match='after the end line'):
        calibread.read(written(tmp_path, '{"t": 4, "k": "end", "why": "stop"}', '{"t": 5, "k": "cmd", "v": 0, "w": 0}'))
    monkeypatch.setattr(calibread, 'MAX_FILE', 10)
    with pytest.raises(CalibError, match='too big'):
        calibread.read(path)
    monkeypatch.setattr(calibread, 'MAX_FILE', 1 << 20)
    monkeypatch.setattr(calibread, 'MAX_LINES', 3)
    with pytest.raises(CalibError, match='too long'):
        calibread.read(path)
    plain = tmp_path / 'plain'
    plain.write_text('plain')
    with pytest.raises(CalibError, match='not gzip'):
        calibread.read(str(plain))
    with pytest.raises(CalibError, match='no header'):
        calibread.read(written(tmp_path, raw=''))


def test_no_odometry_or_rtk(tmp_path):
    result = evaluate(written(tmp_path, '{"t": 1, "k": "imu", "gx": 0, "gy": 0, "gz": 0, "ax": 0, "ay": 0, "az": 9.8}'))
    assert not result['ready'] and all(v['warning'] == 'no odometry' for v in result['values'])
    path = written(tmp_path, *['{"t": %d, "k": "twist", "v": 0.2, "w": 0}' % t for t in range(1, 50)])
    assert all(v['warning'] == 'no RTK fixed' for v in evaluate(path)['values'])


def test_cli(tmp_path, capsys):
    m = calibsim.program(1)
    path = m.recording(str(tmp_path / 'p.jsonl.gz'))
    assert calib.main([path]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out['ready'] and len(out['values']) == 6
    assert calib.main([written(tmp_path, '{"t": 1, "k": "twist", "v": NaN, "w": 0}')]) == 1
    assert 'not a number' in json.loads(capsys.readouterr().out)['error']
    assert calib.main([]) == 2 and calib.main([path, '--fast']) == 2


def test_record_lines():
    """A number not finite: null where it may be, else the line is left out (the reader would refuse the file)."""
    assert calibrecord.line('gps', 1.0, {'x': 1.23456, 'vh': math.nan, 'fix': 'fixed'}) == \
        '{"t":1.0,"k":"gps","x":1.2346,"vh":null,"fix":"fixed"}'
    assert calibrecord.line('twist', 1.0, {'v': math.inf, 'w': 0.0}) is None
    assert calibrecord.line('twist', math.nan, {'v': 0.1, 'w': 0.0}) is None


def test_record_counts_like_the_evaluation(tmp_path):
    """Counted as it comes: a joystick drive's runs and turns about as the evaluation finds them, enough at its end
    and not before its last turn."""
    m = calibsim.joystick(2)
    rec = calibread.read(m.recording(str(tmp_path / 'j.jsonl.gz')))
    counts, twist = calibrecord.Counts(calib.MIN['joystick']), rec.columns['twist']
    enough_at = None
    for t, kind, a, b in as_it_comes(rec):
        if kind == 'gps':
            counts.gps(t, a)
            continue
        counts.add(t, a, b)
        if enough_at is None and counts.enough():
            enough_at = t
    result = evaluate(rec)
    assert counts.runs == result['based_on']['runs'] == 7
    assert abs(counts.m - result['based_on']['m']) < 2.0 and len(counts.turns[True]) == len(counts.turns[False]) == 3
    assert enough_at is not None and enough_at > twist['t'][-1] - 60
    # the progress by the names of what the mode needs, all of it there (the stand at the start: 12 s less the ends)
    got, need = counts.progress(), calibrecord.needed('joystick')
    assert set(need) - {'run_m'} <= set(got) and got['turns_left'] == got['turns_right'] == 3
    assert all(got[name] >= least for name, least in need.items() if name != 'run_m'), (got, need)
    assert need == {'m': 20.0, 'runs': 4, 'run_m': 5.0, 'still_s': 10.0, 'turns_left': 3, 'turns_right': 3,
                    'rotation_left': 2.5, 'rotation_right': 2.5, 'tight_left': 2.0, 'tight_right': 2.0}


def as_it_comes(rec, fixed=lambda t: True):
    """A recording's twist (t, 'twist', v, w) and GNSS lines (t, 'gps', counts for the evaluation, None) in time
    order, the GNSS ones without RTK fixed where fixed(t) says so."""
    twist, gps = rec.columns['twist'], rec.columns['gps']
    lines = [(float(t), 'twist', float(v), float(w)) for t, v, w in zip(twist['t'], twist['v'], twist['w'])]
    lines += [(float(t), 'gps', bool(f == 0 and acc <= calibfit.RTK_ACC and fixed(t)), None)
              for t, f, acc in zip(gps['t'], rec.fixes, gps['acc'])]
    return sorted(lines, key=lambda line: (line[0], line[1]))


def test_record_counts_only_with_rtk_fixed(tmp_path):
    """Without RTK fixed nothing counts (a mow begun in the dock: it waits for it); losing it drops what goes on (the
    run, the turn after the run before, the stand), what was counted stays."""
    rec = calibread.read(calibsim.joystick(2).recording(str(tmp_path / 'j.jsonl.gz')))

    def counted(fixed):
        counts = calibrecord.Counts(calib.MIN['joystick'])
        for t, kind, a, b in as_it_comes(rec, fixed):
            counts.gps(t, a) if kind == 'gps' else counts.add(t, a, b)
        return counts.progress()

    full = counted(lambda t: True)
    assert counted(lambda t: False) == {k: 0 if isinstance(v, int) else 0.0 for k, v in full.items()}
    t0, t1 = float(rec.columns['twist']['t'][0]), float(rec.columns['twist']['t'][-1])
    late = counted(lambda t: t > t0 + 0.5 * (t1 - t0))  # (fixed from halfway on)
    assert 0 < late['runs'] < full['runs'] and late['m'] < full['m'] and late['still_s'] < full['still_s']
    gap = counted(lambda t: not t0 + 0.4 * (t1 - t0) < t < t0 + 0.45 * (t1 - t0))  # (lost for a while)
    assert gap['runs'] < full['runs'] or gap['turns'] < full['turns']
    assert all(gap[k] <= full[k] for k in full)


def test_record_counts_the_stands():
    """Standing counts like the evaluation's stands (STAND_S s at least, STAND_TRIM off each end), the one going on
    too; without them it isn't enough."""
    counts = calibrecord.Counts(dict(calib.MIN['joystick'], runs=0, m=0.0, turns=0, rotation=0.0, antenna=0.0))
    counts.fixed_until = math.inf  # (RTK fixed all along)
    for k in range(121):  # (12 s still, then moving)
        counts.add(k * 0.1, 0.0, 0.0)
    assert counts.still_s() == pytest.approx(11.0) and counts.enough()
    counts.add(12.1, 0.2, 0.0)
    for k in range(20):  # (2 s: too short for a stand)
        counts.add(13.0 + k * 0.1, 0.005, 0.0)
    counts.add(15.0, 0.2, 0.0)
    assert counts.progress()['still_s'] == 11.0
    assert not calibrecord.Counts(calib.MIN['joystick']).enough()


def test_recorder(tmp_path, monkeypatch):
    rec = calibrecord.Recorder(str(tmp_path / 'calib'))
    params = {'ticks_per_m': 2025.0, 'wheel_distance_m': 0.3556, 'antenna_offset_x': -0.1, 'antenna_offset_y': 0.0}
    with pytest.raises(calibrecord.Refused, match='mode must be one of'):
        rec.start('program', params, 'base_link', 'x', 100.0)
    rec.start('joystick', params, 'base_link', 'x', 100.0)
    with pytest.raises(calibrecord.Refused, match='recorded already'):
        rec.start('joystick', params, 'base_link', 'x', 100.0)
    rec.add('twist', 100.5, v=0.2, w=0.0)
    rec.add('gps', 100.6, x=1.0, y=2.0, z=0.0, acc=0.01, fix='fixed', vh=None, mh=None, ve=None, vn=None)
    assert rec.tick(101.0) is None
    rec.flush()
    got = calibread.read(rec.path)  # (read while it's written: cut off there)
    assert got.cut and list(got.columns['gps']['x']) == [1.0] and got.params == params
    assert rec.tick(100.0 + calibrecord.LIMIT_S['joystick']) == ('time', None)
    monkeypatch.setattr(calibrecord, 'MAX_BYTES', 1)
    assert rec.tick(102.0) == ('size', None)
    assert rec.end('size', 103.0) is None and not rec.recording
    rec.add('twist', 104.0, v=0.2, w=0.0)  # (after the end: dropped)
    got = calibread.read(rec.path)
    assert not got.cut and got.end['why'] == 'size' and list(got.columns['twist']['t']) == [100.5]
    with open(rec.result, 'w') as f:
        f.write('{}')
    monkeypatch.setattr(calibrecord, 'FREE', 1 << 60)
    with pytest.raises(calibrecord.Refused, match='MB free'):
        rec.start('mow', params, 'base_link', 'x', 200.0)
    monkeypatch.setattr(calibrecord, 'FREE', 0)
    rec.start('mow', params, 'base_link', 'x', 200.0)  # (the one before gone, its result too)
    rec.flush()
    assert not os.path.exists(rec.result) and calibread.read(rec.path).columns['twist']['t'].size == 0
    assert rec.end('stop', 201.0) is None
