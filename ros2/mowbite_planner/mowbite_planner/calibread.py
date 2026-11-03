"""Reading a calibration recording (format mowbite-calib/1, see mowbite_planner.calib): gzip'd JSON lines, the header
first, then gps, imu, ticks or twist, cmd, pose and phase lines and the end line. Strict: a line too long, nested,
not JSON, a number not finite or a field missing is an error, so are a file and its text too big; lines of a kind not
known are skipped. Only the standard library and numpy."""

import gzip
import json
import math
import os
from array import array
from typing import Dict, List, Optional, Tuple

import numpy as np

FORMAT = 'mowbite-calib/1'
SOURCES = ('openmower', 'mowbite-os')
MODES = ('joystick', 'mow', 'program')
FIXES = ('fixed', 'float', 'single', 'none')
PHASES = ('stand', 'align', 'line', 'spin_left', 'spin_right')
EDGES = ('start', 'end')
ENDS = ('stop', 'enough', 'time', 'size', 'error')
PARAMS = ('ticks_per_m', 'wheel_distance_m', 'antenna_offset_x', 'antenna_offset_y')
# the numbers of the data lines kept (t too), the ones checked only, the ones that may be null or left out
COLUMNS = {'gps': ('x', 'y', 'acc'), 'imu': ('gx', 'gy', 'gz', 'ax', 'ay', 'az'), 'ticks': ('l', 'r'),
           'twist': ('v', 'w'), 'cmd': ('v', 'w')}
CHECKED = {'gps': ('z',), 'pose': ('x', 'y', 'yaw')}
NULLS = {'gps': ('vh', 'mh', 've', 'vn', 'rt')}
# bytes of the file at most (the recorder's 64 MB and a write more), of its text, of a line, lines, how deep the
# header may nest (data lines are flat)
MAX_FILE = 65 * 1024 * 1024
MAX_TEXT = 1024 * 1024 * 1024
MAX_LINE = 4096
MAX_LINES = 5000000
MAX_DEPTH = 3


class CalibError(ValueError):
    """A recording that can't be read, why in the message."""


def _float(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError('number out of range: %s' % text[:24])
    return value


def _int(text):
    if len(text) > 16:
        raise ValueError('integer too long: %s...' % text[:24])
    return int(text)


def _constant(text):
    raise ValueError('not a number: %s' % text)


def _depth(value) -> int:
    if isinstance(value, dict):
        return 1 + max([_depth(v) for v in value.values()] or [0])
    if isinstance(value, list):
        return 1 + max([_depth(v) for v in value] or [0])
    return 0


def _number(line: dict, key: str, no: int, null: bool = False) -> float:
    value = line.get(key)
    if value is None and null:
        return math.nan
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibError('line %d: %s is not a number' % (no, key))
    return float(value)


def _header(line: dict) -> dict:
    if line.get('format') != FORMAT:
        raise CalibError('not a %s recording' % FORMAT)
    if _depth(line) > MAX_DEPTH:
        raise CalibError('line 1: nested deeper than %d' % MAX_DEPTH)
    for key, choices in (('source', SOURCES), ('mode', MODES)):
        if line.get(key) not in choices:
            raise CalibError('line 1: %s must be one of %s' % (key, ', '.join(choices)))
    _number(line, 'started', 1)
    params = line.get('params')
    if not isinstance(params, dict):
        raise CalibError('line 1: no params')
    for name in PARAMS:
        _number(params, name, 1)
    basis = line.get('basis')
    if basis is not None and not isinstance(basis, dict):
        raise CalibError('line 1: basis is not an object')
    for got in (params, basis or {}):
        for name in ('ticks_per_m', 'wheel_distance_m'):
            if name in got and _number(got, name, 1) <= 0:
                raise CalibError('line 1: %s must be over 0' % name)
    imu = line.get('imu')
    if imu is not None:
        if not isinstance(imu, dict):
            raise CalibError('line 1: imu is not an object')
        for name in ('roll', 'pitch'):
            _number(imu, name, 1)
    if line.get('imu_frame') is not None and not isinstance(line['imu_frame'], str):
        raise CalibError('line 1: imu_frame is not a string')
    return line


class Recording:
    """A recording read: header, columns by kind ({name: numpy array}, t too, in time order), fixes (index of FIXES
    for each gps line), phases [(t, name, edge)], end (the end line, None: none), cut (it stops in the middle of the
    stream or of a line), skipped (lines of kinds not known)."""

    def __init__(self, header: dict, data: Dict[str, Dict[str, array]], fixes: array,
                 phases: List[Tuple[float, str, str]], end: Optional[dict], cut: bool, skipped: int) -> None:
        self.header, self.phases, self.end, self.cut, self.skipped = header, phases, end, cut, skipped
        self.source, self.mode = header['source'], header['mode']
        self.params = {n: float(header['params'][n]) for n in PARAMS}
        basis = header.get('basis') or {}
        self.ticks_per_m = float(basis.get('ticks_per_m', self.params['ticks_per_m']))
        self.wheel_distance = float(basis.get('wheel_distance_m', self.params['wheel_distance_m']))
        imu = header.get('imu') or {}
        self.imu_old = (imu.get('roll'), imu.get('pitch'))
        self.imu_frame = header.get('imu_frame')
        self.columns = {}
        for kind, cols in data.items():
            t = np.array(cols['t'], dtype=float)
            order = np.argsort(t, kind='mergesort')
            self.columns[kind] = {n: np.array(c, dtype=float)[order] for n, c in cols.items()}
            if kind == 'gps':
                self.fixes = np.array(fixes, dtype=np.int8)[order]




class _Lines:
    """What the data lines gave so far."""

    def __init__(self) -> None:
        self.data = {kind: {n: array('d') for n in ('t',) + names} for kind, names in COLUMNS.items()}
        self.fixes = array('b')
        self.phases = []
        self.end = None
        self.skipped = 0

    def add(self, line: dict, no: int) -> None:
        if self.end is not None:
            raise CalibError('line %d: after the end line' % no)
        if _depth(line) > 1:
            raise CalibError('line %d: nested' % no)
        t = _number(line, 't', no)
        kind = line.get('k')
        if kind in self.data:
            values = [_number(line, n, no) for n in COLUMNS[kind]]
            for n in CHECKED.get(kind, ()):
                _number(line, n, no)
            for n in NULLS.get(kind, ()):
                _number(line, n, no, null=True)
            if kind == 'gps':
                if line.get('fix') not in FIXES:
                    raise CalibError('line %d: fix must be one of %s' % (no, ', '.join(FIXES)))
                self.fixes.append(FIXES.index(line['fix']))
            cols = self.data[kind]
            cols['t'].append(t)
            for n, v in zip(COLUMNS[kind], values):
                cols[n].append(v)
        elif kind == 'pose':
            for n in CHECKED['pose']:
                _number(line, n, no)
        elif kind == 'phase':
            if line.get('name') not in PHASES or line.get('edge') not in EDGES:
                raise CalibError('line %d: phase name or edge not known' % no)
            self.phases.append((t, line['name'], line['edge']))
        elif kind == 'end':
            if line.get('why') not in ENDS:
                raise CalibError('line %d: why must be one of %s' % (no, ', '.join(ENDS)))
            reason = line.get('reason')
            self.end = {'t': t, 'why': line['why'], 'reason': reason if isinstance(reason, str) else None}
        elif isinstance(kind, str):
            self.skipped += 1
        else:
            raise CalibError('line %d: no kind (k)' % no)


def read(path: str) -> Recording:
    """The recording at path; CalibError where it isn't one or breaks a limit of the reader (MAX_*). A stream that
    stops in the middle (still written, or its writer gone) is read up to there (cut)."""
    try:
        size = os.path.getsize(path)
    except OSError as e:
        raise CalibError('no recording: %s' % e)
    if size > MAX_FILE:
        raise CalibError('recording too big: %d bytes, at most %d' % (size, MAX_FILE))
    got, header, cut = _Lines(), None, False
    lines = text = 0
    with open(path, 'rb') as raw, gzip.GzipFile(fileobj=raw) as stream:
        while True:
            try:
                chunk = stream.readline(MAX_LINE + 1)
            except EOFError:
                cut = True
                break
            except OSError as e:
                raise CalibError('not gzip or broken: %s' % e)
            if not chunk:
                break
            lines += 1
            text += len(chunk)
            if lines > MAX_LINES or text > MAX_TEXT:
                raise CalibError('recording too long: over %d lines or %d bytes' % (MAX_LINES, MAX_TEXT))
            if len(chunk) > MAX_LINE:
                raise CalibError('line %d: longer than %d bytes' % (lines, MAX_LINE))
            try:
                line = json.loads(chunk, parse_float=_float, parse_int=_int, parse_constant=_constant)
            except (ValueError, RecursionError) as e:
                if not chunk.endswith(b'\n'):  # (the last one, cut off as it was written)
                    cut = True
                    break
                raise CalibError('line %d: %s' % (lines, e))
            if not isinstance(line, dict):
                raise CalibError('line %d: not an object' % lines)
            if header is None:
                header = _header(line)
            else:
                got.add(line, lines)
    if header is None:
        raise CalibError('no header: the recording is empty')
    return Recording(header, got.data, got.fixes, sorted(got.phases), got.end, cut, got.skipped)
