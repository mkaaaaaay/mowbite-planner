"""Recording a drive for the calibration on OpenMower (mowbite-calib/1, read by calibread): what the ROS 1 node hands
it from OpenMower's topics, written as gzip'd JSON lines into one file of its folder (the one before goes, and its
result) within the limits (LIMIT_S by mode, MAX_BYTES, FREE); how far it got (straight runs, turns between them) for
the progress, and whether that's enough to ask the evaluation (calib.MIN). No ROS, nothing of the planner's core."""

import collections
import gzip
import json
import math
import os
import shutil
import threading
import zlib
from typing import Optional, Tuple

from .calib import MIN
from .calibfit import RUN_CURVE, RUN_USE, RUN_V, SMOOTH, TIGHT_R, TURN_MIN
from .calibread import FORMAT

RECORDING = 'recording.jsonl.gz'
RESULT = 'result.json'
# s a recording lasts at most by mode, its bytes at most, bytes free on the disk to start and to go on
LIMIT_S = {'joystick': 20 * 60.0, 'mow': 120 * 60.0}
MAX_BYTES = 64 * 1024 * 1024
FREE = 500 * 1024 * 1024
FREE_GOING = 100 * 1024 * 1024
# s between two flushes (what's written so far can be read)
FLUSH_EVERY = 5.0
# the digits a number is written with (else 4), the ones that may be null
DIGITS = {'t': 3, 'gx': 5, 'gy': 5, 'gz': 5, 'v': 5, 'w': 5}
NULLS = ('vh', 'mh', 've', 'vn', 'rt')


class Refused(ValueError):
    """A recording not started, why in the message."""


def line(kind: str, t: float, fields: dict) -> Optional[str]:
    """A data line, None where a number isn't finite (the reader would refuse the whole file)."""
    out = {'t': round(t, 3), 'k': kind}
    for key, value in fields.items():
        if isinstance(value, float):
            if not math.isfinite(value):
                if key not in NULLS:
                    return None
                value = None
            else:
                value = round(value, DIGITS.get(key, 4))
        out[key] = value
    if not math.isfinite(t):
        return None
    return json.dumps(out, separators=(',', ':'))


class Counts:
    """How far a recording got, from the odometry as it comes (like the evaluation finds it, roughly): straight runs of
    need['run_m'] m at least (runs, m), the turns between runs each way (rad) and the tight ones' rotation (the
    antenna)."""

    def __init__(self, need: dict) -> None:
        self.need, self.window, self.last = need, collections.deque(), None
        self.straight, self.had_run, self.pending = False, False, None
        self.run = self.rot = self.way = 0.0
        self.runs, self.m = 0, 0.0
        self.turns = {True: [], False: []}
        self.tight = {True: 0.0, False: 0.0}

    def add(self, t: float, v: float, w: float) -> None:
        window = self.window
        window.append((t, v, w))
        while window[0][0] < t - SMOOTH:
            window.popleft()
        vm, wm = sum(x[1] for x in window) / len(window), sum(x[2] for x in window) / len(window)
        dt = 0.0 if self.last is None else min(max(t - self.last, 0.0), 0.3)
        self.last = t
        straight = vm >= RUN_V and abs(wm) <= RUN_CURVE * vm
        if straight:
            if not self.straight:  # (a run maybe: the turn before it counts once it's long enough)
                self.pending, self.rot, self.way = (self.rot, self.way), 0.0, 0.0
            self.run += v * dt
            if self.pending is not None and self.run >= RUN_USE:
                rot, way = self.pending
                self.pending = None
                if self.had_run and abs(rot) >= TURN_MIN:
                    self.turns[rot > 0].append(abs(rot))
                    if way <= TIGHT_R * abs(rot):
                        self.tight[rot > 0] += abs(rot)
        else:
            if self.straight:
                self.end_run()
            self.rot += w * dt
            self.way += abs(v) * dt
        self.straight = straight

    def end_run(self) -> None:
        if self.run >= RUN_USE:
            self.had_run = True
            if self.run >= self.need['run_m']:
                self.runs, self.m = self.runs + 1, self.m + self.run
        elif self.pending is not None:  # (too short for a run: part of the turn)
            self.rot, self.way = self.pending[0] + self.rot, self.pending[1] + self.way + self.run
        self.run, self.pending = 0.0, None

    def enough(self) -> bool:
        need = self.need
        if self.runs < need['runs'] or self.m < need['m']:
            return False
        for left in (True, False):
            turns = self.turns[left]
            if len(turns) < need['turns'] or sum(turns) / (2 * math.pi) < need['rotation'] or \
                    self.tight[left] / (2 * math.pi) < need['antenna']:
                return False
        return True

    def progress(self) -> dict:
        turned = sum(self.turns[True]) + sum(self.turns[False])
        return {'m': round(self.m, 1), 'turns': round(turned / (2 * math.pi), 1), 'runs': self.runs}


class Recorder:
    """One recording at a time into folder/RECORDING (folder/RESULT is the evaluation's): start, add from any thread,
    tick from one (writes what came, says why it has to end), end."""

    def __init__(self, folder: str) -> None:
        self.folder = folder
        self.path, self.result = os.path.join(folder, RECORDING), os.path.join(folder, RESULT)
        self.queue = collections.deque()
        self.lock = threading.Lock()
        self.raw = self.stream = None
        self.mode, self.started, self.flushed, self.counts = None, 0.0, 0.0, None

    @property
    def recording(self) -> bool:
        return self.stream is not None

    def start(self, mode: str, params: dict, imu_frame: Optional[str], software: str, now: float) -> None:
        """A new recording (the one before and its result gone); Refused: why not."""
        if mode not in LIMIT_S:
            raise Refused('mode must be one of %s' % ', '.join(sorted(LIMIT_S)))
        with self.lock:
            if self.stream is not None:
                raise Refused('a calibration is recorded already (%s)' % self.mode)
            os.makedirs(self.folder, exist_ok=True)
            free = shutil.disk_usage(self.folder).free
            if free < FREE:
                raise Refused('only %d MB free on the mower, %d MB needed' % (free >> 20, FREE >> 20))
            for old in (self.path, self.result):
                try:
                    os.remove(old)
                except FileNotFoundError:
                    pass
            header = {'format': FORMAT, 'source': 'openmower', 'mode': mode, 'started': round(now, 3),
                      'params': params, 'imu_frame': imu_frame, 'software': software}
            self.raw = open(self.path, 'wb')
            self.stream = gzip.GzipFile(fileobj=self.raw, mode='wb')
            self.stream.write(json.dumps(header).encode() + b'\n')
            self.queue.clear()
            self.mode, self.started, self.flushed, self.counts = mode, now, now, Counts(MIN[mode])

    def add(self, kind: str, t: float, **fields) -> None:
        if self.stream is not None:
            self.queue.append((kind, t, fields))

    def drain(self, stream) -> None:
        """Writes what came into stream."""
        lines = []
        while self.queue:
            kind, t, fields = self.queue.popleft()
            if kind == 'twist':
                self.counts.add(t, fields['v'], fields['w'])
            text = line(kind, t, fields)
            if text is not None:
                lines.append(text)
        if lines:
            stream.write(('\n'.join(lines) + '\n').encode())

    def tick(self, now: float) -> Optional[Tuple[str, Optional[str]]]:
        """Writes what came: (why, reason) where it has to end (time, size or error), else None."""
        with self.lock:
            if self.stream is None:
                return None
            try:
                self.drain(self.stream)
                if now - self.flushed >= FLUSH_EVERY:
                    self.stream.flush(zlib.Z_SYNC_FLUSH)
                    self.flushed = now
                if now - self.started >= LIMIT_S[self.mode]:
                    return 'time', None
                if self.raw.tell() >= MAX_BYTES:
                    return 'size', None
                if shutil.disk_usage(self.folder).free < FREE_GOING:
                    return 'error', 'the mower\'s disk is nearly full'
            except OSError as e:
                return 'error', 'not written: %s' % e
        return None

    def flush(self) -> None:
        """All that came so far readable (an evaluation while it records)."""
        with self.lock:
            if self.stream is not None:
                self.drain(self.stream)
                self.stream.flush(zlib.Z_SYNC_FLUSH)

    def end(self, why: str, now: float, reason: Optional[str] = None) -> Optional[str]:
        """Ends it with the end line; an error where it couldn't be written to the end."""
        with self.lock:
            if self.stream is None:
                return None
            stream, raw, self.stream = self.stream, self.raw, None
            try:
                self.drain(stream)
                end = {'t': round(now, 3), 'k': 'end', 'why': why}
                if reason:
                    end['reason'] = reason
                stream.write(json.dumps(end).encode() + b'\n')
                stream.close()
                return None
            except OSError as e:
                return str(e)
            finally:
                raw.close()
