"""Recording a drive for the calibration on OpenMower (mowbite-calib/1, read by calibread): what the ROS 1 node hands
it from OpenMower's topics (the ESCs' tacho as ticks lines: Tacho), written as gzip'd JSON lines into one file of its
folder (the one before goes, and its result) within the limits (LIMIT_S by mode, MAX_BYTES, FREE); how far it got
(straight runs, turns between them each way, stands: from the gyro and the RTK track, nothing of what's calibrated) for
the progress with what the mode needs (needed), and whether that's enough to ask the evaluation (calib.MIN). No ROS,
nothing of the planner's core."""

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
from .calibfit import (CALM_W, JOIN_TURN, RTK_ACC, RUN_CURVE, RUN_EDGE, RUN_JOIN, RUN_USE, RUN_V, SMOOTH, STAND_S,
                       STAND_TRIM, STILL, TIGHT_R, TURN_BACK, TURN_MIN, TURN_S)
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
# s a GNSS line with RTK fixed counts for (the receiver sends 5 to 20 a second)
FIX_HOLD = 1.0
# counting: rad/s the gyro turns at most standing (over SMOOTH s), m a piece of the track is summed up in (its noise
# not), s a gyro sample is held at most
STILL_TURN = 0.02
PIECE = 0.2
GYRO_HOLD = 0.1
# the tacho: a step over TACHO_JUMP ticks (an ESC or the firmware started again) isn't counted
TACHO_JUMP = 20000
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


def needed(mode: str) -> dict:
    """What the mode needs (calib.MIN) by the names of Counts.progress (each way the same)."""
    need = MIN[mode]
    out = {'m': need['m'], 'runs': need['runs'], 'run_m': need['run_m'], 'still_s': need['stand_s']}
    for side in ('left', 'right'):
        out.update({'turns_' + side: need['turns'], 'rotation_' + side: need['rotation'],
                    'tight_' + side: need['antenna']})
    return out


def fixed(fix: str, acc: float) -> bool:
    """Whether a GNSS line counts for the evaluation (calibfit: RTK fixed, RTK_ACC m at most)."""
    return fix == 'fixed' and acc <= RTK_ACC


class Tacho:
    """OpenMower's ESC tacho (uint32 counting both ways and wrapping, the right one mirrored: it counts down going on)
    as the ticks of a ticks line: each side summed up from the first, forward +, a pair once both sides have a new
    count."""

    def __init__(self) -> None:
        self.last, self.summed, self.new = {}, {'l': 0, 'r': 0}, set()

    def add(self, side: str, tacho: int) -> Optional[Tuple[int, int]]:
        """side 'l' or 'r': (l, r) where that makes a pair, else None."""
        last, self.last[side] = self.last.get(side), int(tacho)
        if last is not None:
            step = (int(tacho) - last + (1 << 31)) % (1 << 32) - (1 << 31)
            if abs(step) <= TACHO_JUMP:
                self.summed[side] += step if side == 'l' else -step
        self.new.add(side)
        if len(self.new) < 2:
            return None
        self.new.clear()
        return self.summed['l'], self.summed['r']


class Counts:
    """How far a recording got, from the gyro's turn and the RTK track as they come (like the evaluation finds it,
    roughly; nothing of what's calibrated): straight runs (forward by the odometry's sign, over SMOOTH s the track at
    RUN_V m/s at least, the gyro turning RUN_CURVE rad/m at most; on through a pause of 2 RUN_EDGE + RUN_JOIN s at most
    that turns JOIN_TURN at most, not backwards, the gyro under CALM_W rad/s but for RUN_JOIN s) of need['run_m'] m at
    least (runs, m), the turns between runs each way (rad; TURN_MIN at least within TURN_S s, not back more than
    TURN_BACK) and the tight ones' rotation (the antenna: the track under TIGHT_R m a rad), the stands (s: the odometry
    under STILL m/s, its zero being one whatever's calibrated, and the gyro under STILL_TURN rad/s for STAND_S s,
    STAND_TRIM off each end; RTK's noise is as much as a slow drive). Only while the GNSS has RTK fixed (each line,
    FIX_HOLD s apart at most): without it what goes on is dropped, the run, the turn after the run before and the
    stand."""

    def __init__(self, need: dict) -> None:
        self.need, self.window = need, collections.deque()
        self.heading, self.imu_t, self.v, self.gps_t, self.anchor = 0.0, None, 0.0, None, None
        self.state, self.paused, self.had_run, self.pending, self.last_heading = 'turn', None, False, None, None
        self.swing = None  # ([since, the least heading, the most] of the turn going on)
        self.run = self.rot = self.way = 0.0
        self.runs, self.m = 0, 0.0
        self.turns = {True: [], False: []}
        self.tight = {True: 0.0, False: 0.0}
        self.stood, self.still = 0.0, None  # (s of the stands ended, [from, to] of the one going on)

    def imu(self, t: float, gz: float) -> None:
        if self.imu_t is not None:
            self.heading += gz * min(max(t - self.imu_t, 0.0), GYRO_HOLD)
        self.imu_t = t

    def odometry(self, v: float) -> None:
        """The odometry's speed: only its sign is taken (forward or backward)."""
        self.v = v

    def lost(self) -> None:
        """RTK fixed gone: what goes on dropped."""
        self.window.clear()
        self.anchor, self.last_heading, self.state, self.paused, self.had_run, self.pending, self.still = \
            None, None, 'turn', None, False, None, None
        self.swing = None
        self.run = self.rot = self.way = 0.0

    def moved(self, x: float, y: float, flush: bool = False) -> float:
        """m along the track since it was last counted: in pieces of PIECE m (else 0), flush: what's there."""
        if self.anchor is None:
            self.anchor = (x, y)
            return 0.0
        d = math.hypot(x - self.anchor[0], y - self.anchor[1])
        if d < PIECE and not flush:
            return 0.0
        self.anchor = (x, y)
        return d

    def gps(self, t: float, x: float, y: float, fixed: bool) -> None:
        if not fixed or (self.gps_t is not None and t - self.gps_t > FIX_HOLD):
            self.lost()
        self.gps_t = t if fixed else None
        if not fixed:
            return
        window = self.window
        window.append((t, x, y, self.heading))
        while window[0][0] < t - SMOOTH:
            window.popleft()
        turned = 0.0 if self.last_heading is None else self.heading - self.last_heading
        self.last_heading = self.heading
        span = t - window[0][0]
        if span < SMOOTH / 2:
            self.moved(x, y)
            return
        v = math.hypot(x - window[0][1], y - window[0][2]) / span
        w = (self.heading - window[0][3]) / span
        if abs(self.v) < STILL and abs(w) < STILL_TURN:
            self.still = [t if self.still is None else self.still[0], t]
        elif self.still is not None:
            self.stood, self.still = self.stood + self.stand(*self.still), None
        straight = self.v > 0 and v >= RUN_V and abs(w) <= RUN_CURVE * v
        state = self.state
        if state == 'pause':
            if abs(w) > CALM_W:
                self.paused[2] += t - self.paused[3]
            self.paused[3] = t
            if abs(self.heading - self.paused[1]) > JOIN_TURN or self.v <= -STILL or self.paused[2] > RUN_JOIN or \
                    t - self.paused[0] > 2 * RUN_EDGE + RUN_JOIN:
                state = 'turn'
            elif straight:
                state = 'run'
        elif state == 'run' and not straight:
            state = 'pause' if self.v > -STILL else 'turn'
        elif state == 'turn' and straight:
            state = 'run'
        gone = self.moved(x, y, flush=state != self.state)  # (at a change the piece is the one before's)
        if self.state == 'turn':
            self.way += gone
            self.rot += turned
        else:
            self.run += gone
        if state == 'pause' and self.state == 'run':
            self.paused = [t, self.heading, 0.0, t]  # (since, the heading then, s restless, the last step)
        elif state == 'turn' and self.state != 'turn':
            since, heading = (self.paused[0], self.paused[1]) if self.state == 'pause' else (t, self.heading)
            if self.state == 'pause':  # (the turn from where it paused)
                self.rot = self.heading - heading
            if self.end_run():
                self.swing = [since, heading, heading]
        if self.swing is None and 'turn' in (state, self.state):
            self.swing = [t, self.heading, self.heading]
        elif self.swing is not None:
            self.swing[1:] = min(self.swing[1], self.heading), max(self.swing[2], self.heading)
        if state == 'run' and self.state == 'turn':  # (a run maybe: the turn before it counts once it's long enough)
            swing = self.swing[2] - self.swing[1], t - self.swing[0]
            self.pending, self.rot, self.way = (self.rot, self.way) + swing, 0.0, 0.0
        elif state == 'run' and self.pending is not None and self.run >= RUN_USE:
            rot, way, swing, took = self.pending
            self.pending, self.swing = None, None
            if self.had_run and abs(rot) >= TURN_MIN and swing <= abs(rot) + TURN_BACK and took <= TURN_S:
                self.turns[rot > 0].append(abs(rot))
                if way <= TIGHT_R * abs(rot):
                    self.tight[rot > 0] += abs(rot)
        self.state = state

    def end_run(self) -> bool:
        """Whether it was a run (else it's part of the turn before it)."""
        run, self.run = self.run, 0.0
        if run >= RUN_USE:
            self.had_run = True
            if run >= self.need['run_m']:
                self.runs, self.m = self.runs + 1, self.m + run
            self.pending = None
            return True
        if self.pending is not None:  # (too short for a run: part of the turn)
            self.rot, self.way = self.pending[0] + self.rot, self.pending[1] + self.way + run
        self.pending = None
        return False

    @staticmethod
    def stand(t0: float, t1: float) -> float:
        return t1 - t0 - 2 * STAND_TRIM if t1 - t0 >= STAND_S + 2 * STAND_TRIM else 0.0

    def still_s(self) -> float:
        return self.stood + (self.stand(*self.still) if self.still is not None else 0.0)

    def counted(self) -> Tuple[int, float]:
        """(runs, m) with the run going on where it's long enough."""
        if self.state != 'turn' and self.run >= self.need['run_m']:
            return self.runs + 1, self.m + self.run
        return self.runs, self.m

    def enough(self) -> bool:
        need, (runs, m) = self.need, self.counted()
        if runs < need['runs'] or m < need['m'] or self.still_s() < need['stand_s']:
            return False
        for left in (True, False):
            turns = self.turns[left]
            if len(turns) < need['turns'] or sum(turns) / (2 * math.pi) < need['rotation'] or \
                    self.tight[left] / (2 * math.pi) < need['antenna']:
                return False
        return True

    def progress(self) -> dict:
        """{m, turns (full turns), runs, still_s, and each way (left, right): turns_ (how many), rotation_ (their full
        turns), tight_ (the tight ones' full turns)}."""
        turned, (runs, m) = sum(self.turns[True]) + sum(self.turns[False]), self.counted()
        out = {'m': round(m, 1), 'turns': round(turned / (2 * math.pi), 1), 'runs': runs,
               'still_s': round(self.still_s(), 1)}
        for left, side in ((True, 'left'), (False, 'right')):
            out.update({'turns_' + side: len(self.turns[left]),
                        'rotation_' + side: round(sum(self.turns[left]) / (2 * math.pi), 2),
                        'tight_' + side: round(self.tight[left] / (2 * math.pi), 2)})
        return out


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
            if kind == 'imu':
                self.counts.imu(t, fields['gz'])
            elif kind == 'twist':
                self.counts.odometry(fields['v'])
            elif kind == 'gps':
                self.counts.gps(t, fields['x'], fields['y'], fixed(fields['fix'], fields['acc']))
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
