"""A small drive simulator for the calibration's tests (mowbite_planner.calib): a mower on a sloping plane, its wheels,
antenna and IMU a bit off, driven like the calibration's modes, its sensors written as a mowbite-calib/1 recording.

Off like a real one: RTK noise, GNSS delay, gyro bias and scale, a dead zone and lag in the drive, the wheels slipping
along (per run a bit apart) and when turning (on the spot more than in arcs, per turn a bit apart), a pivot off the
axle wandering while it spins, the wheels apart in size, the odometry in whole ticks with its clock jittering."""

import gzip
import json
import math

import numpy as np

from mowbite_planner.calib import FORMAT

DT = 0.01
GRAVITY = 9.80665
RATES = {'gps': 17.0, 'imu': 98.0, 'odometry': 25.0, 'cmd': 20.0}


class Mower:
    """seed draws what isn't given. ticks: ticks a metre of the wheels' surface (both on average), wheel_diff: the right
    one that much bigger than the left, wheel_distance (geometric), antenna (m, body frame), slip along, spin_slip and
    arc_slip (the rotation lost turning), pivot (m, how far it wanders off the axle spinning), delay (s, GNSS), rtk (m,
    noise each axis), gyro_bias (rad/s), gyro_scale, deadzone (m/s), speed (real / commanded), imu (roll, pitch of its
    mount), slope (the ground's rise along x and y). basis: (ticks_per_m, wheel_distance_m) the odometry is worked out
    with, params the header's (old), by default the basis and an antenna off. odometry: twist or ticks."""

    def __init__(self, seed=0, source='openmower', mode='joystick', odometry='twist', basis=None, params=None,
                 imu_frame='base_link', **kw):
        rng = self.rng = np.random.RandomState(seed)
        self.seed, self.source, self.mode, self.odometry, self.imu_frame = seed, source, mode, odometry, imu_frame
        self.ticks = kw.pop('ticks', 2025.0 * (1 + rng.uniform(-0.03, 0.03)))
        self.wheel_diff = kw.pop('wheel_diff', rng.uniform(-0.005, 0.005))
        self.wheel_distance = kw.pop('wheel_distance', 0.3556 * (1 + rng.uniform(-0.03, 0.03)))
        self.antenna = np.array(kw.pop('antenna', (-0.10 + rng.uniform(-0.05, 0.05), rng.uniform(-0.03, 0.03))))
        self.slip = kw.pop('slip', rng.uniform(0.0, 0.03))
        self.spin_slip = kw.pop('spin_slip', rng.uniform(0.0, 0.05))
        self.arc_slip = kw.pop('arc_slip', rng.uniform(0.0, 0.03))
        self.pivot = kw.pop('pivot', 0.02)
        self.delay = kw.pop('delay', rng.uniform(0.0, 0.25))
        self.rtk = kw.pop('rtk', 0.01)
        self.gyro_bias = kw.pop('gyro_bias', rng.uniform(-0.01, 0.01))
        self.gyro_scale = kw.pop('gyro_scale', 1 + rng.uniform(-0.03, 0.03))
        self.deadzone = kw.pop('deadzone', 0.02)
        self.speed = kw.pop('speed', 1.0)
        self.imu = kw.pop('imu', (rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03)))
        self.slope = kw.pop('slope', (rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03)))
        assert not kw, kw
        self.basis = basis or (round(self.ticks * (1 + rng.uniform(-0.05, 0.05)), 1),
                               round(self.wheel_distance * (1 + rng.uniform(-0.05, 0.05)), 4))
        self.params = params or {'ticks_per_m': self.basis[0], 'wheel_distance_m': self.basis[1],
                                 'antenna_offset_x': round(float(self.antenna[0]) + rng.uniform(-0.03, 0.03), 3),
                                 'antenna_offset_y': round(float(self.antenna[1]) + rng.uniform(-0.03, 0.03), 3)}
        self.t = self.x = self.y = self.th = self.vl = self.vr = self.tl = self.tr = 0.0
        self.run_slip, self.turn_slip, self.c, self.spinning = self.slip, self.spin_slip, np.zeros(2), False
        self.hist = []
        self.phases = []

    # the truth the calibration should find
    @property
    def true_ticks(self):
        left, right = self.ticks * (1 + self.wheel_diff / 2), self.ticks * (1 - self.wheel_diff / 2)
        return 1 / ((1 / left + 1 / right) / 2 * (1 - self.slip))

    def true_wheel_distance(self, spin=True):
        return self.wheel_distance / (1 - (self.spin_slip if spin else self.arc_slip))

    def step(self, v_cmd, w_cmd):
        tpm, wd = self.basis
        speeds = []
        for target in (v_cmd - w_cmd * wd / 2, v_cmd + w_cmd * wd / 2):
            target *= self.speed
            speeds.append(0.0 if abs(target) < self.deadzone else target)
        self.vl += (speeds[0] - self.vl) * DT / 0.15
        self.vr += (speeds[1] - self.vr) * DT / 0.15
        nl, nr = self.vl * tpm, self.vr * tpm
        self.tl += nl * DT
        self.tr += nr * DT
        gl = nl / (self.ticks * (1 + self.wheel_diff / 2)) * (1 - self.run_slip)
        gr = nr / (self.ticks * (1 - self.wheel_diff / 2)) * (1 - self.run_slip)
        v = (gl + gr) / 2
        moving = abs(gl) + abs(gr) > 0.005
        spin = min(1.0, max(0.0, 1 - 2 * abs(v) / (abs(gl) + abs(gr)))) if moving else 0.0
        w = (gr - gl) / self.wheel_distance * (1 - (self.turn_slip * spin + self.arc_slip * (1 - spin)))
        extra = np.zeros(2)
        if spin > 0.5:
            if not self.spinning:  # (a pivot of its own and a bit more or less slip each turn on the spot)
                r, a = self.pivot * math.sqrt(self.rng.uniform()), self.rng.uniform(0, 2 * math.pi)
                self.c = np.array([r * math.cos(a), r * math.sin(a)])
                self.turn_slip = self.spin_slip + self.rng.normal(0, 0.003)
            self.c = self.c + self.rng.normal(0, self.pivot * 0.01, 2)
            if np.hypot(*self.c) > self.pivot:
                self.c *= self.pivot / np.hypot(*self.c)
            extra = np.array([w * self.c[1], -w * self.c[0]])
        self.spinning = spin > 0.5
        c, s = math.cos(self.th), math.sin(self.th)
        bx, by = v + extra[0], extra[1]
        self.x += (c * bx - s * by) * DT
        self.y += (s * bx + c * by) * DT
        self.th += w * DT
        self.t += DT
        self.hist.append((self.t, self.x, self.y, self.th, v, w, self.tl, self.tr, v_cmd, w_cmd))

    def mark(self, name, edge):
        if self.mode == 'program':
            self.phases.append((self.t, name, edge))

    def stand(self, s, mark=True):
        if mark:
            self.mark('stand', 'start')
        for _ in range(int(round(s / DT))):
            self.step(0.0, 0.0)
        if mark:
            self.mark('stand', 'end')

    def line(self, dist, v=0.2, ramp=0.1, hold=True, stop=True):
        """dist m on (the ground's), ramping at ramp m/s2, holding the heading (else the wheels as they go)."""
        self.mark('line', 'start')
        self.run_slip = self.slip + self.rng.normal(0, 0.002)
        heading, x, y = self.th, self.x, self.y
        cmd = self.hist[-1][8] if self.hist else 0.0
        least = (self.deadzone + 0.01) / self.speed  # (the least that still moves it)
        for _ in range(int(600 / DT)):
            left = dist - math.hypot(self.x - x, self.y - y)
            if left <= 0:
                break
            want = min(v, max(least, math.sqrt(2 * ramp * left))) if stop else v
            cmd = min(want, cmd + ramp * DT)
            turn = -2.0 * ((self.th - heading + math.pi) % (2 * math.pi) - math.pi) if hold else 0.0
            self.step(cmd, turn)
        else:
            raise RuntimeError('the line never ends')
        self.mark('line', 'end')

    def spin(self, angle, rate=0.3, accel=0.6):
        self.mark('spin_left' if angle > 0 else 'spin_right', 'start')
        start, sign, cmd = self.th, math.copysign(1.0, angle), 0.0
        least = (self.deadzone + 0.01) / self.speed / (self.basis[1] / 2)
        for _ in range(int(600 / DT)):
            if abs(self.th - start) >= abs(angle):
                break
            want = min(rate, max(least, math.sqrt(2 * accel * (abs(angle) - abs(self.th - start)))))
            cmd = min(want, cmd + accel * DT)
            self.step(0.0, sign * cmd)
        else:
            raise RuntimeError('the spin never ends')
        self.mark('spin_left' if angle > 0 else 'spin_right', 'end')

    def arc(self, angle, radius, v):
        start, sign = self.th, math.copysign(1.0, angle)
        for _ in range(int(600 / DT)):
            if abs(self.th - start) >= abs(angle):
                break
            self.step(v, sign * v / radius)
        else:
            raise RuntimeError('the arc never ends')

    def recording(self, path, end='stop', cut=None):
        """Writes the recording to path (cut: only so many bytes of it, as if cut off)."""
        h = np.array(self.hist)
        t, x, y, th, v, w, tl, tr, cv, cw = h.T
        rng, t0 = self.rng, 1.7e9 + self.seed * 1000.0
        header = {'format': FORMAT, 'source': self.source, 'mode': self.mode, 'started': t0, 'params': self.params,
                  'imu_frame': self.imu_frame, 'software': 'calibsim'}
        if self.source == 'mowbite-os':
            header['basis'] = {'ticks_per_m': self.basis[0], 'wheel_distance_m': self.basis[1]}
            header['imu'] = {'roll': 0.0, 'pitch': 0.0}
        lines = []
        ts = np.arange(0.5, t[-1], 1 / RATES['gps'])
        hx, hy, hth = np.interp(ts, t, x), np.interp(ts, t, y), np.interp(ts, t, th)
        ax = hx + self.antenna[0] * np.cos(hth) - self.antenna[1] * np.sin(hth) + rng.normal(0, self.rtk, ts.size)
        ay = hy + self.antenna[0] * np.sin(hth) + self.antenna[1] * np.cos(hth) + rng.normal(0, self.rtk, ts.size)
        for k in range(ts.size):
            lines.append((t0 + ts[k] + self.delay, {'k': 'gps', 'x': round(ax[k], 4), 'y': round(ay[k], 4), 'z': 0.0,
                                                    'acc': 0.014, 'fix': 'fixed', 'vh': None, 'mh': None}))
        ts = np.arange(0.5, t[-1], 1 / RATES['imu'])
        hth, hw, hv = np.interp(ts, t, th), np.interp(ts, t, w), np.interp(ts, t, v)
        gx, gy = self.slope
        roll = self.imu[0] - np.sin(hth) * gx + np.cos(hth) * gy
        pitch = self.imu[1] - np.cos(hth) * gx - np.sin(hth) * gy
        dv = np.gradient(hv, ts)
        f = np.column_stack([-GRAVITY * np.sin(pitch) + dv, GRAVITY * np.cos(pitch) * np.sin(roll) + hv * hw,
                             GRAVITY * np.cos(pitch) * np.cos(roll)]) + rng.normal(0, 0.02, (ts.size, 3))
        gz = self.gyro_scale * hw + self.gyro_bias + rng.normal(0, 0.002, ts.size)
        for k in range(ts.size):
            lines.append((t0 + ts[k], {'k': 'imu', 'gx': round(rng.normal(0, 0.002), 5),
                                       'gy': round(rng.normal(0, 0.002), 5), 'gz': round(gz[k], 5),
                                       'ax': round(f[k, 0], 4), 'ay': round(f[k, 1], 4), 'az': round(f[k, 2], 4)}))
        ts = np.arange(0.5, t[-1], 1 / RATES['odometry'])
        left, right = np.floor(np.interp(ts, t, tl)), np.floor(np.interp(ts, t, tr))
        tpm, wd = self.basis
        stamp = t0 + ts + 0.004 + rng.uniform(0, 0.003, ts.size)
        for k in range(1, ts.size):
            if self.odometry == 'ticks':
                line = {'k': 'ticks', 'l': int(left[k]), 'r': int(right[k])}
            else:
                dl, dr = left[k] - left[k - 1], right[k] - right[k - 1]
                span = 1 / RATES['odometry']
                line = {'k': 'twist', 'v': round((dl + dr) / 2 / tpm / span, 5),
                        'w': round((dr - dl) / (tpm * wd) / span, 5)}
            lines.append((stamp[k], line))
        ts = np.arange(0.5, t[-1], 1 / RATES['cmd'])
        idx = np.searchsorted(t, ts).clip(0, t.size - 1)
        for k in range(ts.size):
            lines.append((t0 + ts[k], {'k': 'cmd', 'v': round(cv[idx[k]], 4), 'w': round(cw[idx[k]], 4)}))
        for at, name, edge in self.phases:
            lines.append((t0 + at, {'k': 'phase', 'name': name, 'edge': edge}))
        lines.sort(key=lambda line: line[0])
        text = [json.dumps(header)] + ['{"t": %.3f, %s' % (at, json.dumps(line)[1:]) for at, line in lines]
        if end:
            text.append(json.dumps({'t': round(t0 + t[-1] + 1, 3), 'k': 'end', 'why': end}))
        data = gzip.compress(('\n'.join(text) + '\n').encode(), compresslevel=1)
        with open(path, 'wb') as f:
            f.write(data[:cut] if cut else data)
        return path


def program(seed, line=3.0, **kw):
    """MowBite OS's program (the Tron's): a stand, four times a line, a stand, 540 deg on the spot (left at the far end,
    right at the start), a stand."""
    rng = np.random.RandomState(1000 + seed)
    kw.setdefault('ticks', 1000.0 * (1 + rng.uniform(-0.03, 0.03)))
    kw.setdefault('wheel_distance', 0.40 * (1 + rng.uniform(-0.03, 0.03)))
    kw.setdefault('antenna', (rng.uniform(-0.25, 0.10), rng.uniform(-0.05, 0.05)))
    m = Mower(seed, source='mowbite-os', mode='program', basis=(1000.0, 0.40),
              params={'ticks_per_m': 1000.0, 'wheel_distance_m': 0.40, 'antenna_offset_x': -0.1,
                      'antenna_offset_y': 0.0}, **kw)
    m.stand(10)
    for i in range(4):
        m.line(line, 0.2, 0.1)
        m.stand(5)
        m.spin(3 * math.pi if i % 2 == 0 else -3 * math.pi, 0.3)
        m.stand(5)
    return m


def joystick(seed, run_m=6.0, spins=(1, 1, 1, -1, -1, -1), stand=12.0, **kw):
    """By joystick on OpenMower (the NX60i's: real about half the speed asked for): a stand, runs pushing the stick
    straight on, between them a turn and a bit on the spot."""
    kw.setdefault('speed', 0.5)
    m = Mower(seed, source='openmower', mode='joystick', **kw)
    m.stand(stand, mark=False)
    m.line(run_m, 0.4, 0.3, hold=False)
    for turn in spins:
        m.stand(1.0, mark=False)
        m.spin(turn * 2.1 * math.pi, 0.5)
        m.stand(1.0, mark=False)
        m.line(run_m, 0.4, 0.3, hold=False)
    m.stand(3.0, mark=False)
    return m


def mow(seed, lanes=44, lane_m=6.0, on_spot=(), **kw):
    """A mow on OpenMower: a stand in the dock, lanes and U-turns at their ends (arcs, those in on_spot on the
    spot)."""
    kw.setdefault('speed', 0.5)
    m = Mower(seed, source='openmower', mode='mow', **kw)
    m.stand(15.0, mark=False)
    for i in range(lanes):
        m.line(lane_m, 0.5, 0.5, stop=i in on_spot or i == lanes - 1)
        if i == lanes - 1:
            break
        sign = 1 if i % 2 == 0 else -1
        if i in on_spot:
            m.stand(0.5, mark=False)
            m.spin(sign * math.pi, 0.5)
            m.stand(0.5, mark=False)
        else:
            m.arc(sign * math.pi, 0.3, 0.3)
    m.stand(3.0, mark=False)
    return m
