"""How OpenMower drives a plan, simulated: mower_logic's paths (export.plan_pieces), each one driven to its first pose
(Move Base Flex's move_base) and then along (exe_path) by OpenMower's FTC controller, its code ported with the values in
effect on the mower; the wheels following its commands late and not quite (the drive runs open loop on the duty
cycle); xbot_positioning's EKF, ported quirks and all, telling it where the mower is from GNSS, the wheels and the gyro,
each of them a bit off. What the blade mows and where the body gets out of the edges counts where the mower really
was; the track OpenMower records (position_history) is where it thought it was, thinned the way xbot_monitoring thins
it, and the events (event_history) are the ones mower_logic sends. No ROS: math and random, shapely for the checks.

MowerModel() is an ideal mower (OpenMower knows where it is, the wheels do what they're told: only the controller's
own errors, a bit better than OpenMower's simulator), MowerModel.omsim() like OpenMower's simulator, and
MowerModel.realistic(seed) like a real one with RTK (its errors calibrated on the user's tracks)."""

import collections
import dataclasses
import math
import random
from typing import Any, Callable, Deque, Dict, List, Mapping, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point
from shapely.prepared import prep
from shapely.strtree import STRtree

from .config import LANE, PERIMETER, TRANSIT, TURN, CoveragePlan, PlanningError
from .export import plan_pieces
from .footprint import Footprint
from .geometry import union
from .mathutil import Pose2D
from .planner import swept_area
from .report import drive_time, left_of

# Move Base Flex's controller loop (move_base_legacy_relay sets move_base's defaults at OpenMower's start): the first
# command of a plan comes this many s after the plan (setPlan and computeVelocityCommands in the same loop, so FTC's D
# terms see a tiny dt there)
FIRST_DT = 1e-3
# mower_logic (MowingBehavior.cpp): s it sleeps after sending a goal, s between its looks at the goal's state, the
# blade's state set every SAFETY_TICK s (checkSafety), FIRST_TRIES tries to get to a path's first pose and TRIMS times
# a pose further on, the path counts as done with fewer than PATH_DONE poses left
POLL = 0.1
SAFETY_TICK = 0.5
FIRST_TRIES = 3
TRIMS = 3
PATH_DONE = 5
# m, Move Base Flex's GlobalPlanner plans on a costmap of cells this big
COSTMAP_CELL = 0.1
# BackwardForwardRecovery: m back (else forward), m/s, s at most each way, m checked ahead in the costmap
RECOVERY_DISTANCE = 0.5
RECOVERY_SPEED = 0.6
RECOVERY_TIME = 3.0
RECOVERY_CHECK = 0.6
# a path given up after this many navigation errors on it (a real mower would keep trying: the simulation stops)
MAX_NAV_ERRORS = 20
# xbot_positioning: GNSS fixes worse than this (m) aren't used, motion vectors slower than this (m/s) neither, the
# EKF's measurement noise for positions and motion vectors (its noise model is the identity per IMU message)
MAX_GPS_ACCURACY = 0.2
MIN_SPEED = 0.01
R_POSITION = 500.0
R_MOTION = 1e4
# Hz the IMU drives the EKF at (a pose published each time)
IMU_RATE = 100.0
# m/s, rad/s the wheel speed and gyro readings come in steps of (fw diff_drive_service, LSM6DS3TR-C at 2000 dps)
ODOMETRY_STEP = 0.006
GYRO_STEP = 1.22e-3
# xbot_monitoring's position history (xbot_monitoring.cpp, PositionHistory.h): the median of x and of y every
# HISTORY_RATE s, then compacted in windows of HISTORY_WINDOW points (HISTORY_CONTEXT kept for the next): a point kept
# HISTORY_DIST m from the last kept, at a heading change over HISTORY_HEADING (moves under HISTORY_NOISE m have no
# heading) or every HISTORY_BEAT points at least, then Douglas-Peucker to HISTORY_RDP m (to the segment)
HISTORY_RATE = 0.15
HISTORY_WINDOW = 60
HISTORY_CONTEXT = 10
HISTORY_DIST = 0.1
HISTORY_NOISE = 0.03
HISTORY_HEADING = math.radians(8.0)
HISTORY_BEAT = 10
HISTORY_RDP = 0.01
# evaluate: m driven or rad turned between two looks at the body, points of the blade's way buffered at once, m within
# which places where the body got out count as one
BODY_STEP = 0.05
BODY_TURN = math.radians(5.0)
BLADE_PIECE = 400
PLACE = 0.3
# evaluate: how far off the plan, looked at every that many samples (a controller cycle each)
TRACK_EVERY = 4
# simulate's progress: said every that many controller cycles, s of a path's poses a step from mower_logic's ways to
# its first pose and its sleeps on top of drive_time's guess
PROGRESS_TICKS = 200
PIECE_SECONDS = 4.0
# evaluate: rad/s turned at m/s at most for at least s, a turn on the spot
SPIN_RATE = 0.5
SPIN_SPEED = 0.03
SPIN_TIME = 0.5

# what the mower does at a sample (Sample.mode)
APPROACH, PATH, RECOVERY = 'approach', 'path', 'recovery'


@dataclasses.dataclass(frozen=True)
class MowerModel:
    """What the simulated mower is like. The controller's values are OpenMower's ftc_local_planner.yaml (in effect on
    the mower, mower_logic leaves them alone) and Move Base Flex's (move_base's defaults, move_base_legacy_relay). The
    rest are the errors of a real mower: the drive, the GNSS and the sensors xbot_positioning goes by. Those named
    *_sigma or a spread are drawn once a run (from seed)."""
    # OpenMower's FTC controller: m/s on the way ahead staying straight for speed_fast_threshold m (no pose turned
    # more than speed_fast_threshold_angle deg from there), else speed_slow; deg/s it turns its point; m/s2 to speed up
    speed_fast: float = 0.4
    speed_slow: float = 0.15
    speed_fast_threshold: float = 1.0
    speed_fast_threshold_angle: float = 20.0
    speed_angular: float = 45.0
    acceleration: float = 0.1
    # its PIDs (on the distance ahead to its point, to the side, on the heading), the integrals' limits
    kp_lon: float = 4.0
    ki_lon: float = 0.0
    kd_lon: float = 0.0
    ki_lon_max: float = 10.0
    kp_lat: float = 25.0
    ki_lat: float = 0.06153
    kd_lat: float = 1.23
    ki_lat_max: float = 62.0
    kp_ang: float = 3.0
    ki_ang: float = 0.0
    kd_ang: float = 0.5
    ki_ang_max: float = 62.0
    # m/s, rad/s at most; m, deg off the goal that count as there, s it waits there (or turns there, or turns at the
    # start) at most; m off its point it gives up at; m to the side from which it steers mostly onto the line again
    max_cmd_vel_speed: float = 1.0
    max_cmd_vel_ang: float = 3.1
    max_goal_distance_error: float = 0.2
    max_goal_angle_error: float = 20.0
    goal_timeout: float = 20.0
    max_follow_distance: float = 1.0
    lateral_priority_distance: float = 0.1
    lateral_priority_min_gain: float = 0.1
    # Move Base Flex: Hz the controller runs at, s with no command after which it gives up
    controller_frequency: float = 20.0
    controller_patience: float = 5.0
    # mower_logic: s it sleeps after sending a goal, s the blade takes to get to speed
    goal_sleep: float = 1.0
    spinup: float = 2.0
    # the drive: m between the wheels; s the wheels start late, s they take to get to a speed (first order), m/s2 a
    # wheel speeds up at most (0: no limit); what they do of the speed commanded (open loop on the duty cycle: the
    # battery, the grass) and its spread from run to run, the right wheel's more than the left's (1 sigma a run), the
    # part lost to slip going and turning
    wheel_distance: float = 0.3556
    drive_delay: float = 0.0
    drive_tau: float = 0.0
    drive_accel: float = 0.0
    drive_gain: float = 1.0
    drive_gain_sigma: float = 0.0
    drive_asymmetry_sigma: float = 0.0
    slip: float = 0.0
    slip_turn: float = 0.0
    # positioning: False, OpenMower knows exactly where the mower is (no EKF, none of what follows)
    positioning: bool = False
    # GNSS: Hz new positions come at; once a second that position comes this many times more (the receiver's other
    # NMEA sentences: the EKF fuses each one again) and once more gps_late s later; m 1 sigma of its noise and s it
    # takes to change (0: white), m of white noise on top, m 1 sigma of a slow drift and its s, s late, jumps an hour
    # and m, s they last; m it says it's off; Hz its motion vector is new at and m/s its noise; m the antenna is ahead
    # of and left of the centre
    gps_rate: float = 10.0
    gps_burst: float = 0.0
    gps_late: float = 0.0868
    gps_sigma: float = 0.0
    gps_white: float = 0.0
    gps_tau: float = 0.0
    gps_drift: float = 0.0
    gps_drift_tau: float = 900.0
    gps_latency: float = 0.0
    gps_jumps: float = 0.0
    gps_jump: float = 0.2
    gps_jump_time: float = 0.3
    gps_accuracy: float = 0.02
    motion_vector_rate: float = 1.0
    motion_vector_sigma: float = 0.0
    antenna_offset_x: float = -0.10
    antenna_offset_y: float = 0.0
    # the wheel speeds and the gyro read in steps (a real mower's wheel ticks and gyro) or not (OpenMower's simulator)
    quantize: bool = True
    # wheel odometry: what it says of the speed (ticks_per_m off: < 1 says too little), Hz, m/s noise
    odometry_scale: float = 1.0
    odometry_rate: float = 25.0
    odometry_sigma: float = 0.0
    # gyro: rad/s 1 sigma of the bias left after calibrating (a run), 1 sigma of its scale error (a run), its noise
    gyro_bias_sigma: float = 0.0
    gyro_scale_sigma: float = 0.0
    gyro_sigma: float = 0.0
    seed: int = 0

    @classmethod
    def names(cls) -> Tuple[str, ...]:
        return tuple(f.name for f in dataclasses.fields(cls))

    def with_overrides(self, values: Mapping[str, Any]) -> 'MowerModel':
        """A copy with values (by name) changed, checked (PlanningError for unknown names and bad values)."""
        unknown = sorted(set(values) - set(self.names()))
        if unknown:
            raise PlanningError('unknown mower model values: ' + ', '.join(unknown))
        cast = {}
        for f in dataclasses.fields(self):
            if f.name in values:
                v = values[f.name]
                try:
                    cast[f.name] = bool(v) if f.type in (bool, 'bool') else int(v) if f.type in (int, 'int') \
                        else float(v)
                except (TypeError, ValueError):
                    raise PlanningError('mower model %s: %r is no number' % (f.name, v)) from None
        model = dataclasses.replace(self, **cast)
        model.validate()
        return model

    def validate(self) -> None:
        for f in dataclasses.fields(self):
            v = getattr(self, f.name)
            if isinstance(v, float) and not math.isfinite(v):
                raise PlanningError('mower model %s: %r' % (f.name, v))
        for name in ('speed_fast', 'speed_slow', 'speed_angular', 'acceleration', 'controller_frequency',
                     'wheel_distance', 'gps_rate', 'odometry_rate', 'motion_vector_rate', 'drive_gain'):
            if getattr(self, name) <= 0:
                raise PlanningError('mower model %s must be above 0' % name)
        for name in ('drive_delay', 'drive_tau', 'drive_accel', 'drive_gain_sigma', 'drive_asymmetry_sigma',
                     'gps_sigma', 'gps_white', 'gps_late', 'gps_tau', 'gps_drift', 'gps_latency', 'gps_jumps',
                     'gps_jump_time', 'motion_vector_sigma', 'odometry_sigma', 'gyro_bias_sigma', 'gyro_scale_sigma',
                     'gyro_sigma', 'goal_sleep', 'spinup', 'controller_patience', 'goal_timeout'):
            if getattr(self, name) < 0:
                raise PlanningError('mower model %s must not be below 0' % name)
        if not 0 <= self.slip < 1 or not 0 <= self.slip_turn < 1 or not 0 <= self.gps_burst <= 10:
            raise PlanningError('mower model slip and slip_turn must lie in 0..1, gps_burst in 0..10')
        if self.gps_latency > 1.0 or self.drive_delay > 1.0:
            raise PlanningError('mower model gps_latency and drive_delay must not be above 1 s')

    @classmethod
    def omsim(cls, seed: int = 0) -> 'MowerModel':
        """Like OpenMower's simulator (mower_simulation): the twist commanded driven exactly, white noise on the
        odometry (50 Hz), the gyro and the GNSS (5 Hz, every position once, the motion vector from the noisy speeds),
        the antenna where its custom_params put it."""
        return cls(positioning=True, gps_rate=5.0, gps_sigma=0.005, motion_vector_rate=5.0,
                   motion_vector_sigma=0.02, antenna_offset_x=0.3, quantize=False, odometry_rate=50.0,
                   odometry_sigma=0.02, gyro_sigma=0.02, seed=seed)

    @classmethod
    def realistic(cls, seed: int = 0) -> 'MowerModel':
        """Like the user's mower (YardForce, OpenMower v2, RTK by NTRIP VRS), fitted to dense logs of real mows
        (xb_pose at 100 Hz and the raw GNSS, 2026-09): on straight lanes where the mower thinks it is wobbles 0.42 cm
        (p50) / 1.32 (p95) / 1.70 (p99) off a line fitted to each lane (simulated 0.43 / 1.27 / 1.71), its heading
        1.24 deg (1.56), the GNSS 6.9 mm across the estimate (7.2). The odometry says 8-10 % too little (wheel ticks
        on grass), the GNSS fix comes 10 times a second and once a second 3 times more and once 87 ms later. Its blade
        on, the mower mows 11.4-12.0 m a minute (the user's mows in 2026-10, simulated on today's plans of the same
        garden 11.4-12.0). What those logs can't show (where the mower really is) are priors: the drive (the firmware
        sends a command within 40 ms), slip turning, the GNSS 60 ms late, drifting 1 cm over a quarter of an hour.
        Coming out of a turn the real mower was off more than this one (3.0 cm past the next lane against 2.2 on
        turns like the slic3r planner's) and the real tracks of 2026-10 lie a bit further off their lanes (0.5-1.0 cm
        p50 against 0.4): a bit better than the real one there."""
        return cls(drive_delay=0.02, drive_accel=1.0, drive_gain_sigma=0.04, drive_asymmetry_sigma=0.02, slip=0.02,
                   slip_turn=0.2, positioning=True, gps_burst=3.1, gps_sigma=0.011, gps_tau=1.5, gps_white=0.002,
                   gps_drift=0.01, gps_latency=0.06, motion_vector_sigma=0.01, odometry_scale=0.92,
                   gyro_bias_sigma=1e-4, gyro_scale_sigma=0.01, gyro_sigma=0.005, seed=seed)


@dataclasses.dataclass
class SimResult:
    """A simulated mow. samples: one each controller cycle, (t, x, y, yaw, ex, ey, eyaw, blade, mode, plan index):
    where the mower was, where it thought it was, whether the blade mowed, what it was doing (APPROACH, PATH,
    RECOVERY), the plan pose its controller was at (-1 off the plan). events: mower_logic's events like in
    event_history. history: the lines of a position_history file. nav_errors: [(t, plan index)]. aborted: mower_logic
    gave up (no way to a path's start)."""
    samples: List[Tuple] = dataclasses.field(default_factory=list)
    events: List[Dict[str, Any]] = dataclasses.field(default_factory=list)
    history: List[Dict[str, Any]] = dataclasses.field(default_factory=list)
    nav_errors: List[Tuple[float, int]] = dataclasses.field(default_factory=list)
    aborted: bool = False
    model: Optional[MowerModel] = None
    t0: float = 0.0
    duration: float = 0.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(x: float, m: float) -> float:
    return -m if x < -m else m if x > m else x


# FTC's states (ftc_planner.h)
_PRE_ROTATE, _FOLLOWING, _WAITING, _POST_ROTATE, _FINISHED = range(5)


class _Ftc:
    """OpenMower's FTCPlanner (ftc_planner.cpp, upstream a69d144) as Move Base Flex runs it: a point running along
    the plan at its own speed (whatever the mower does, until it's max_follow_distance off), the mower steered onto it
    by PIDs. Ported line by line, quirks kept: the errors of the cycle before survive a new plan (a D spike in the
    first cycle), the integrals add up in every state, the last pose doubled with the heading of the one before."""

    def __init__(self, m: MowerModel):
        self.m = m
        self.last_lat = self.last_lon = self.last_ang = 0.0
        self.state = _PRE_ROTATE
        self.cur_speed = m.speed_slow
        self.fast_angle = math.radians(m.speed_fast_threshold_angle)
        self.goal_angle = math.radians(m.max_goal_angle_error)
        self.turn_rate = math.radians(m.speed_angular)

    def set_plan(self, plan: Sequence[Tuple[float, float, float]], now: float) -> None:
        self.state, self.t_state, self.crashed = _PRE_ROTATE, now, False
        self.p = [tuple(q) for q in plan]
        self.idx, self.prog = 0, 0.0
        self.last_time = now
        self.cur_speed = self.m.speed_slow
        self.lat = self.lon = self.ang = 0.0
        self.i_lon = self.i_lat = self.i_ang = 0.0
        if len(self.p) > 2:
            self.p.append(self.p[-1])
            x, y, _ = self.p[-2]
            self.p[-2] = (x, y, self.p[-3][2])
        else:  # (fewer than 3 poses: done at once)
            self.state = _FINISHED
        self.cp = self.p[0]

    def reached(self) -> bool:
        return self.state == _FINISHED and not self.crashed

    def compute(self, pose: Tuple[float, float, float], now: float) -> Tuple[float, float, bool]:
        """(v, w, ok): the command for the mower at pose (where it thinks it is)."""
        dt = now - self.last_time
        self.last_time = now
        if self.crashed:
            return 0.0, 0.0, False
        if self.state == _FINISHED:
            return 0.0, 0.0, True
        self._control_point(dt, pose)
        new = self._next_state(now)
        if new != self.state:
            self.state, self.t_state = new, now
        v, w = self._velocity(dt)
        if self.crashed:
            return 0.0, 0.0, False
        return v, w, True

    def _lookahead(self) -> float:
        m, (cx, cy, ca) = self.m, self.cp
        d, lx, ly = 0.0, cx, cy
        for i in range(self.idx + 1, len(self.p)):
            x, y, a = self.p[i]
            if d > m.speed_fast_threshold or abs(_wrap(a - ca)) > self.fast_angle:
                break
            d += math.hypot(x - lx, y - ly)
            lx, ly = x, y
        return d

    def _control_point(self, dt: float, pose: Tuple[float, float, float]) -> None:
        m, n = self.m, len(self.p)
        if self.state == _PRE_ROTATE:
            self.cp = self.p[0]
        elif self.state == _FOLLOWING:
            target = m.speed_fast if self._lookahead() >= m.speed_fast_threshold else m.speed_slow
            if target > self.cur_speed:
                self.cur_speed = min(target, self.cur_speed + dt * m.acceleration)
            elif target < self.cur_speed:
                self.cur_speed = max(target, self.cur_speed - dt * m.acceleration)
            dm, am = dt * self.cur_speed, dt * self.turn_rate
            while am > 0 and dm > 0 and self.idx < n - 2:
                x0, y0, a0 = self.p[self.idx]
                x1, y1, a1 = self.p[self.idx + 1]
                pd = math.hypot(x1 - x0, y1 - y0)
                pa = abs(_wrap(a1 - a0))
                if pd <= 0.0:
                    self.idx += 1
                    continue
                rd, ra = pd * (1.0 - self.prog), pa * (1.0 - self.prog)
                if rd < dm and ra < am:
                    self.prog = 0.0
                    self.idx += 1
                    dm -= rd
                    am -= ra
                else:
                    pdist = (pd * self.prog + dm) / pd
                    pang = (pa * self.prog + am) / pa if pa > 0 else math.inf
                    self.prog = min(pang, pdist)
                    dm = am = 0.0
            x0, y0, a0 = self.p[self.idx]
            x1, y1, a1 = self.p[self.idx + 1]
            t = self.prog
            self.cp = ((1 - t) * x0 + t * x1, (1 - t) * y0 + t * y1, a0 + _wrap(a1 - a0) * t)
        elif self.state == _POST_ROTATE:
            self.cp = self.p[-1]
        rx, ry, ra = pose
        dx, dy = self.cp[0] - rx, self.cp[1] - ry
        c, s = math.cos(ra), math.sin(ra)
        self.lon = c * dx + s * dy
        self.lat = -s * dx + c * dy
        self.ang = _wrap(self.cp[2] - ra)

    def _next_state(self, now: float) -> int:
        m, since = self.m, now - self.t_state
        if self.state == _PRE_ROTATE:
            if since > m.goal_timeout:
                self.crashed = True
                return _FINISHED
            if abs(self.ang) < self.goal_angle:
                return _FOLLOWING
        elif self.state == _FOLLOWING:
            if math.hypot(self.lon, self.lat) > m.max_follow_distance:
                self.crashed = True
                return _FINISHED
            if self.idx == len(self.p) - 2:
                return _WAITING
        elif self.state == _WAITING:
            if since > m.goal_timeout or math.hypot(self.lon, self.lat) < m.max_goal_distance_error:
                return _POST_ROTATE
        elif self.state == _POST_ROTATE:
            if since > m.goal_timeout or abs(self.ang) < self.goal_angle:
                return _FINISHED
        return self.state

    def _velocity(self, dt: float) -> Tuple[float, float]:
        m = self.m
        if self.state == _FINISHED or self.crashed:
            return 0.0, 0.0
        lat, lon, ang = self.lat, self.lon, self.ang
        self.i_lon = _clamp(self.i_lon + lon * dt, m.ki_lon_max)
        self.i_lat = _clamp(self.i_lat + lat * dt, m.ki_lat_max)
        self.i_ang = _clamp(self.i_ang + ang * dt, m.ki_ang_max)
        d_lat = (lat - self.last_lat) / dt
        d_lon = (lon - self.last_lon) / dt
        d_ang = (ang - self.last_ang) / dt
        self.last_lat, self.last_lon, self.last_ang = lat, lon, ang
        if self.state == _FOLLOWING:
            v = lon * m.kp_lon + self.i_lon * m.ki_lon + d_lon * m.kd_lon
            if v < 0:  # (forward_only)
                v = 0.0
            else:
                v = min(v, m.max_cmd_vel_speed)
            g = 1.0
            if m.lateral_priority_distance > 0:
                g = max(m.lateral_priority_min_gain,
                        (m.lateral_priority_distance - abs(lat)) / m.lateral_priority_distance)
            w = g * (ang * m.kp_ang + self.i_ang * m.ki_ang + d_ang * m.kd_ang) \
                + lat * m.kp_lat + self.i_lat * m.ki_lat + d_lat * m.kd_lat
        else:  # (turning on the spot: before following, at the end)
            v = 0.0
            w = ang * m.kp_ang + self.i_ang * m.ki_ang + d_ang * m.kd_ang
        return v, _clamp(w, m.max_cmd_vel_ang)


class _Ekf:
    """xbot_positioning's EKF (upstream a69d144) on [x, y, theta, vx, vr]: predicted from the wheels' speed and the gyro
    every IMU message (its process noise the identity each time, so how often counts), corrected by GNSS positions
    (the antenna's, R 500, no lever arm term in H) and motion vectors (R 1e4, only the theta column in H). vx and vr
    only ever meet the rest in the motion vector's prediction: they're kept apart (a filter each), the rest as a 3x3
    covariance, the same numbers as the 5x5 one, a lot quicker."""

    def __init__(self, m: MowerModel, x: float, y: float, yaw: float):
        self.ox, self.oy = m.antenna_offset_x, m.antenna_offset_y
        self.x, self.y, self.th, self.vx, self.vr = x, y, yaw, 0.0, 0.0
        self.p = [1.0, 0.0, 0.0, 1.0, 0.0, 1.0]  # P00 P01 P02 P11 P12 P22
        self.pv = self.pw = 1.0  # P33, P44

    def predict(self, v: float, w: float, dt: float) -> None:
        th = self.th + w * dt
        a, b = -math.sin(th) * v * dt, math.cos(th) * v * dt
        p00, p01, p02, p11, p12, p22 = self.p
        self.p = [p00 + 2 * a * p02 + a * a * p22 + 1.0, p01 + a * p12 + b * p02 + a * b * p22, p02 + a * p22,
                  p11 + 2 * b * p12 + b * b * p22 + 1.0, p12 + b * p22, p22 + 1.0]
        self.x += math.cos(th) * v * dt
        self.y += math.sin(th) * v * dt
        self.th = th
        # (the speeds: a random walk, measured right after with R = I)
        pv, pw = self.pv + 1.0, self.pw + 1.0
        kv, kw = pv / (pv + 1.0), pw / (pw + 1.0)
        self.vx += kv * (v - self.vx)
        self.vr += kw * (w - self.vr)
        self.pv, self.pw = pv * (1.0 - kv), pw * (1.0 - kw)

    def _update(self, ph: Sequence[Tuple[float, float]], s: Tuple[float, float, float], e: Tuple[float, float]):
        """x += K e, P -= K H P with K = P H' S^-1; ph: P H' (3 rows), s: S (s00, s01, s11)."""
        det = s[0] * s[2] - s[1] * s[1]
        i00, i01, i11 = s[2] / det, -s[1] / det, s[0] / det
        k = [(r0 * i00 + r1 * i01, r0 * i01 + r1 * i11) for r0, r1 in ph]
        self.x += k[0][0] * e[0] + k[0][1] * e[1]
        self.y += k[1][0] * e[0] + k[1][1] * e[1]
        self.th += k[2][0] * e[0] + k[2][1] * e[1]
        # (H P = (P H')')
        kh = [[ki[0] * ph[j][0] + ki[1] * ph[j][1] for j in range(3)] for ki in k]
        p00, p01, p02, p11, p12, p22 = self.p
        self.p = [p00 - kh[0][0], p01 - kh[0][1], p02 - kh[0][2], p11 - kh[1][1], p12 - kh[1][2], p22 - kh[2][2]]

    def position(self, gx: float, gy: float, r: float) -> None:
        c, s = math.cos(self.th), math.sin(self.th)
        hx = self.x + c * self.ox - s * self.oy
        hy = self.y + s * self.ox + c * self.oy
        p00, p01, p02, p11, p12, p22 = self.p
        self._update([(p00, p01), (p01, p11), (p02, p12)], (p00 + r, p01, p11 + r), (gx - hx, gy - hy))

    def motion_vector(self, mx: float, my: float, r: float = R_MOTION) -> None:
        th, v, w, ox, oy = self.th, self.vx, self.vr, self.ox, self.oy
        c, s = math.cos(th), math.sin(th)
        hx = v * c - s * ox * w - c * oy * w
        hy = v * s + c * ox * w - s * oy * w
        h0 = -v * s - ox * w * c + s * oy * w
        h1 = v * c - ox * w * s - c * oy * w
        _, _, p02, _, p12, p22 = self.p
        self._update([(p02 * h0, p02 * h1), (p12 * h0, p12 * h1), (p22 * h0, p22 * h1)],
                     (h0 * h0 * p22 + r, h0 * h1 * p22, h1 * h1 * p22 + r), (mx - hx, my - hy))


class _History:
    """xbot_monitoring's PositionHistory: the points of the pose (a median every HISTORY_RATE s) thinned in windows,
    a new segment at every change of state or blade, as the lines of a position_history file."""

    def __init__(self):
        self.job = self.session = ''
        self.blades = False
        self.buf: List[Tuple[float, float]] = []
        self.segments: List[Dict[str, Any]] = []

    def add(self, x: float, y: float, now: float) -> None:
        if not self.session:
            return
        self.buf.append((x, y))
        if len(self.buf) > HISTORY_WINDOW:
            self._compact(False, now)

    def event(self, event: Mapping[str, Any], now: float) -> None:
        job_changed = changed = False
        if event['type'] == 'STATE':
            job, session = event.get('job_id', ''), event.get('session_id', '')
            job_changed = job != self.job
            changed = job_changed or session != self.session
            self.job, self.session = job, session
        elif event['type'] == 'BLADES':
            changed = bool(event['enabled']) != self.blades
            self.blades = bool(event['enabled'])
        if job_changed:
            self._compact(True, now)
            if self.job:
                self.segments, self.buf = [], []
                self._segment(now)
        elif changed:
            self._compact(True, now)
            self._segment(now)

    def lines(self) -> List[Dict[str, Any]]:
        self._compact(True, None)
        return [{'type': 'new_segment', 'attributes': s['attributes'], 'started_at': s['started_at'],
                 'points': [[round(x, 4), round(y, 4)] for x, y in s['points']]} for s in self.segments]

    def _segment(self, now: float) -> None:
        if not self.job:
            return
        bridge = [self.segments[-1]['points'][-1]] if self.segments and self.segments[-1]['points'] else []
        self.segments.append({'attributes': {'job_id': self.job, 'session_id': self.session, 'blades': self.blades},
                              'started_at': now, 'points': bridge})

    def _compact(self, flush: bool, now: Optional[float]) -> None:
        if len(self.buf) <= HISTORY_CONTEXT and not flush:
            return
        n = len(self.buf) if flush else len(self.buf) - HISTORY_CONTEXT
        window = [(*self.segments[-1]['points'][-1], False)] if self.segments and self.segments[-1]['points'] else []
        window += [(x, y, i < n) for i, (x, y) in enumerate(self.buf)]
        kept = [(x, y) for x, y, c in _rdp(_decimate(window)) if c] if window else []
        if kept:
            if not self.segments:
                self.segments.append({'attributes': {'job_id': self.job, 'session_id': self.session,
                                                     'blades': self.blades}, 'started_at': now, 'points': []})
            self.segments[-1]['points'] += kept
        del self.buf[:n]


def _decimate(pts: Sequence[Tuple[float, float, bool]]) -> List[Tuple[float, float, bool]]:
    out, last_h, since = [pts[0]], math.nan, 0
    for cur in pts[1:]:
        last = out[-1]
        since += 1
        dx, dy = cur[0] - last[0], cur[1] - last[1]
        d2 = dx * dx + dy * dy
        h, turned = last_h, False
        if d2 > HISTORY_NOISE ** 2:
            h = math.atan2(dy, dx)
            if not math.isnan(last_h):
                d = math.fmod(abs(last_h - h), 2.0 * math.pi)
                turned = (2.0 * math.pi - d if d > math.pi else d) > HISTORY_HEADING
        if d2 > HISTORY_DIST ** 2 or turned or since >= HISTORY_BEAT:
            out.append(cur)
            last_h, since = h, 0
    return out


def _rdp(pts: Sequence[Tuple[float, float, bool]]) -> List[Tuple[float, float, bool]]:
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        s, e = stack.pop()
        if e <= s + 1:
            continue
        (ax, ay, _), (bx, by, _) = pts[s], pts[e]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        best, k = 0.0, s
        for i in range(s + 1, e):
            px, py = pts[i][0] - ax, pts[i][1] - ay
            t = 0.0 if l2 == 0 else max(0.0, min(1.0, (px * dx + py * dy) / l2))
            d = (px - t * dx) ** 2 + (py - t * dy) ** 2
            if d > best:
                best, k = d, i
        if best > HISTORY_RDP ** 2:
            keep[k] = True
            stack += [(s, k), (k, e)]
    return [p for p, f in zip(pts, keep) if f]


class _World:
    """The mower, what it knows of itself, mower_logic's events and what's recorded, stepped through time."""

    def __init__(self, plan: CoveragePlan, m: MowerModel, start: Pose2D, t0: float):
        self.plan, self.m, self.t0 = plan, m, t0
        self.rnd = random.Random(m.seed)
        self.res = SimResult(model=m, t0=t0)
        self.t = 0.0
        self.x, self.y, self.yaw = start.x, start.y, start.yaw
        self.cmd = (0.0, 0.0)
        self.queue: Deque[Tuple[float, Tuple[float, float]]] = collections.deque()
        self.drive_cmd = (0.0, 0.0)
        self.vl = self.vr = 0.0
        gain = m.drive_gain * (1.0 + self.rnd.gauss(0.0, m.drive_gain_sigma)) if m.drive_gain_sigma else m.drive_gain
        asym = self.rnd.gauss(0.0, m.drive_asymmetry_sigma) if m.drive_asymmetry_sigma else 0.0
        self.gain_l, self.gain_r = gain * (1.0 - 0.5 * asym), gain * (1.0 + 0.5 * asym)
        self.blade_cmd = self.blade = False
        self.mode, self.index = APPROACH, -1
        self.ftc = _Ftc(m)
        self.progress: Optional[Callable[[float], None]] = None
        self.history = _History()
        self.pose_buf: List[Tuple[float, float]] = []
        self.next_history = HISTORY_RATE
        ids = random.Random(m.seed * 7919 + 1)
        alphabet = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'
        self.job_id = ''.join(ids.choice(alphabet) for _ in range(32))
        self.session_id = ''.join(ids.choice(alphabet) for _ in range(32))
        if m.positioning:
            self.dt = 1.0 / IMU_RATE
            self.ekf = _Ekf(m, start.x, start.y, start.yaw)
            self.gyro_bias = self.rnd.gauss(0.0, m.gyro_bias_sigma) if m.gyro_bias_sigma else 0.0
            self.gyro_scale = 1.0 + (self.rnd.gauss(0.0, m.gyro_scale_sigma) if m.gyro_scale_sigma else 0.0)
            self.odometry = 0.0
            self.next_odometry = 0.0
            self.next_gps = 0.0
            self.next_mv = 0.0
            self.epoch = 0
            self.gps_white = self.rnd.gauss(0.0, m.gps_sigma) if m.gps_sigma else 0.0
            self.gps_white_y = self.rnd.gauss(0.0, m.gps_sigma) if m.gps_sigma else 0.0
            self.drift = (self.rnd.gauss(0.0, m.gps_drift), self.rnd.gauss(0.0, m.gps_drift)) if m.gps_drift \
                else (0.0, 0.0)
            self.jump_until, self.jump = -1.0, (0.0, 0.0)
            self.mv = (0.0, 0.0)
            # (where the mower was: the GNSS is late)
            self.past: Deque[Tuple[float, float, float, float, float, float]] = collections.deque()
            self.deliveries: Deque[Tuple[float, float, float]] = collections.deque()
        else:
            self.dt = 1.0 / m.controller_frequency
            self.ekf = None
        self._sample()

    # -- where it thinks it is
    def estimate(self) -> Tuple[float, float, float]:
        if self.ekf is None:
            return self.x, self.y, self.yaw
        return self.ekf.x, self.ekf.y, self.ekf.th

    # -- events
    def emit(self, kind: str, **details) -> None:
        ex, ey, _ = self.estimate()
        event = dict({'t': round(self.t0 + self.t, 3), 'type': kind}, **details)
        event.update(x=round(ex, 4), y=round(ey, 4))
        if kind not in ('BOOTED', 'SHUTDOWN'):
            event.update(job_id=self.job_id, session_id=self.session_id)
        if kind == 'STATE' and details.get('state') == 'IDLE':
            event['session_id'] = ''
        self.res.events.append(event)
        self.history.event(event, self.t0 + self.t)

    def set_blade(self, on: bool) -> None:
        """mower_logic's blade: set at the next SAFETY_TICK (BLADES then)."""
        if on == self.blade_cmd:
            return
        self.blade_cmd = on
        tick = math.ceil(self.t / SAFETY_TICK - 1e-9) * SAFETY_TICK
        self.run_until(tick)
        self.blade = on
        self.emit('BLADES', enabled=on)

    # -- time
    def run_until(self, t_end: float) -> None:
        """The mower going on with the command it has until t_end."""
        while self.t < t_end - 1e-9:
            dt = min(self.dt, t_end - self.t)
            self._step(dt)

    def _step(self, dt: float) -> None:
        m = self.m
        t = self.t + dt
        # the drive: the command m.drive_delay late, the wheels getting to its speeds
        if m.drive_delay > 0:
            self.queue.append((self.t, self.cmd))
            while self.queue and self.queue[0][0] <= t - m.drive_delay:
                self.drive_cmd = self.queue.popleft()[1]
        else:
            self.drive_cmd = self.cmd
        v, w = self.drive_cmd
        half = 0.5 * m.wheel_distance * w
        tl, tr = self.gain_l * _clamp(v - half, 1.0), self.gain_r * _clamp(v + half, 1.0)
        if m.drive_tau > 0:
            a = 1.0 - math.exp(-dt / m.drive_tau)
            tl, tr = self.vl + (tl - self.vl) * a, self.vr + (tr - self.vr) * a
        if m.drive_accel > 0:
            step = m.drive_accel * dt
            tl = self.vl + _clamp(tl - self.vl, step)
            tr = self.vr + _clamp(tr - self.vr, step)
        self.vl, self.vr = tl, tr
        vg = 0.5 * (tl + tr) * (1.0 - m.slip)
        wg = (tr - tl) / m.wheel_distance * (1.0 - m.slip_turn)
        # (an arc)
        yaw = self.yaw + wg * dt
        c, s = math.cos(yaw), math.sin(yaw)
        if abs(wg) > 1e-9:
            r = vg / wg
            self.x += r * (s - math.sin(self.yaw))
            self.y -= r * (c - math.cos(self.yaw))
        else:
            self.x += vg * dt * c
            self.y += vg * dt * s
        self.yaw = _wrap(yaw)
        self.t = t
        if self.ekf is not None:
            self._sense(vg, wg, 0.5 * (tl + tr), c, s)
        else:
            self._pose_out()

    def _sense(self, vg: float, wg: float, v_wheels: float, c: float, s: float) -> None:
        """The sensors and xbot_positioning, one IMU message (c, s: of the mower's heading)."""
        m, ekf, t = self.m, self.ekf, self.t
        ox, oy = m.antenna_offset_x, m.antenna_offset_y
        # the antenna, where and how fast (kept for the GNSS that's late)
        ax, ay = self.x + c * ox - s * oy, self.y + s * ox + c * oy
        avx, avy = vg * c - wg * (s * ox + c * oy), vg * s + wg * (c * ox - s * oy)
        self.past.append((t, ax, ay, avx, avy, self.yaw))
        while len(self.past) > 2 and self.past[1][0] <= t - m.gps_latency - 1e-9:
            self.past.popleft()
        if t >= self.next_odometry - 1e-9:
            self.next_odometry += 1.0 / m.odometry_rate
            odo = m.odometry_scale * v_wheels + (self.rnd.gauss(0.0, m.odometry_sigma) if m.odometry_sigma else 0.0)
            self.odometry = round(odo / ODOMETRY_STEP) * ODOMETRY_STEP if m.quantize else odo
        gyro = self.gyro_scale * wg + self.gyro_bias + (self.rnd.gauss(0.0, m.gyro_sigma) if m.gyro_sigma else 0.0)
        if m.quantize:
            gyro = round(gyro / GYRO_STEP) * GYRO_STEP
        # (the bias calibrated away at the start: what's left of it is gyro_bias)
        ekf.predict(self.odometry, gyro, self.dt)
        if t >= self.next_gps - 1e-9:
            self.next_gps += 1.0 / m.gps_rate
            self._gps_epoch(t)
        while self.deliveries and self.deliveries[0][0] <= t + 1e-9:
            _, gx, gy = self.deliveries.popleft()
            ekf.position(gx, gy, R_POSITION)
            if math.hypot(*self.mv) >= MIN_SPEED:
                ekf.motion_vector(*self.mv)
        self._pose_out()

    def _gps_epoch(self, t: float) -> None:
        m = self.m
        _, ax, ay, avx, avy, _ = self.past[0]  # (where the antenna was gps_latency ago)
        if m.gps_sigma:
            if m.gps_tau > 0:
                phi = math.exp(-1.0 / (m.gps_rate * m.gps_tau))
                k = m.gps_sigma * math.sqrt(1.0 - phi * phi)
                self.gps_white = phi * self.gps_white + self.rnd.gauss(0.0, k)
                self.gps_white_y = phi * self.gps_white_y + self.rnd.gauss(0.0, k)
            else:
                self.gps_white, self.gps_white_y = self.rnd.gauss(0.0, m.gps_sigma), self.rnd.gauss(0.0, m.gps_sigma)
        if m.gps_drift:
            phi = math.exp(-1.0 / (m.gps_rate * m.gps_drift_tau))
            k = m.gps_drift * math.sqrt(1.0 - phi * phi)
            self.drift = (phi * self.drift[0] + self.rnd.gauss(0.0, k), phi * self.drift[1] + self.rnd.gauss(0.0, k))
        if m.gps_jumps and t > self.jump_until and self.rnd.random() < m.gps_jumps / 3600.0 / m.gps_rate:
            a = self.rnd.uniform(-math.pi, math.pi)
            self.jump_until, self.jump = t + m.gps_jump_time, (m.gps_jump * math.cos(a), m.gps_jump * math.sin(a))
        jx, jy = self.jump if t <= self.jump_until else (0.0, 0.0)
        wx, wy = (self.rnd.gauss(0.0, m.gps_white), self.rnd.gauss(0.0, m.gps_white)) if m.gps_white else (0.0, 0.0)
        gx = ax + self.gps_white + self.drift[0] + jx + wx
        gy = ay + self.gps_white_y + self.drift[1] + jy + wy
        if t >= self.next_mv - 1e-9:
            self.next_mv += 1.0 / m.motion_vector_rate
            sv = m.motion_vector_sigma
            self.mv = (avx + self.rnd.gauss(0.0, sv), avy + self.rnd.gauss(0.0, sv)) if sv else (avx, avy)
        if m.gps_accuracy <= MAX_GPS_ACCURACY:
            self.deliveries.append((t, gx, gy))
            self.epoch += 1
            if m.gps_burst and self.epoch % max(1, int(round(m.gps_rate))) == 0:
                n = int(m.gps_burst) + (1 if self.rnd.random() < m.gps_burst - int(m.gps_burst) else 0)
                self.deliveries.extend([(t, gx, gy)] * n)
                self.deliveries.append((t + m.gps_late, gx, gy))

    def _pose_out(self) -> None:
        """A pose published (xb_pose): into the position history's median."""
        ex, ey, _ = self.estimate()
        self.pose_buf.append((ex, ey))
        if self.t >= self.next_history - 1e-9:
            self.next_history += HISTORY_RATE
            k = len(self.pose_buf) // 2
            self.history.add(sorted(p[0] for p in self.pose_buf)[k], sorted(p[1] for p in self.pose_buf)[k],
                             self.t0 + self.t)
            self.pose_buf = []

    def _sample(self) -> None:
        ex, ey, eyaw = self.estimate()
        self.res.samples.append((round(self.t, 3), self.x, self.y, self.yaw, ex, ey, eyaw, self.blade, self.mode,
                                 self.index))

    # -- Move Base Flex
    def follow(self, poses: Sequence[Tuple[float, float, float]], mode: str, offset: int = -1) -> Tuple[bool, int]:
        """exe_path with FTC (the one controller of move_base and exe_path: what it had of the plan before stays):
        (succeeded, how far its point got: an index into poses)."""
        m, ftc = self.m, self.ftc
        ftc.set_plan(poses, self.t)
        start, last_valid = self.t, None
        tick, k = self.t + FIRST_DT, 0
        self.mode = mode
        while True:
            self.run_until(tick)
            self.index = offset + ftc.idx if offset >= 0 else -1
            self._sample()
            if ftc.reached():
                return True, ftc.idx
            v, w, ok = ftc.compute(self.estimate(), self.t)
            if ok:
                self.cmd, last_valid = (v, w), self.t
            else:
                self.cmd = (0.0, 0.0)
                if self.t - start > m.controller_patience and \
                        (last_valid is None or self.t - last_valid > m.controller_patience):
                    return False, ftc.idx
            k += 1
            tick = start + k / m.controller_frequency
            if self.progress is not None and k % PROGRESS_TICKS == 0:
                self.progress(self.t)

    def move_base(self, goal: Pose2D) -> bool:
        """mower_logic's way to a path's first pose: GlobalPlanner's plan on the costmap (here straight: from where the
        mower is, cells apart, each pose facing the next, the goal's place facing that way too and then the goal
        itself; 3 poses for a goal a few cm off) driven by FTC."""
        ex, ey, _ = self.estimate()
        d = math.hypot(goal.x - ex, goal.y - ey)
        h = math.atan2(goal.y - ey, goal.x - ex) if d > 1e-6 else goal.yaw
        n = max(1, int(math.ceil(d / COSTMAP_CELL)))
        poses = [(ex + (goal.x - ex) * i / n, ey + (goal.y - ey) * i / n, h) for i in range(n + 1)]
        poses.append((goal.x, goal.y, goal.yaw))
        sent = self.t
        ok, _ = self.follow(poses, APPROACH)
        self.cmd = (0.0, 0.0)
        self.noticed(sent)
        return ok

    def noticed(self, sent: float) -> None:
        """mower_logic looks at the goal goal_sleep after sending it, then every POLL s."""
        first = sent + self.m.goal_sleep
        at = first if self.t <= first else first + math.ceil((self.t - first) / POLL - 1e-9) * POLL
        self.run_until(at)

    def recover(self) -> None:
        """BackwardForwardRecovery: back RECOVERY_DISTANCE (forward where it can't), each way RECOVERY_TIME at most."""
        self.mode = RECOVERY
        space = self.plan.transit_space if self.plan.transit_space is not None else self.plan.free_space
        space = prep(space.buffer(0.05)) if space is not None else None
        for sign in (-1.0, 1.0):
            ex0, ey0, _ = self.estimate()
            start = self.t
            while self.t - start < RECOVERY_TIME:
                ex, ey, eyaw = self.estimate()
                if math.hypot(ex - ex0, ey - ey0) >= RECOVERY_DISTANCE:
                    break
                ahead = Point(ex + sign * RECOVERY_CHECK * math.cos(eyaw), ey + sign * RECOVERY_CHECK * math.sin(eyaw))
                if space is None or not space.contains(ahead):
                    break
                self.cmd = (sign * RECOVERY_SPEED, 0.0)
                self.run_until(self.t + 0.1)
                self._sample()
            self.cmd = (0.0, 0.0)
            ex, ey, _ = self.estimate()
            if math.hypot(ex - ex0, ey - ey0) >= RECOVERY_DISTANCE:
                break
        self.run_until(self.t + 0.2)


def simulate(plan: CoveragePlan, model: Optional[MowerModel] = None, start: Optional[Pose2D] = None,
             t0: float = 0.0, progress: Optional[Callable[[float], None]] = None) -> SimResult:
    """The mower mowing plan like OpenMower does, from start (else the plan's first pose): mower_logic's paths
    (export.plan_pieces) one after the other, each driven to its first pose (move_base), the blade spun up when it's
    off, then followed (exe_path); where FTC gives up, a navigation error: the blade off, backing up, on again from
    where its point got to. t0: s of the unix epoch at the start (for the events and the track). progress: called with
    how far it got (0..1, by the time simulated against drive_time's guess) every PROGRESS_TICKS controller cycles, an
    exception it raises ends the simulation."""
    m = model or MowerModel()
    m.validate()
    pieces = plan_pieces(plan)
    if not pieces:
        raise PlanningError('nothing to simulate: the plan has no path')
    first = plan.poses[pieces[0][1]]
    w = _World(plan, m, start or first, t0)
    if progress is not None:
        expected = drive_time(plan.poses)['s'] + PIECE_SECONDS * len(pieces) + m.spinup
        w.progress = lambda t: progress(min(0.99, t / expected))
    if m.positioning:  # (standing a bit: the EKF settled where it is)
        w.run_until(2.0)
    w.emit('STATE', state='MOWING')
    w.run_until(w.t + 1.0)
    w.emit('AREA', area_id='', area_name='')
    for _, a, b in pieces:
        poses = [(p.x, p.y, p.yaw) for p in plan.poses[a:b + 1]]
        idx, tries, trims, errors = 0, 0, 0, 0
        while True:
            if not w.move_base(Pose2D(*poses[idx])):
                tries += 1
                if tries < FIRST_TRIES:
                    continue
                if trims < TRIMS and idx + 1 < len(poses):
                    idx, trims, tries = idx + 1, trims + 1, 0
                    continue
                w.res.aborted = True
                break
            tries = trims = 0
            if not w.blade_cmd:
                w.set_blade(True)
                w.cmd = (0.0, 0.0)
                w.run_until(w.t + m.spinup)
            sent = w.t
            _, got = w.follow(poses[idx:], PATH, a + idx)
            w.cmd = (0.0, 0.0)
            w.noticed(sent)
            idx += got
            if len(poses) - idx < PATH_DONE:
                break
            # a navigation error: on from where FTC's point got to
            idx = max(idx, 1)
            errors += 1
            w.res.nav_errors.append((round(w.t, 3), a + idx))
            w.set_blade(False)
            w.recover()
            w.emit('NAVIGATION_ERROR')
            if errors >= MAX_NAV_ERRORS:
                w.res.aborted = True
                break
        if w.res.aborted:
            break
    w.set_blade(False)
    w.cmd = (0.0, 0.0)
    w.run_until(w.t + 0.5)
    w.emit('JOB_COMPLETE')
    w.emit('STATE', state='IDLE')
    w._sample()
    w.res.history = w.history.lines()
    w.res.duration = round(w.t, 3)
    return w.res


def _runs(samples: Sequence[Tuple], blade_only: bool) -> List[List[Tuple]]:
    """The samples in runs: where the blade mowed (blade_only), else of each piece of the way."""
    runs: List[List[Tuple]] = []
    last = None
    for s in samples:
        on = s[7] if blade_only else True
        if not on:
            last = None
            continue
        if last is None:
            runs.append([])
        runs[-1].append(s)
        last = s
    return runs


def mowed_area(plan: CoveragePlan, samples: Sequence[Tuple]):
    """What the blade mowed where the mower really was (samples with the blade on), in plan.free_space."""
    cfg = plan.config
    ahead, off, width = cfg.blade_ahead, cfg.blade_offset, cfg.mower_width
    parts = []
    for run in _runs(samples, True):
        pts = []
        for s in run:
            x, y, yaw = s[1], s[2], s[3]
            q = (x + ahead * math.cos(yaw) - off * math.sin(yaw), y + ahead * math.sin(yaw) + off * math.cos(yaw))
            if not pts or math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 1e-4:
                pts.append(q)
        if len(pts) == 1:
            parts.append(Point(pts[0]).buffer(0.5 * width, 4))
        # (in pieces sharing their ends: the same union, GEOS a lot quicker)
        for i in range(0, len(pts) - 1, BLADE_PIECE):
            parts.append(LineString(pts[i:i + BLADE_PIECE + 1]).buffer(0.5 * width, 4))
    return union(parts).intersection(plan.free_space)


def body_places(plan: CoveragePlan, samples: Sequence[Tuple], modes: Sequence[str] = (PATH, RECOVERY)) \
        -> Tuple[List[List[float]], int]:
    """Where the body got out of plan.body_space (the edges less their margins) where the mower really was, looked at
    every BODY_STEP m or BODY_TURN rad: ([[x, y, m], ...] the farthest out within PLACE of each other, farthest first;
    how many looks found it out). Nothing without a body outline."""
    cfg = plan.config
    if cfg is None or not cfg.body or plan.body_space is None:
        return [], 0
    fp = Footprint(plan.body_space, cfg.body_outline)
    looked, last = [], None
    for s in samples:
        if s[8] not in modes:
            last = None
            continue
        if last is None or math.hypot(s[1] - last[1], s[2] - last[2]) >= BODY_STEP or \
                abs(_wrap(s[3] - last[3])) >= BODY_TURN:
            looked.append(Pose2D(s[1], s[2], s[3]))
            last = s
    bad = [p for p, ok in zip(looked, fp.fit(looked)) if not ok] if looked else []
    found = []
    for i in range(0, len(bad), 200):
        part = bad[i:i + 200]
        found += [(d, xy) for d, xy in fp.out_by([(None, p) for p in part]) if xy is not None and d > 0]
    places: List[List[float]] = []
    for d, (x, y) in sorted(found, key=lambda f: -f[0]):
        if all(math.hypot(x - px, y - py) > PLACE for px, py, _ in places):
            places.append([round(x, 3), round(y, 3), round(d, 3)])
    return places, len(found)


def tracking(plan: CoveragePlan, samples: Sequence[Tuple]) -> Dict[str, Dict[str, float]]:
    """How far the mower really was off the plan's way while following it, by the kind of segment its controller was
    on: {kind: {'p50', 'p95', 'max'}} in m, and 'estimate': how far off it thought it was."""
    kinds = {}
    for seg in plan.segments:
        for i in range(seg.start, seg.end):
            kinds[i] = seg.kind
    lines, kind_of = [], []
    for i in range(len(plan.poses) - 1):
        a, b = plan.poses[i], plan.poses[i + 1]
        if (a.x, a.y) != (b.x, b.y):
            lines.append(LineString([(a.x, a.y), (b.x, b.y)]))
            kind_of.append(kinds.get(i, TRANSIT))
    out: Dict[str, Dict[str, float]] = {}
    follow = [s for s in samples if s[8] == PATH][::TRACK_EVERY]
    if not lines or not follow:
        return out
    tree = STRtree(lines)
    by: Dict[str, List[float]] = collections.defaultdict(list)
    points = [Point(s[1], s[2]) for s in follow]
    if hasattr(tree, 'query_nearest'):  # (shapely 2: all at once)
        (i, j), d = tree.query_nearest(points, return_distance=True, all_matches=False)
        for k, dist in zip(j.tolist(), d.tolist()):
            by[kind_of[k]].append(dist)
    else:
        for q in points:
            k = int(tree.nearest(q))
            by[kind_of[k]].append(lines[k].distance(q))
    est = [math.hypot(s[1] - s[4], s[2] - s[5]) for s in follow]
    by['estimate'] = est
    for kind, ds in by.items():
        ds = sorted(ds)
        out[kind] = {'p50': round(ds[len(ds) // 2], 4), 'p95': round(ds[int(0.95 * (len(ds) - 1))], 4),
                     'max': round(ds[-1], 4)}
    return out


def spins(samples: Sequence[Tuple]) -> int:
    """How often the mower turned on the spot (SPIN_RATE at SPIN_SPEED at most for SPIN_TIME s at least)."""
    count, since = 0, None
    for a, b in zip(samples, samples[1:]):
        dt = b[0] - a[0]
        if dt <= 0:
            continue
        v = math.hypot(b[1] - a[1], b[2] - a[2]) / dt
        w = abs(_wrap(b[3] - a[3])) / dt
        if w >= SPIN_RATE and v <= SPIN_SPEED:
            if since is None:
                since = a[0]
            elif b[0] - since >= SPIN_TIME and since >= 0:
                count += 1
                since = -1.0  # (counted)
        else:
            since = None
    return count


def evaluate(plan: CoveragePlan, res: SimResult, progress: Optional[Callable[[float], None]] = None) \
        -> Dict[str, Any]:
    """What the simulated mow did: {'s' (how long), 'mow_s' (the blade on), 'model_s' (report.drive_time's guess),
    'driven_m', 'mowed_m2', 'left_m2', 'left_pieces', 'left' (like check_plan, where the blade really went),
    'planned_left_m2' (the plan driven exactly), 'body': {'places', 'looks', 'worst'} (body_places),
    'nav_errors', 'aborted', 'tracking' (tracking), 'spins'}. progress: called with how far it got (0..1) after each
    part, an exception it raises ends it."""
    def done(fraction: float) -> None:
        if progress is not None:
            progress(fraction)

    samples = res.samples
    driven = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:]))
    mow_s = sum(b[0] - a[0] for a, b in zip(samples, samples[1:]) if a[7] and b[7])
    out: Dict[str, Any] = {'s': round(res.duration, 1), 'mow_s': round(mow_s, 1),
                           'model_s': round(drive_time(plan.poses)['s'], 1), 'driven_m': round(driven, 2)}
    cfg = plan.config
    if cfg is not None and cfg.mower_width > 0:
        _, _, got = left_of(plan, mowed_area(plan, samples))
        out.update(got)
        done(0.3)
        _, _, planned = left_of(plan, swept_area(plan, cfg.mower_width).intersection(plan.free_space))
        out['planned_left_m2'] = planned['left_m2']
        done(0.45)
    places, looks = body_places(plan, samples)
    out['body'] = {'places': places[:50], 'looks': looks, 'worst': places[0][2] if places else 0.0}
    done(0.7)
    out['nav_errors'] = len(res.nav_errors)
    out['aborted'] = res.aborted
    out['tracking'] = tracking(plan, samples)
    out['spins'] = spins(samples)
    done(1.0)
    return out


# (the kinds, for those importing them from here)
KINDS = (PERIMETER, LANE, TURN, TRANSIT)
