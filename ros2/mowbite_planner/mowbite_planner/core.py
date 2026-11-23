"""The MowBite Planner's node without the robot: PlannerCore holds its settings (for all areas, set over RPC and kept in
a file), the planner.* RPC methods every robot has (planner.settings, .settings.set, .angle.reset, .log, .area.set,
.plan, .leftovers, .drive), the plans kept and planned ahead in one worker with their progress, and what follows a
finished mow (angle_increment, perimeter_direction random, planner.drive, the timing, the leftovers). What each method
answers: see the ROS 1 node's top (ros1/mowbite_planner_ros1/nodes/mowbite_planner_node), OpenMower's adapter.

An adapter is a class of PlannerCore that connects it to a robot, Python 3.8 or later:
  transport     it hands the RPC requests to call(self.methods, method, params, ...) (methods: name -> callback, its
                own added) and sends the answer or the error back; asked_id() is set from REQUEST.id for the worker's
                waiting jobs. Codes: INVALID_PARAMS (a PlanningError: the caller's mistake; its code where it has one),
                INTERNAL, SUPERSEDED, METHOD_NOT_FOUND; call's data: a PlanningError's word for the error's data
  publish       publish(topic, payload, retain): the progress (PROGRESS_TOPIC) out to whoever shows it
  map           set_map(text) with every map (OpenMower's JSON), replace_map(data) writes one back
                (planner.area.set), map_unusable(): why the map can't be planned now ('' when it can), plan_map(data):
                the map as the plans take it, area_setting(props, own, name, value): an area's setting into the map
  parameters    param(name, default): the robot's own (mower_logic's /mower_logic, the controller's FTC_PARAMS)
  events        on_event(payload) with every event of the mower's logic (STATE, JOB_COMPLETE, JOB_RESET)
  arrives       arrives(boundary, obstacles, cfg, drivable, keep_out): for a plan of those, whether the robot gets to
                a pose facing its heading from where it is (plan_coverage's arrives(pose, came_from), starts.py),
                None: it drives what it's given;
                arrives.key (JSON, else True) is part of the plans' keys; kept_plan's from_start: arrives.start (x, y,
                yaw) the robot gets to the first section from (starts.py)
  storage       the files it keeps: settings_file (next to it the angle steps, timing, planning times and the maps
                before), plans_dir, history_dir (the robot's track, position_history)
  capabilities  cannot: {setting: why} the robot can't have (shown ignored, not settable, the default counts; a planner
                setting: an area's and a request's own neither),
                given_by: {setting: where from} the robot gives itself, given(): their values (not settable, not
                stored, theirs count; given_again() once they change),
                drive_ignored(): why drive slope and reverse are planned forwards ('' where they aren't),
                back_out_ignored(): why back_out is off ('' where it isn't: the robot backs up where a plan does;
                by default drive_ignored's),
                recorded_loop_ignored(recorded): why recorded_loop is off for an area ('' where it isn't; recorded:
                the map has the body it was recorded with),
                classic_plan (planner slic3r), remote_plan (remote_planning), remote_state, mower_ready, held and
                start_mowing (leftovers_auto),
                checked(plan): the robot's own last check of a plan just planned, a PlanningError where it doesn't
                take it (not kept as a plan then, the no kept like any other)
  logs          loginfo, logwarn, logerr, logwarn_throttle: to the robot's log, the lines go to LOG (planner.log)
"""

import collections
import dataclasses
import glob
import hashlib
import itertools
import json
import logging
import math
import os
import pickle
import shutil
import subprocess
import sys
import threading
import time
import weakref
from concurrent.futures import Future

from shapely.geometry import Point as Spot

import mowbite_planner
from . import backing
from . import config as limits
from .config import IGNORED, NothingLeft, PlannerConfig, PlanningError
from .footprint import Footprint
from .geometry import clean_polygon, polygons, union
from .mapfile import _areas, _points, _props, area_settings, body_known, match_area, mow_areas, openmower_settings
from .mathutil import Pose2D
from .mqtt_service import PlannerService
from .planner import fill_field, plan_coverage, plan_footprint, set_workers
from .progress import Stopped, stoppable
from .remote import canon, code_version, plan_key, request_json, stack
from .report import drive_time, track_mowed

# JSON-RPC's codes: the caller's mistake (a PlanningError), the planner's, a method no node answers
INVALID_PARAMS = -32602
INTERNAL = -32603
METHOD_NOT_FOUND = -32601

# set by every request of mower_logic (from its parameters), a value here wouldn't count (skip_lanes is a setting, a
# request's skip_fill skips the lanes too). FTC drives forwards only
FIXED = {'skip_outline_loops': 'mower_logic', 'skip_obstacle_loops': 'mower_logic',
         'angle_steps': 'finished mows (planner.angle.reset)', 'finished_mows': 'finished mows'}
# come with every request of mower_logic (from its parameters, or the area's), unless set here (settings, stored)
MOWER_LOGIC = {'overlap': '/mower_logic/tool_width', 'perimeter_offset': '/mower_logic/outline_offset',
               'perimeter_passes': '/mower_logic/outline_count',
               'lane_overlap_passes': '/mower_logic/outline_overlap_count'}
CHOICES = {'fill_pattern': limits.FILL_PATTERNS, 'perimeter_order': limits.PERIMETER_ORDERS,
           'perimeter_direction': limits.PERIMETER_DIRECTIONS, 'angle_strategy': limits.ANGLE_STRATEGIES,
           'lane_order': limits.LANE_ORDERS, 'turn_types': limits.TURN_TYPES, 'route_order': limits.ROUTE_ORDERS,
           'edges': limits.EDGES, 'narrow_parts': limits.NARROW_PARTS,
           'lane_spacing_mode': limits.LANE_SPACING_MODES, 'headland_corners': limits.HEADLAND_CORNERS,
           'drive': limits.DRIVES}
# the value that means "the planner works it out"
AUTO_VALUES = {'perimeter_passes': -1, 'obstacle_passes': -1}
RAD = ('angle', 'angle_offset', 'angle_min', 'angle_max', 'angle_step', 'crosshatch_angle', 'angle_increment')
# may be negative (perimeter_offset: outside the outline, blade_offset: right of the centre)
SIGNED = RAD + ('perimeter_offset', 'blade_offset', 'blade_ahead')
# [[x, y], ...] in m: the body's outline round base_link (x ahead, y to the left)
POINTS = ('robot_outline',)
# which planner answers mower_logic, for all areas (a setting of the node, not of the MowBite Planner): slic3r is
# OpenMower's, started next to this node with its service moved to CLASSIC_SERVICE (see the launch file)
PLANNERS = ('mowbite', 'slic3r')
CLASSIC_SERVICE = '/slic3r_classic/plan_path'
CLASSIC_WAIT = 5.0  # s it waits for that service
# the node's own settings, kept with the planner's: which planner, leftovers_auto and remote_planning (see the top)
NODE_SETTINGS = ('planner', 'leftovers_auto', 'remote_planning')
# own: the ones of MOWER_LOGIC set here, they beat mower_logic's
Setup = collections.namedtuple('Setup', 'mower_width base own_angle stored own planner leftovers_auto '
                               'remote_planning')
RANGES = {'mower_width': (0.0, None), 'overlap': (0.0, 1.0), 'turn_radius': (0.0, limits.MAX_TURN_RADIUS),
          'angle_increment_every': (1, None),
          'perimeter_passes': (-1, limits.MAX_PASSES), 'obstacle_passes': (-1, limits.MAX_PASSES),
          'lane_overlap_passes': (0, limits.MAX_PASSES),
          'angle_step': (limits.MIN_ANGLE_STEP, None),
          'blade_offset': (-limits.MAX_BLADE_OFFSET, limits.MAX_BLADE_OFFSET),
          'blade_ahead': (-limits.MAX_BLADE_OFFSET, limits.MAX_BLADE_OFFSET),
          'edge_margin': (0.0, limits.MAX_BLADE_OFFSET),
          'obstacle_margin': (0.0, limits.MAX_BLADE_OFFSET), 'spin_margin': (0.0, limits.MAX_BLADE_OFFSET),
          'recorded_loop_margin': (0.0, limits.MAX_RECORDED_MARGIN)}
# plans kept, in memory and on disk (in plans_dir, <build>-<key>.pkl: the ones of another planner code or another
# Python, shapely, GEOS or numpy go at the start, none of them is asked for again), in both the ones last asked for:
# the same request again (a preview, then the mow; mowing.plan, then planner.plan; a mow going on) gets the plan at once
CACHED_PLANS = 16
# MB the plans in memory take at most, the job's too; a plan counts PLAN_ROOM times its pickle's bytes
CACHED_MB = 100.0
PLAN_ROOM = 4
# the planner's no to a request kept too (in memory): an area too small asked for again doesn't take as long again
REFUSED = 64
DISK_PLANS = 16
# in plans_dir: the keys of the plans of the job mower_logic mows (MowbitePlannerRos1.pin), the leftovers armed (kept
# over a restart, PlannerCore.keep_armed)
PINNED_FILE = 'job.json'
ARMED_FILE = 'armed.pickle'
# s after the map or the settings last changed, every active mowing area is planned ahead (and kept) as it is: the next
# mow and the app's preview find it
PLAN_AHEAD_DELAY = 5.0
# mower_logic's events (xbot_mqtt.publishEvent): a finished mow turns the lanes on (angle_increment)
EVENTS_TOPIC = 'events/json'
# OpenMower's track of every job (ROS_HOME/position_history/<start>_<job id>.jsonl): what planner.drive checks, a
# driven point whose body gets more than DRIVE_OFF m into the margins is a place, points closer than DRIVE_STEP m to the
# one before give no heading and are left out
HISTORY_DIR = 'position_history'
DRIVE_OFF = 0.01
DRIVE_STEP = 0.02
# m round a docking station the track isn't checked: docking it drives over the area's edge on purpose
DOCK_REACH = 1.0
# how long mowing a plan takes: report.drive_time with OpenMower's FTC speeds (move_base_flex/FTCPlanner), times what
# the last TIMING_RUNS finished mows took against that (from the start of the blade running to its end in the job's
# track; one plan for each area of the job, TIMING_RANGE of the estimate at most), kept next to settings_file
TIMING_RUNS = 10
# s planner.leftovers' arm lasts: mower_logic's plans in that time are the leftovers', until that job is finished
LEFTOVERS_ARMED = 900.0
# s it lasts once mower_logic got one of them, where the job they went to has no id (an id binds them to that job: its
# JOB_COMPLETE or JOB_RESET ends them, another job's STATE too; a new map or settings end them either way)
LEFTOVERS_JOB = 6 * 3600.0
# m2, an unmowed piece planner.leftovers counts (complete.LEAST_MOWED)
LEFT_PIECE = 0.02
# leftovers_auto: m2 the passes mow at least for the mower to go out again (under it the drive there isn't worth it),
# s between two looks whether it's ready (mower_ready)
LEFTOVERS_AUTO_MIN = 0.3
READY_POLL = 2.0
# s a finished mow's track is waited for before it's read: xbot_monitoring writes it every 30 s and the rest when the
# job ends (mower_logic's next STATE event of another job or none: idle), then TRACK_SETTLE s for that to be on disk
TRACK_WAIT = 900.0
TRACK_SETTLE = 2.0
# of an area: with more left than this, planner.leftovers plans nothing for it (that track wasn't a mow of it)
LEFT_MOST = 0.5
TIMING_RANGE = (0.5, 2.0)
# the controller's parameters (param) the estimate takes its speeds from (OpenMower's FTC: speed_fast, speed_slow,
# speed_angular)
FTC_PARAMS = '/move_base_flex/FTCPlanner'
# the planner's jobs by urgency (Worker): mower_logic waits for its plan, then what was asked for over RPC, then the
# checks after a mow, planning ahead last (the node's own and planner.plan's ahead); their names in the progress, the
# waiting jobs shown there at most; JSON-RPC's code for a request superseded by one with its slot (-32001: the map
# changed, -32002: the answer too big)
MOWER, ASKED, AFTER_MOW, AHEAD = range(4)
RANKS = ('mower', 'asked', 'after_mow', 'ahead')
WAITING_SHOWN = 20
SUPERSEDED = -32003
# mowbite/planner/progress (PlanProgress): s between two messages at least (the one as a planning ends at once), done
# of this many, s it grows towards 90 % in without a planning of that area before
PROGRESS_TOPIC = 'mowbite/planner/progress'
PROGRESS_EVERY = 1.0
PROGRESS_OF = 1000
PROGRESS_SLOW = 60.0
# how long the last planning of each area took, kept over a restart (mowbite_planner_took.json next to the settings
# file): the latest TOOK_AREAS areas
TOOK_FILE = 'mowbite_planner_took.json'
TOOK_AREAS = 64
# planner.area.set: OpenMower's area properties (and MowBite's) that stand for a planner setting (see area_settings),
# gone with the area's own value of that setting; the maps before kept
OPENMOWER_PROPS = {'angle': 'angle', 'perimeter_passes': 'outline_count',
                   'lane_overlap_passes': 'outline_overlap_count', 'perimeter_offset': 'outline_offset',
                   'angle_min': 'angle_min', 'angle_max': 'angle_max'}
MAP_BACKUPS = 10
# planner.log: the node's last log lines kept in memory
LOG_LINES = 2000
# run_calib: s an evaluation of a calibration may take at most, its niceness
CALIB_EVALUATE = 1200.0
CALIB_NICE = 10

# where PlannerCore logs without an adapter's logging
log = logging.getLogger('mowbite_planner')


class LogLines(logging.Handler):
    """The node's last LOG_LINES log lines for planner.log, a handler on the logger the adapter's log goes to (the ROS 1
    node's rosout). The unexpected errors come with their traceback."""

    def __init__(self):
        logging.Handler.__init__(self, logging.INFO)
        self.lines = collections.deque(maxlen=LOG_LINES)

    def emit(self, record):
        try:
            # (logfatal's CRITICAL as an ERROR)
            self.lines.append({'t': record.created, 'level': logging.getLevelName(min(record.levelno, logging.ERROR)),
                               'msg': self.format(record)})
        except Exception:  # noqa: BLE001 - a log line must not take the planner down
            self.handleError(record)

    def after(self, since, limit):
        """The lines after since (unix s), only the newest limit of them, the oldest first."""
        with self.lock:
            lines = [line for line in self.lines if line['t'] > since]
        return lines[-limit:]


LOG = LogLines()


class Superseded(Exception):
    """A request still waiting in the worker, superseded by another with its slot (planner.plan)."""


class _Job:
    """A job waiting in the worker or running there: whoever waits for it, (their future, RPC request id, slot)."""

    def __init__(self, urgency, order, fn, args, job, area_id, same, waiter):
        self.urgency, self.order, self.fn, self.args = urgency, order, fn, args
        self.job, self.area_id, self.same, self.waiters = job, area_id, same, [waiter]
        self.stop = None  # (running: why it's stopped)
        self.began = None  # (running: Worker.generation as it began)


class Worker:
    """Everything that plans or looks at a plan's shapes, one job at a time on a thread of its own: planner.plan,
    .leftovers and .drive, mower_logic's plan_path, planning ahead, the checks after a mow. Two threads at one shape can
    crash GEOS (shapely lets go of the GIL, a prepared geometry builds its indexes when first asked), and the other RPCs
    answer meanwhile. The most urgent job first (MOWER ... AHEAD), of those the one asked for first; a job a job asks for
    runs at once. A job planning ahead (AHEAD) stops once a MOWER or ASKED one waits, or once what it plans changed
    (stale), an ASKED or AFTER_MOW one once a MOWER one waits (mower_logic waits for its plan with no time limit), and
    waits again in its place (progress.stoppable: its planning ends at its next step). changed(): the jobs waiting
    changed (called outside the worker's lock), stopped(job, why): one stopped."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jobs, self.running = [], None
        self.order = itertools.count()
        # (one more with every change of the map or the settings, stale: a job begun before it plans what was)
        self.generation = 0
        self.changed = lambda: None
        self.stopped = lambda job, why: None
        self.thread = threading.Thread(target=self.serve, name='planner worker', daemon=True)
        self.thread.start()

    def run(self, urgency, fn, *args, **what):
        """fn(*args) in the worker: its result, or its exception raised here (what: see submit)."""
        return self.submit(urgency, fn, *args, **what)[0]

    def submit(self, urgency, fn, *args, job='plan', area_id=None, rid=None, same=None, slot=None):
        """fn(*args) in the worker: (its result, whether it joined a job waiting or running with the same same (not
        None; a running one only where the map and the settings didn't change since it began): that one's result),
        its exception raised here. job, area_id and rid (the RPC request's id): what waiting tells of it. slot: who
        waits with it for a job not begun yet gets Superseded instead, a job nobody waits for any more is dropped. A
        job asked for more urgently than it waits is moved up."""
        if threading.current_thread() is self.thread:
            return fn(*args), False
        future, gone = Future(), []
        with self.cond:
            if slot is not None:
                for queued in self.jobs:
                    gone += [w for w in queued.waiters if w[2] == slot]
                    queued.waiters = [w for w in queued.waiters if w[2] != slot]
                self.jobs = [queued for queued in self.jobs if queued.waiters]
            running = [self.running] if self.running is not None and self.running.began == self.generation else []
            found = next((j for j in running + self.jobs if j.same == same), None) if same is not None else None
            if found is not None:
                found.waiters.append((future, rid, slot))
                found.urgency = min(found.urgency, urgency)
            else:
                self.jobs.append(_Job(urgency, next(self.order), fn, args, job, area_id, same, (future, rid, slot)))
            if any(j.urgency == MOWER for j in self.jobs):
                self.stop_running('mower_logic waits', (ASKED, AFTER_MOW, AHEAD))
            elif any(j.urgency < AFTER_MOW for j in self.jobs):
                self.stop_running('a request waits', (AHEAD,))
            self.cond.notify()
        for waiter in gone:
            waiter[0].set_exception(Superseded('superseded by a newer request'))
        self.changed()
        return future.result(), found is not None

    def stale(self):
        """The map or the settings changed: a planning ahead that runs stops (it plans what was)."""
        with self.cond:
            self.generation += 1
            self.stop_running('the map or the settings changed', (AHEAD,))

    def stop_running(self, why, ranks):
        if self.running is not None and self.running.urgency in ranks and self.running.stop is None:
            self.running.stop = why

    def urgency(self):
        """The urgency of the job running (None: none)."""
        running = self.running
        return running.urgency if running is not None else None

    def waiting(self):
        """(how many jobs wait, the first WAITING_SHOWN of them in the order they'll run: [{area_id, job, rank,
        ids}])."""
        with self.cond:
            jobs = sorted(self.jobs, key=lambda j: (j.urgency, j.order))
            shown = [{'area_id': j.area_id, 'job': j.job, 'rank': RANKS[j.urgency],
                      'ids': [rid for _, rid, _ in j.waiters if rid]} for j in jobs[:WAITING_SHOWN]]
        return len(jobs), shown

    def serve(self):
        while True:
            with self.cond:
                while not self.jobs:
                    self.cond.wait()
                job = min(self.jobs, key=lambda j: (j.urgency, j.order))
                self.jobs.remove(job)
                self.running, job.began = job, self.generation
            self.changed()
            result, error = None, None
            try:
                with stoppable(lambda: job.stop is not None):
                    result = job.fn(*job.args)
            except BaseException as e:  # noqa: BLE001 - handed to whoever waits
                error = e
            with self.cond:
                self.running = None
                why, job.stop = job.stop, None
                if isinstance(error, Stopped) and why is not None:  # (waits again in its place)
                    self.jobs.append(job)
                waiters = list(job.waiters)
            if isinstance(error, Stopped) and why is not None:
                self.stopped(job, why)
                continue
            for future, _, _ in waiters:
                if error is not None:
                    future.set_exception(error)
                else:
                    future.set_result(result)


class PlanProgress:
    """mowbite/planner/progress (retained, QoS 1, through xbot_monitoring): how far the planning that runs got, at most
    every PROGRESS_EVERY s and once more as it ends. done of PROGRESS_OF grows with plan_coverage's steps and between
    them with the time that area's last planning took (estimate_s; without one more slowly), so a bar keeps moving while
    a long step runs in a process of its own. A planning of no area of the map (an outline asked for) isn't shown: the
    messages stay the last area's. waiting() (the worker's): what waits, in every message; when no area's planning runs
    a change of it is sent too (a while after it, if it's still changed then: a job begun at once isn't). planned_on:
    where it's planned (mower, or house with remote_planning, see where), remote_why: why not on the house after all.
    took_file: where the times the areas' last plannings took are kept ('': only in memory)."""

    def __init__(self, publish, waiting=lambda: (0, []), logwarn=log.warning, took_file=''):
        self.publish, self.waiting, self.logwarn = publish, waiting, logwarn
        self.lock = threading.Lock()
        self.area, self.key, self.stage, self.done, self.running, self.sent = None, None, None, 0, False, 0.0
        self.started, self.ended, self.error, self.unshown = None, None, None, False
        self.on, self.remote_why = 'mower', None
        self.told = (0, [])  # (the waiting jobs the last message had)
        self.later = None  # (the timer sending a change of them)
        self.took_file, self.took_lock = took_file, threading.Lock()
        self.took = self.read_took()  # area id: s its last planning took

    def begin(self, area_id, key):
        """A planning of area_id begins, key: the plan's."""
        with self.lock:
            self.unshown = area_id is None
            if self.unshown:
                return
            self.area, self.key, self.stage, self.part, self.done = area_id, key, 'angles', 0.0, 0
            self.started, self.ended, self.error, self.running = time.time(), None, None, True
            self.on, self.remote_why = 'mower', None
        self.send()
        threading.Thread(target=self.tick, name='planner progress', daemon=True).start()

    def where(self, on, why=None):
        """It's planned on the mower or the house (computer), why not on the house after all."""
        with self.lock:
            if self.unshown:
                return
            self.on, self.remote_why = on, why
        self.send(at_once=True)

    def step(self, stage, done, of):
        """plan_coverage's progress."""
        with self.lock:
            if self.unshown:
                return
            self.stage, self.part = stage, max(self.part, float(done) / max(1, of))

    def end(self, error=None, stopped=False):
        """It ended, error: why it failed; stopped: from outside (no error, its time not kept)."""
        took = None
        with self.lock:
            shown, self.unshown = not self.unshown, False
            if shown and not self.running:
                return
            if shown:
                self.running, self.error, self.ended = False, error, time.time()
                if error is None and not stopped:
                    self.took.pop(self.area, None)
                    self.took[self.area] = self.ended - self.started
                    for old in list(self.took)[:max(0, len(self.took) - TOOK_AREAS)]:
                        del self.took[old]
                    took = dict(self.took)
                    self.stage, self.done = 'done', PROGRESS_OF
        if shown:
            self.send(at_once=True)
        if took is not None:
            self.write_took(took)
        self.queue_changed()  # (what waits now, where that wasn't told yet)

    def read_took(self):
        if not self.took_file:
            return {}
        try:
            with open(self.took_file) as f:
                took = json.load(f)['took']
            return {str(a): float(s) for a, s in took.items() if float(s) > 0.0}
        except FileNotFoundError:
            return {}
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
            self.logwarn('MowBite Planner: planning times not read (%s): %s', self.took_file, e)
            return {}

    def write_took(self, took):
        if not self.took_file:
            return
        try:
            with self.took_lock:
                with open(self.took_file + '.tmp', 'w') as f:
                    json.dump({'took': took}, f)
                os.replace(self.took_file + '.tmp', self.took_file)
        except OSError as e:
            self.logwarn('MowBite Planner: planning times not kept (%s): %s', self.took_file, e)

    def tick(self):
        while self.running:
            time.sleep(PROGRESS_EVERY / 4.0)
            self.send()

    def queue_changed(self):
        """The jobs waiting changed (Worker.changed): sent a while after, see send."""
        with self.lock:
            if self.later is None:
                self.later = threading.Timer(PROGRESS_EVERY / 4.0, self.send_queue)
                self.later.daemon = True
                self.later.start()

    def send_queue(self):
        with self.lock:
            self.later = None
        self.send(queue=True)

    def send(self, at_once=False, queue=False):
        with self.lock:
            now = time.time()
            waiting = self.waiting()
            if queue:  # (while a planning runs its messages tell it)
                if self.running or waiting == self.told:
                    return
                if now - self.sent < PROGRESS_EVERY:
                    self.later = threading.Timer(PROGRESS_EVERY - (now - self.sent), self.send_queue)
                    self.later.daemon = True
                    self.later.start()
                    return
            elif self.area is None or not at_once and (not self.running or now - self.sent < PROGRESS_EVERY):
                return
            estimate = self.took.get(self.area)
            elapsed = None if self.started is None else (self.ended or now) - self.started
            if self.running:
                timed = min(0.95, elapsed / estimate) if estimate else 0.9 * elapsed / (elapsed + PROGRESS_SLOW)
                self.done = max(self.done, int(PROGRESS_OF * min(0.99, max(self.part, timed))))
            self.sent, self.told = now, waiting
            message = {'area_id': self.area, 'key': self.key, 'running': self.running, 'stage': self.stage,
                       'done': self.done, 'of': PROGRESS_OF,
                       'started': round(self.started, 1) if self.started is not None else None,
                       'elapsed_s': round(elapsed, 1) if elapsed is not None else None,
                       'estimate_s': round(estimate, 1) if estimate else None, 'error': self.error,
                       'waiting_n': waiting[0], 'waiting': waiting[1], 'planned_on': self.on,
                       'remote_why': self.remote_why}
        try:
            self.publish(json.dumps(message))
        except Exception as e:  # noqa: BLE001 - showing progress must not take a planning down
            self.logwarn('MowBite Planner: progress not published: %s', e)



# the id of the RPC request the thread answers (XbotRpc.answer), for the worker's waiting jobs
REQUEST = threading.local()


def asked_id():
    """The id of the RPC request this thread answers, None: none."""
    return getattr(REQUEST, 'id', None) or None



def active_areas(data):
    """The map's active mowing areas with an id (what isn't one left out)."""
    return [a for a in _areas(data) if _props(a).get('type') == 'mow' and _props(a).get('active') is not False and
            a.get('id')]


def same_request(params):
    """What makes two planner.plan requests the same (their answer the same): their params but ahead and slot."""
    return json.dumps({k: v for k, v in params.items() if k not in ('ahead', 'slot')}, sort_keys=True)


def call(methods, method, params, loginfo=log.info, logerr=log.error, data=None):
    """An RPC method of methods with params (None: none): (its result as JSON, None) or (None, (code, message)), a
    PlanningError the caller's mistake; loginfo, logerr: the adapter's. data: a dict for the error's data, it gets
    'code' where a PlanningError has a word (as a service's code says it: BODY_OUT, NOT_READY...)."""
    try:
        if params is None:  # none given, or null
            params = {}
        if not isinstance(params, dict):
            raise PlanningError('params must be an object (by name)')
        return json.dumps(methods[method](params), allow_nan=False), None
    except PlanningError as e:  # (with a code of its own, another node's: that one)
        loginfo('MowBite Planner: %s: %s', method, e)
        if data is not None and e.word:
            data['code'] = e.word
        return None, (getattr(e, 'code', None) or INVALID_PARAMS, str(e))
    except Superseded as e:
        loginfo('MowBite Planner: %s: %s', method, e)
        return None, (SUPERSEDED, str(e))
    except Exception as e:  # noqa: BLE001 - an RPC must not take the planner down
        logerr('MowBite Planner: %s failed: %s: %s', method, type(e).__name__, e, exc_info=True)
        return None, (INTERNAL, 'internal error: %s' % e)



class PlannerCore:
    """The planner's node without the robot, see the top. mower_width: the blade (m), settings: the settings for all
    areas the robot starts it with (the planner's and NODE_SETTINGS), stored over them from settings_file; workers:
    processes of their own that plan (planner.set_workers)."""

    # {setting: why} the robot can't have, {setting: where from} it gives itself (see the top), an adapter's own
    cannot = {}
    given_by = {}

    def __init__(self, mower_width, settings, settings_file, plans_dir, history_dir, workers=0):
        # (before anything hands it work)
        self.worker = Worker()
        self.progress = PlanProgress(lambda payload: self.publish(PROGRESS_TOPIC, payload, True), self.worker.waiting,
                                     self.logwarn,
                                     os.path.join(os.path.dirname(settings_file), TOOK_FILE) if settings_file else '')
        self.worker.changed = self.progress.queue_changed
        self.worker.stopped = lambda job, why: self.loginfo(
            'MowBite Planner: planning %s %s stopped (%s), it waits again', job.area_id or 'a plan',
            'ahead' if job.urgency == AHEAD else '(%s)' % RANKS[job.urgency], why)
        self.launch_width = float(mower_width)
        self.launch = dict(settings or {})
        self.defaults = self.setup(self.launch_width, self.launch, {})
        self.file = settings_file
        # swapped as a whole, a plan never sees half of a change
        self.current = self.defaults
        stored = self.read_stored()
        # (one a newer version stored: left out, the rest still counts)
        unknown = sorted(set(stored) - set(PlannerConfig.names()) - set(NODE_SETTINGS) - set(limits.INTERNAL) -
                         set(limits.RETIRED))
        if unknown:
            self.logwarn('MowBite Planner: stored settings not known, left out (%s): %s', self.file,
                         ', '.join(unknown))
            stored = {n: v for n, v in stored.items() if n not in unknown}
        # (the stored settings left out at the start, as they were, only the ones that don't go with the others or
        # the mower: the robot's given values that come later may take them, given_again(); a change of settings
        # takes them first)
        self.left_out = {}
        if stored:
            self.current, self.left_out = self.settle(stored)
        self.map, self.map_text = None, None  # (as the robot sent it)
        self.map_error = ''  # (why the last map sent isn't planned with, set_map)
        self.map_came = threading.Event()  # (set once there's a map)
        self.lock = threading.Lock()
        self.set_lock = threading.Lock()
        self.plans = collections.OrderedDict()  # key -> (plan, where it was planned, its pickle's bytes)
        self.plans_lock = threading.Lock()
        self.refused = collections.OrderedDict()  # key -> the planner's no (PlanningError) to it, the same at once
        self.planning = {}  # a lock for each plan being planned: asked for again meanwhile, it's waited for
        self.plans_dir = plans_dir
        self.code = code_version()
        # (what the plans in plans_dir are told by: the code and what else they depend on, remote.stack)
        self.build = hashlib.sha1((self.code + json.dumps(stack(), sort_keys=True)).encode()).hexdigest()[:12]
        self.drop_old_plans()
        self.pinned = self.read_pinned() if self.plans_dir else set()
        self.ahead = None
        self.ahead_lock = threading.Lock()
        self.ahead_count = 0  # (the latest plan_ahead, one before it ends early)
        self.ahead_run = threading.Lock()
        # the finished mows counted for angle_increment: steps so far, mows since the last one, and all of them
        # (perimeter_direction random), next to settings_file
        self.angle_file = os.path.join(os.path.dirname(self.file), 'mowbite_planner_angle.json') if self.file else ''
        self.angle_lock = threading.Lock()
        self.angle = self.read_angle()
        self.local = threading.local()  # whether the last plan of this thread was kept from before
        # the plans whose warnings are in the log already (by id): asked for again, a preview or the mow, only how many
        self.warned = weakref.WeakValueDictionary()
        self.history = history_dir
        self.drive = None  # the last finished mow's planner.drive
        self.drive_thread = None
        # [[estimate s, took s, job id], ...] of the last finished mows, and the plans the robot got: [(when, area id,
        # estimate s)]
        self.timing_file = os.path.join(os.path.dirname(self.file), 'mowbite_planner_timing.json') if self.file else ''
        self.timing_lock = threading.Lock()
        self.timing = self.read_timing()
        self.served = collections.deque(maxlen=64)
        # planner.leftovers armed: {'until': when it ends (None: with its job), 'plans': {area id: its leftovers' plan or
        # None}, 'served': the robot got one of them, 'job': the id of the job it got them in (None: none known),
        # 'key': the map and settings they're of (armed_key)}, kept over a restart
        self.armed = self.read_armed()
        # the job (its id, '' none known) mower_logic was refused a plan in (no map): its JOB_COMPLETE isn't a mow
        self.refused_job = None
        # the job of the last event ('' none), see track_written
        self.job = ''
        self.job_change = threading.Condition()
        # planner.area.set: one change of the map at a time, the maps before next to settings_file
        self.map_lock = threading.Lock()
        self.backups = os.path.join(os.path.dirname(self.file), 'mowbite_map_backups') if self.file else ''
        self.workers = max(0, int(workers or 0))
        set_workers(self.workers)
        self.methods = {'planner.settings': self.rpc_settings, 'planner.settings.set': self.rpc_settings_set,
                        'planner.area.set': self.rpc_area_set,
                        'planner.plan': self.rpc_plan_asked,
                        'planner.drive': lambda params: self.worker.run(ASKED, self.rpc_drive, params, job='drive',
                                                                        rid=asked_id()),
                        'planner.leftovers': lambda params: self.worker.run(ASKED, self.rpc_leftovers, params,
                                                                            job='leftovers', rid=asked_id()),
                        'planner.angle.reset': self.rpc_angle_reset, 'planner.log': self.rpc_log}

    # what the adapter gives it (see the top)

    def loginfo(self, *args, **kwargs):
        log.info(*args, **kwargs)

    def logwarn(self, *args, **kwargs):
        log.warning(*args, **kwargs)

    def logerr(self, *args, **kwargs):
        log.error(*args, **kwargs)

    def logwarn_throttle(self, period, message):
        """message as a warning, the same one at most every period s."""
        now = time.monotonic()
        said = self.__dict__.setdefault('_said', {})
        if now >= said.get(message, -math.inf) + period:
            said[message] = now
            self.logwarn(message)

    def publish(self, topic, payload, retain):
        pass

    def param(self, name, default=None):
        return default

    def replace_map(self, data, based_on):
        """planner.area.set: data written as the robot's map, where it's still the one of the text based_on.
        PlanningError where it isn't written."""
        raise PlanningError('the map can\'t be written here')

    def map_unusable(self):
        return ''

    def unusable(self):
        """Why the map can't be planned now ('' when it can): the last one sent wasn't readable, or map_unusable."""
        return self.map_error or self.map_unusable()

    def given(self):
        """{setting: value} of given_by as the robot has them now ({}: none yet, the defaults count)."""
        return {}

    def plan_map(self, data):
        return data

    def arrives(self, boundary, obstacles, cfg, drivable, keep_out, reach_over=(), unmowed=(), margins=(), bodies=()):
        """For a plan of these (plan_coverage's), arrives(pose, came_from): whether the robot gets to the pose facing its
        heading from came_from (planner.starts), None: it drives what it's given."""
        return None

    def hints_missing(self):
        """Whether planner.plan's body_hints tells of the areas recorded without a body (MowBite OS)."""
        return False

    def classic_plan(self, boundary, obstacles, cfg, *rest, **more):
        raise PlanningError('no slic3r planner here')

    def remote_plan(self, key, form, args, reach_over, unmowed, area_id, bodies=()):
        return None

    def remote_state(self):
        return 'no house computer', None

    def drive_ignored(self):
        """Why drive slope and reverse are planned forwards here ('' where they aren't)."""
        return ''

    def checked(self, plan):
        """plan as it's kept and handed out: the robot's own last check of one just planned (MowBite OS: the body's),
        a PlanningError where it doesn't take it. Here as it is."""
        return plan

    def back_out_ignored(self):
        """Why back_out is off here ('' where it isn't): where the robot doesn't back up, as for drive."""
        return self.drive_ignored()

    def recorded_loop_ignored(self, recorded):
        """Why recorded_loop is off here for an area ('' where it isn't; recorded: whether the map has the body it was
        recorded with, recorded_body). Here for none: an area without is taken as recorded with this mower."""
        return ''

    def mower_ready(self):
        return False

    def held(self):
        """Why the mower may not be sent out now ('' when it may)."""
        return ''

    def start_mowing(self):
        pass

    def job_complete(self):
        """A mow finished (JOB_COMPLETE), before the rest that follows it."""

    def area_setting(self, props, own, name, value):
        """planner.area.set: an area's setting (props its properties, own its planner property) into the map, value
        None: the area's own gone (and OpenMower's property that stands for it, OPENMOWER_PROPS)."""
        if value is None:
            own.pop(name, None)
            if name in OPENMOWER_PROPS:
                props.pop(OPENMOWER_PROPS[name], None)
        else:
            own[name] = value

    def current_map(self):
        with self.lock:
            return self.map

    def rpc_area_set(self, params):
        """planner.area.set, see the top."""
        area_id, settings = params.get('area_id'), params.get('settings')
        if settings is not None and not isinstance(settings, dict):
            raise PlanningError('settings must be an object of planner settings')
        # (settings of before an app may still send: left out)
        settings = settings and {n: v for n, v in settings.items() if n not in limits.RETIRED}
        settable = set(PlannerConfig.names()) - set(FIXED) - set(self.given_by) - set(self.cannot)
        unknown = sorted(set(settings or {}) - settable)
        if unknown:
            raise PlanningError('not settings an area can have: %s' % ', '.join(unknown))
        with self.map_lock:
            with self.lock:
                data, text = self.map, self.map_text
            if data is None:
                raise PlanningError('no map yet')
            if self.map_error:
                raise PlanningError(self.map_error)
            before, data = data, json.loads(json.dumps(data))
            area = next((a for a in data.get('areas', []) if isinstance(a, dict) and a.get('id') == area_id), None)
            if area is None:
                raise PlanningError('no area %r in the map' % area_id)
            props = area.get('properties')
            if not isinstance(props, dict):
                props = area['properties'] = {}
            if settings:
                if props.get('type') != 'mow':
                    raise PlanningError('settings only for mowing areas')
                own = {n: v for n, v in (props.get('planner') or {}).items() if n not in limits.RETIRED}
                for name, value in settings.items():
                    self.area_setting(props, own, name, value)
                if own:
                    props['planner'] = own
                else:
                    props.pop('planner', None)
                # (as a plan of it takes them)
                base = self.current.base.with_overrides({'mower_width': self.current.mower_width})
                base.with_overrides(area_settings(area)).validate()
            if 'margin' in params:
                margin = params['margin']
                if props.get('type') == 'mow':
                    raise PlanningError('a margin only for obstacles and areas to keep out of')
                if margin is None:
                    props.pop('margin', None)
                elif isinstance(margin, bool) or not isinstance(margin, (int, float)) or \
                        not 0.0 <= margin <= limits.MAX_BLADE_OFFSET:
                    raise PlanningError('margin must be 0 to %g m' % limits.MAX_BLADE_OFFSET)
                else:
                    props['margin'] = margin
            self.backup_map(before)
            self.replace_map(data, text)
            with self.lock:
                self.map = data
            self.keep_armed(None)  # (they were of the area as it was)
        self.loginfo('MowBite Planner: area %s now %s', area_id, json.dumps(
            {k: v for k, v in props.items() if k in ('planner', 'margin') or k in OPENMOWER_PROPS.values()},
            sort_keys=True))
        self.plan_ahead_later()
        return {'area_id': area_id, 'properties': props}

    def backup_map(self, data):
        """The map as it was, next to settings_file, the last MAP_BACKUPS kept."""
        if not self.backups:
            return
        try:
            os.makedirs(self.backups, exist_ok=True)
            now = time.time()
            name = os.path.join(self.backups, 'map_%s_%03d.json' % (time.strftime('%Y%m%d_%H%M%S', time.localtime(now)),
                                                                   int(now * 1000) % 1000))
            with open(name + '.tmp', 'w') as f:
                json.dump(data, f)
            os.replace(name + '.tmp', name)
            for old in sorted(glob.glob(os.path.join(self.backups, 'map_*.json')))[:-MAP_BACKUPS]:
                os.remove(old)
        except OSError as e:  # (the change goes ahead all the same, like the app's without its container)
            self.logwarn('MowBite Planner: map not backed up (%s): %s', self.backups, e)

    @property
    def base(self):
        return self.current.base

    def setup(self, width, settings, stored):
        """The settings with the stored ones on top, PlanningError if they don't go together. The ones the robot
        can't have (cannot) at their default."""
        # (settings of before left out, they're fixed now or gone; the robot's own given ones)
        stored = {n: v for n, v in stored.items() if n not in limits.INTERNAL and n not in limits.RETIRED and
                  n not in self.cannot and n not in self.given_by}
        settings = {n: v for n, v in settings.items() if n not in self.cannot and n not in self.given_by}
        on_top = {n: v for n, v in stored.items() if n != 'mower_width'}
        given = self.given() if self.given_by else {}
        width = given.get('mower_width', width)
        settings = dict(settings, **on_top, **{n: v for n, v in given.items() if n != 'mower_width'})
        planner = settings.pop('planner', PLANNERS[0])
        if planner not in PLANNERS:
            raise PlanningError('planner must be one of %s' % ', '.join(PLANNERS))
        leftovers_auto = settings.pop('leftovers_auto', False)
        remote_planning = settings.pop('remote_planning', False)
        for name, value in (('leftovers_auto', leftovers_auto), ('remote_planning', remote_planning)):
            if not isinstance(value, bool):
                raise PlanningError('%s must be true or false' % name)
        # OpenMower's FTC controller only drives forwards: lanes far enough apart that every turn is a forward U-turn
        # of turn_radius. ~settings can change that (and anything else)
        base = PlannerConfig(lane_order='skip').with_overrides(settings)
        # (checked with the blade's width: lane_spacing_min and lane_spacing_max go by it)
        blade = base.with_overrides({'mower_width': stored.get('mower_width', width)})
        blade.validate()
        # with angle or angle_strategy in settings the lanes go that way, not mower_logic's angle, unless the area
        # has its own
        own_angle = 'angle' in settings or 'angle_strategy' in settings
        # as the planner took them: ints of a float setting as floats, turn_types as a list
        node = {'planner': planner, 'leftovers_auto': leftovers_auto, 'remote_planning': remote_planning}
        stored = {n: node[n] if n in node else getattr(blade if n == 'mower_width' else base, n) for n in stored}
        return Setup(blade.mower_width, base, own_angle,
                     {n: list(v) if isinstance(v, tuple) else v for n, v in stored.items()},
                     frozenset(n for n in settings if n in MOWER_LOGIC), planner, leftovers_auto, remote_planning)

    def settle(self, stored, taken=None):
        """(the settings with as many of the stored ones as go: all of them, else one after the other by name each
        that goes with the ones before, the stored ones left out {name: value}); in the log which and why. taken:
        stored ones that go already (kept)."""
        taken = dict(taken or {})
        try:
            return self.setup(self.launch_width, self.launch, dict(taken, **stored)), {}
        except PlanningError as e:
            why = e
        for name in sorted(stored):
            try:
                self.setup(self.launch_width, self.launch, dict(taken, **{name: stored[name]}))
                taken[name] = stored[name]
            except PlanningError:
                pass
        left = {n: v for n, v in stored.items() if n not in taken}
        self.logwarn('MowBite Planner: stored settings left out (%s): %s (%s)', self.file, ', '.join(sorted(left)),
                     why)
        return self.setup(self.launch_width, self.launch, taken), left

    def given_again(self):
        """The robot's given values changed (given()): the settings with them from now on, the leftovers armed gone
        (they were the values before), planned ahead again. PlanningError where they don't go with the settings
        (nothing changed then)."""
        with self.set_lock:
            current = self.setup(self.launch_width, self.launch, self.current.stored)
            if self.left_out:
                left = self.left_out
                current, self.left_out = self.settle(left, current.stored)
                if len(self.left_out) < len(left):
                    self.loginfo('MowBite Planner: stored settings taken after all (%s): %s', self.file,
                                 ', '.join(sorted(set(left) - set(self.left_out))))
            self.defaults = self.setup(self.launch_width, self.launch, {})
            self.current = current
            self.keep_armed(None)
        self.loginfo('MowBite Planner: %s', ', '.join('%s %s' % (n, json.dumps(v)) for n, v in sorted(
            self.given().items())))
        self.plan_ahead_later()

    def bound(self, cfg):
        """cfg as the robot holds it: its given values and the planner settings it can't have at their default (an
        area's or a request's own don't count)."""
        if not self.given_by and not set(self.cannot) & set(PlannerConfig.names()):
            return cfg
        given = {n: v for n, v in self.current_given().items() if getattr(cfg, n) != v}
        # (as the settings have them: robot_outline None too)
        return dataclasses.replace(cfg, **given) if given else cfg

    def current_given(self):
        """given_by's values and the planner settings of cannot as the settings have them now (canon'd as a plan's
        settings)."""
        current = self.current
        names = list(self.given_by) + [n for n in self.cannot if n in PlannerConfig.names() and n not in self.given_by]
        return {n: current.mower_width if n == 'mower_width' else getattr(current.base, n) for n in names}

    def read_stored(self):
        if not self.file or not os.path.exists(self.file):
            return {}
        try:
            with open(self.file) as f:
                stored = json.load(f)
            if not isinstance(stored, dict):
                raise ValueError('not an object')
            return stored
        except (OSError, ValueError) as e:
            self.logwarn('MowBite Planner: stored settings not readable (%s): %s', self.file, e)
            return {}

    def write_stored(self, stored):
        if not self.file:
            return
        tmp = self.file + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(stored, f, indent=1, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.file)

    def rpc_settings(self, params=None):
        current, defaults = self.current, self.defaults
        theirs = openmower_settings(self.param('/mower_logic', {}), current.mower_width)
        out = {}
        for name in PlannerConfig.names():
            default = PlannerConfig.__dataclass_fields__[name].default
            kind = ('boolean' if isinstance(default, bool) else 'integer' if isinstance(default, int)
                    else 'string' if isinstance(default, str) else 'list' if isinstance(default, tuple)
                    else 'points' if name in POINTS else 'number')
            blade = name == 'mower_width'
            entry = {'value': current.mower_width if blade else getattr(current.base, name),
                     'default': defaults.mower_width if blade else getattr(defaults.base, name),
                     'stored': name in current.stored, 'type': kind, 'settable': name not in FIXED}
            if name in FIXED:
                entry['from'] = FIXED[name]
            elif name in MOWER_LOGIC and name not in current.own:  # mower_logic's (an area's own still beats it)
                entry['from'] = MOWER_LOGIC[name]
                entry['value'] = theirs.get(name, entry['value'])
            if name in MOWER_LOGIC and name not in defaults.own and name in theirs:  # (what counts once it isn't set)
                entry['theirs'] = theirs[name]
            if default is None:
                entry['nullable'] = True
            if name in CHOICES:
                entry['choices'] = CHOICES[name]
            if name in IGNORED:  # (changes nothing since planner 2: the app hides it)
                entry['ignored'] = IGNORED[name]
            if name in ('drive', 'back_out'):
                why = (self.drive_ignored() if name == 'drive' else self.back_out_ignored()) or \
                    backing.unavailable(current.base)
                if why:
                    entry['ignored'] = why
                else:
                    entry['forces'] = {'body_fit': True, 'spin_margin': limits.DRIVE_SPIN_MARGIN}
                    entry['forces_why'] = ('slope and reverse: ' if name == 'drive' else 'back_out: ') + backing.WHY
            if name == 'recorded_loop':  # (what else is set keeps it from doing anything)
                why = dataclasses.replace(current.base, recorded_loop=True).recorded_off
                if why:
                    entry['ignored'] = why
            if name in ('body_fit', 'spin_margin'):
                forced = self.forced_by(current.base)
                if forced is not None:
                    entry['effective'] = getattr(forced[0], name)
                    if entry['effective'] != entry['value']:
                        entry['effective_why'] = '%s: %s' % (forced[1], backing.WHY)
            if name in ('lane_spacing_min', 'lane_spacing_max'):  # (what lane_spacing_mode 'auto' goes by)
                k = name == 'lane_spacing_max'
                cfg = current.base.with_overrides({'mower_width': current.mower_width})
                entry['effective'] = round(cfg.spacing_range[k], 4)
                if k and cfg.spacing_range[1] < cfg.spacing_asked[1] - 1e-9:
                    entry['effective_why'] = (f'no wider than the blade covers with the mower {limits.TRACK * 100:.0f} '
                                              'cm off its way each side, but the widest asked where that leaves '
                                              'less unmowed or sticks into spin_margin less often')
            if name in AUTO_VALUES:
                entry['auto_value'] = AUTO_VALUES[name]
            if kind == 'number':
                entry['unit'] = 'rad' if name in RAD else '' if name in ('overlap', 'transit_edge_cost') else 'm'
                if name not in SIGNED:
                    entry['min'] = 0.0
            elif kind == 'points':
                entry['unit'] = 'm'
            if name in RANGES:
                entry['min'], hi = RANGES[name]
                if hi is not None:
                    entry['max'] = hi
            out[name] = {k: list(v) if isinstance(v, tuple) else v for k, v in entry.items()}
        out['planner'] = {'value': current.planner, 'default': defaults.planner, 'stored': 'planner' in current.stored,
                          'type': 'string', 'settable': True, 'choices': list(PLANNERS)}
        out['leftovers_auto'] = {'value': current.leftovers_auto, 'default': defaults.leftovers_auto,
                                 'stored': 'leftovers_auto' in current.stored, 'type': 'boolean', 'settable': True}
        why, house = self.remote_state()
        out['remote_planning'] = {'value': current.remote_planning, 'default': defaults.remote_planning,
                                  'stored': 'remote_planning' in current.stored, 'type': 'boolean', 'settable': True}
        if why:
            out['remote_planning']['ignored'] = why
        for name, why_not in self.cannot.items():
            out[name].update(settable=False, ignored=why_not)
            if name in FIXED:  # (not mower_logic's where there's none)
                out[name].pop('from', None)
        for name, where in self.given_by.items():
            out[name].update({'settable': False, 'from': where})
        # angle_increment: the steps turned so far and the finished mows towards the next one (and all of them)
        steps, mows = self.angle['steps'], self.angle['mows']
        return {'settings': out, 'own_angle': current.own_angle, 'file': self.file or None,
                'angle_steps': steps, 'angle_mows': mows,
                'angle_turned': steps * current.base.angle_increment, 'finished_mows': self.angle['finished'],
                'remote': house or {'reachable': False, 'arch': None, 'code_match': False, 'stack_match': False,
                                    'why': why}, 'version': mowbite_planner.__version__}

    def forced_by(self, cfg):
        """(what cfg is planned with, what forces it) where drive slope or reverse or back_out turn the body check on
        here, else None."""
        if backing.unavailable(cfg):
            return None
        if cfg.drive != 'forward' and not self.drive_ignored():
            return backing.settings(cfg), 'drive ' + cfg.drive
        if cfg.back_out and not self.back_out_ignored():
            return backing.backed(cfg)[0], 'back_out'
        return None

    def rpc_angle_reset(self, params=None):
        """Back to no steps turned (angle_increment)."""
        with self.angle_lock:
            self.angle = dict(self.angle, steps=0, mows=0)
            self.write_angle()
        self.loginfo('MowBite Planner: angle steps reset')
        self.plan_ahead_later()
        return self.rpc_settings()

    def rpc_log(self, params):
        """planner.log, see the top."""
        since = params.get('since')
        limit = params.get('limit')
        if since is not None and (isinstance(since, bool) or not isinstance(since, (int, float))):
            raise PlanningError('since must be a time in unix s')
        if limit is not None and (isinstance(limit, bool) or not isinstance(limit, int) or limit < 1):
            raise PlanningError('limit must be a whole number of 1 or more')
        return {'lines': LOG.after(-math.inf if since is None else since, LOG_LINES if limit is None else limit)}

    def read_angle(self):
        try:
            with open(self.angle_file) as f:
                data = json.load(f)
            # (finished: not in it before perimeter_direction random)
            return {'steps': max(0, int(data['steps'])), 'mows': max(0, int(data['mows'])),
                    'finished': max(0, int(data.get('finished', 0)))}
        except (OSError, ValueError, KeyError, TypeError, OverflowError):
            return {'steps': 0, 'mows': 0, 'finished': 0}

    def write_angle(self):
        if not self.angle_file:
            return
        try:
            with open(self.angle_file + '.tmp', 'w') as f:
                json.dump(self.angle, f)
            os.replace(self.angle_file + '.tmp', self.angle_file)
        except OSError as e:
            self.logwarn('MowBite Planner: angle steps not kept (%s): %s', self.angle_file, e)

    def on_event(self, payload):
        """An event of the mower's logic (EVENTS_TOPIC's payload): every angle_increment_every finished mows
        (JOB_COMPLETE, not JOB_RESET) one step more, while angle_increment turns the lanes (for all areas or one's
        own)."""
        try:
            event = json.loads(payload)
        except ValueError:
            return
        if not isinstance(event, dict):
            return
        if event.get('type') == 'STATE':
            job_id = event.get('job_id') if isinstance(event.get('job_id'), str) else ''
            with self.job_change:
                self.job = job_id
                self.job_change.notify_all()
            self.armed_job(job_id)
            return
        if event.get('type') in ('JOB_RESET', 'JOB_COMPLETE'):
            self.unpin()  # (the job's plans aren't resumed any more)
            refused, self.refused_job = self.refused_job, None
            if event.get('type') == 'JOB_COMPLETE' and refused is not None and \
                    refused == (event.get('job_id') or ''):
                # (mower_logic docked as it got no plan: no mow, the lanes don't turn on, nothing follows it)
                self.loginfo('MowBite Planner: the job ended without a plan (it was refused one): not a finished mow')
                return
        if event.get('type') == 'JOB_RESET':
            armed = self.armed
            if armed is not None and armed['served']:  # (the leftovers' job thrown away: the next mow is a whole one)
                self.keep_armed(None)
            return
        if event.get('type') != 'JOB_COMPLETE':
            return
        self.job_complete()
        armed = self.armed
        self.keep_armed(None)
        if armed is not None and armed['served']:
            # (the leftovers of the mow before: no mow of its own, the lanes don't turn on, it isn't timed)
            self.loginfo('MowBite Planner: the leftovers are mowed')
            self.served.clear()
            return
        with self.job_change:
            self.job = event.get('job_id') or ''
        self.drive_thread = threading.Thread(target=self.after_mow, args=(event.get('job_id'), time.time()),
                                             daemon=True)
        self.drive_thread.start()
        base = self.current.base
        with self.lock:
            data = self.map
        own = [_props(a).get('planner') for a in _areas(data)]
        own = [p for p in own if isinstance(p, dict)]
        turning = base.angle_increment != 0 or any(p.get('angle_increment') for p in own)
        # (perimeter_direction random: from now on another way round, the plans ahead too)
        rolling = base.perimeter_direction == 'random' or any(p.get('perimeter_direction') == 'random' for p in own)
        if turning and self.param('/mower_logic/mow_angle_increment', 0):
            self.logwarn('MowBite Planner: mower_logic turns the lanes too (mow_angle_increment), both add up')
        with self.angle_lock:
            self.angle['finished'] += 1
            if turning:
                self.angle['mows'] += 1
                if self.angle['mows'] >= base.angle_increment_every:
                    self.angle.update(steps=self.angle['steps'] + 1, mows=0)
            self.write_angle()
            steps, finished = self.angle['steps'], self.angle['finished']
        if turning or rolling:
            self.loginfo('MowBite Planner: mow finished, angle steps %d, %d finished mows', steps, finished)
            self.plan_ahead_later()

    def rpc_settings_set(self, params):
        fixed = dict(FIXED, **self.cannot, **{n: 'from %s' % where for n, where in self.given_by.items()})
        fixed = {n: fixed[n] for n in sorted(set(params) & set(fixed))}
        if fixed:
            raise PlanningError('not settable here: %s' % ', '.join('%s (%s)' % (n, why) for n, why in fixed.items()))
        # (settings of before an app may still send: left out)
        params = {n: v for n, v in params.items() if n not in limits.INTERNAL and n not in limits.RETIRED}
        unknown = sorted(set(params) - set(PlannerConfig.names()) - set(NODE_SETTINGS))
        if unknown:
            raise PlanningError('unknown planner settings: %s' % ', '.join(unknown))
        with self.set_lock:
            # (with the ones left out at the start where there are: they'd be gone from the file else)
            stored = dict(self.current.stored, **self.left_out)
            for name, value in params.items():
                if value is None:
                    stored.pop(name, None)
                else:
                    stored[name] = value
            try:
                current = self.setup(self.launch_width, self.launch, stored)
            except PlanningError as e:
                if self.left_out:
                    raise PlanningError('the settings stored before (%s) don\'t go with the mower: %s' % (
                        ', '.join(sorted(self.left_out)), e))
                raise
            self.left_out = {}
            self.write_stored(current.stored)
            # (leftovers_auto changes no plan, the rest of stored is in the others)
            changed = current._replace(stored=None, leftovers_auto=None) != \
                self.current._replace(stored=None, leftovers_auto=None)
            self.current = current
            self.keep_armed(None)  # (the leftovers were other settings')
        self.loginfo('MowBite Planner: settings %s', json.dumps(current.stored, sort_keys=True))
        self.plan_ahead_later(changed)
        return self.rpc_settings()

    def rpc_plan_asked(self, params):
        """planner.plan: rpc_plan in the worker, ahead and slot as the module's docstring says."""
        ahead, slot = params.get('ahead', False), params.get('slot')
        if not isinstance(ahead, bool):
            raise PlanningError('ahead must be true or false')
        if slot is not None and not isinstance(slot, str):
            raise PlanningError('slot must be a string')
        area_id = params.get('area_id')
        result, joined = self.worker.submit(AHEAD if ahead else ASKED, self.rpc_plan, params, rid=asked_id(),
                                            area_id=area_id if isinstance(area_id, str) else None,
                                            same=same_request(params), slot=slot)
        return dict(result, kept=True) if joined else result

    def rpc_plan(self, params, keep=None, mowed=None):
        """keep: a list the plan itself goes to too. mowed: where the blade went already, only what that left is planned
        (not kept: it's that one mow's). PlanningError where the map can't be planned now (map_unusable)."""
        reason = self.unusable()
        if reason:
            raise PlanningError(reason, word='NOT_READY')
        current = self.current
        # (the map as ~/plan_coverage takes it: the same plan for both)
        data = self.plan_map(self.current_map())
        if current.planner == 'slic3r' and keep is None:
            # what mower_logic would ask the slic3r planner: OpenMower's settings and the area's, not the node's
            service = PlannerService(PlannerConfig(mower_width=current.mower_width, nested_areas=False))
            service.planner = self.classic_plan
        else:
            service = PlannerService(current.base.with_overrides({'mower_width': current.mower_width,
                                                                  'angle_steps': self.angle['steps'],
                                                                  'finished_mows': self.angle['finished']}),
                                     current.own_angle)
            service.planner = self.plan_coverage if keep is None else (
                lambda *args, **kw: keep.append(self.plan_coverage(*args, **kw)) or keep[-1])
            service.own = current.own
        service.map = data
        service.missing = self.hints_missing()
        service.params = self.param('/mower_logic', {})
        self.local.kept, self.local.on = False, 'mower'
        got = []
        if mowed is not None:
            service.planner = lambda *args, **kw: got.append(self.left_plan(*args, mowed=mowed, **kw)) or got[-1]
        elif keep is None and current.planner != 'slic3r':
            service.planner = lambda *args, **kw: got.append(self.plan_coverage(*args, **kw)) or got[-1]
        result = service.plan(params)
        if mowed is not None and keep is not None:
            keep.extend(got)
        result['kept'] = self.local.kept  # planned before (ahead, or the same asked for already)
        result['planned_on'] = self.local.on
        if got:
            result['duration'] = self.duration(self.estimate(got[-1]))
        return result

    def estimate(self, plan):
        """s the mower takes for plan as the model says (OpenMower's FTC speeds)."""
        return drive_time(plan.poses, self.param(FTC_PARAMS, {}))['s']

    def duration(self, estimate):
        """planner.plan's duration of a plan the model gives estimate s for: times the finished mows' factor."""
        with self.timing_lock:
            runs = list(self.timing)
        est = sum(est for est, _, _ in runs)
        factor = sum(took for _, took, _ in runs) / est if est > 0.0 else 1.0
        return {'min': round(estimate * factor / 60.0, 1), 'model_min': round(estimate / 60.0, 1),
                'factor': round(factor, 3), 'runs': len(runs)}

    def read_timing(self):
        try:
            with open(self.timing_file) as f:
                runs = json.load(f)['runs']
            return [[float(est), float(took), str(job)] for est, took, job in runs if float(est) > 0.0][-TIMING_RUNS:]
        except (OSError, ValueError, KeyError, TypeError):
            return []

    def time_after(self, job_id, ended=None):
        """How long the finished job took against the model, kept for duration (see TIMING_RUNS), in the log: all of
        its track files (a restart begins another), ended: when it finished (JOB_COMPLETE, unix s; None: not known)."""
        files = glob.glob(os.path.join(self.history, '*_%s.jsonl' % job_id)) if self.history and job_id else []
        if not files:
            return
        try:
            files.sort(key=track_start)
            start = track_start(files[0])
            took = mowing_time(files, ended)
        except (OSError, ValueError):
            return
        served = [(area, est) for t, area, est in list(self.served) if t >= start - 5.0]
        self.served.clear()  # (those were for this job)
        areas = [area for area, _ in served]
        if not served or took is None or len(set(areas)) != len(areas):  # (resumed: a plan twice, not counted)
            self.loginfo('MowBite Planner: job %s not timed', job_id)
            return
        est = sum(e for _, e in served)
        if est <= 0.0:
            self.loginfo('MowBite Planner: job %s not timed, no time estimated', job_id)
            return
        if not TIMING_RANGE[0] * est <= took <= TIMING_RANGE[1] * est:
            self.logwarn('MowBite Planner: job %s took %.0f min, %.0f min estimated: not counted', job_id, took / 60.0,
                         est / 60.0)
            return
        with self.timing_lock:
            self.timing = (self.timing + [[round(est, 1), round(took, 1), str(job_id)]])[-TIMING_RUNS:]
            runs = list(self.timing)
        if self.timing_file:
            try:
                with open(self.timing_file + '.tmp', 'w') as f:
                    json.dump({'runs': runs}, f)
                os.replace(self.timing_file + '.tmp', self.timing_file)
            except OSError as e:
                self.logwarn('MowBite Planner: timing not kept (%s): %s', self.timing_file, e)
        self.loginfo('MowBite Planner: job %s took %.1f min, %.1f min estimated, factor now %.2f (%d mows)', job_id,
                     took / 60.0, est / 60.0, sum(t for _, t, _ in runs) / sum(e for e, _, _ in runs), len(runs))

    def track_files(self, job_id=None, name=None):
        """The position_history files of a job (or by name), the latest last; without either all of them."""
        files = sorted(glob.glob(os.path.join(self.history, '*.jsonl')), key=os.path.getmtime) if self.history else []
        if name:
            files = [f for f in files if os.path.basename(f) == os.path.basename(str(name))]
        elif job_id:
            files = [f for f in files if f.endswith('_%s.jsonl' % job_id)]
        if not files:
            raise PlanningError('no track in %s' % (self.history or 'position_history'))
        return files

    def rpc_leftovers(self, params):
        """planner.leftovers, see the top."""
        current = self.current
        if current.planner == 'slic3r':
            raise PlanningError('planner slic3r: no leftovers planned')
        files = self.track_files(params.get('job_id'), params.get('file'))
        if not params.get('job_id'):
            # (a file, or the latest: with the other files of its job, mower_logic starts a new one when it restarts)
            job = os.path.basename(files[-1])[:-len('.jsonl')].split('_', 1)[-1]
            files = self.track_files(job) if job and not params.get('file') else files[-1:]
        path = files[-1]
        runs = [pts for f in files for blades, pts in read_runs(f) if blades and pts]
        with self.lock:
            data = self.map
        ids = [params['area_id']] if params.get('area_id') is not None else [
            a['id'] for a in active_areas(data) if _props(a).get('mowable') is not False]
        out, plans = {}, {}
        for i in ids:
            keep = []
            try:
                i = self.rpc_plan({'area_id': i}, keep)['area_id']  # (asked for by name or index too: its id)
            except PlanningError as e:
                if params.get('area_id') is not None:
                    raise
                self.loginfo('MowBite Planner: area %s, no leftovers: %s', i, e)  # (the other areas still)
                continue
            cfg = keep[-1].config
            turned = -1.0 if cfg.drive == 'reverse' else 1.0  # (mowing it backs up: the blade behind on the track)
            up = keep[-1].angle if keep[-1].stats.get('drive') == 'slope' else None  # (backing down the lanes)
            mowed = track_mowed(runs, cfg.mower_width, turned * cfg.blade_ahead, turned * cfg.blade_offset, up)
            # (skip_lanes: what the lanes would mow left alone, see fill_field)
            unfilled = fill_field(cfg, keep[-1].lane_region)
            lawn = keep[-1].free_space if unfilled is None else keep[-1].free_space.difference(unfilled)
            left = sum(p.area for p in polygons(lawn.difference(mowed)) if p.area >= LEFT_PIECE)
            got = []
            if left > LEFT_MOST * lawn.area:
                # (not that mow's leftovers: it never mowed this area, or hardly, say it skipped it. Armed it gets no
                # paths either, the leftovers' job mows leftovers only)
                out[i] = {'area_id': i, 'paths': [], 'left_m2': round(left, 3), 'reason': 'most_left', 'message':
                          'most of the area is left (%.1f of %.1f m2): left out, mow it on its own' % (
                              left, lawn.area)}
                plans[i] = None
                continue
            try:
                result = self.rpc_plan({'area_id': i, 'poses': bool(params.get('poses'))}, got, mowed)
                if got:
                    result['duration'] = self.duration(self.estimate(got[-1]))
            except NothingLeft as e:
                result = {'area_id': i, 'paths': [], 'reason': 'nothing_left', 'message': str(e)}
            except PlanningError as e:
                result = {'area_id': i, 'paths': [], 'message': str(e)}
            result['left_m2'] = round(left, 3)
            out[i], plans[i] = result, got[-1] if got else None
        if params.get('arm'):
            with self.set_lock, self.lock:
                same = self.current is current and self.map == data
                if same:
                    self.keep_armed({'until': time.time() + LEFTOVERS_ARMED, 'plans': plans, 'served': False,
                                     'job': None, 'key': self.armed_key(data)})
            if not same:  # (they'd be the old map's or settings')
                raise PlanningError('the map or the settings changed meanwhile: not armed')
            self.loginfo('MowBite Planner: leftovers armed for %.0f s: %s', LEFTOVERS_ARMED, ', '.join(
                '%s %s' % (i, 'nothing' if p is None else '%.2f m2 in %d passes' % (
                    p.stats['leftovers']['area'], p.stats['leftovers']['passes'])) for i, p in plans.items()))
        armed = self.armed
        return {'file': os.path.basename(path), 'files': [os.path.basename(f) for f in files], 'areas': out,
                'armed': 0.0 if not armed else LEFTOVERS_JOB if armed['until'] is None else
                max(0.0, round(armed['until'] - time.time(), 1))}

    def rpc_drive(self, params):
        if not params and self.drive is not None:
            return self.drive
        return self.drive_check(params.get('job_id'), params.get('file'), params.get('area_id'))

    def after_mow(self, job_id, ended=None):
        """What follows a finished mow (ended: when, unix s): its planner.drive and timing, then with leftovers_auto
        what it left."""
        self.track_written(job_id)
        self.drive_after(job_id, ended)
        if not self.current.leftovers_auto:
            return
        try:
            self.leftovers_after(job_id)
        except PlanningError as e:
            self.loginfo('MowBite Planner: no leftovers mowed by itself: %s', e)
        except Exception as e:  # noqa: BLE001 - the leftovers must not take the planner down
            self.logerr('MowBite Planner: the leftovers failed: %s: %s', type(e).__name__, e, exc_info=True)

    def track_written(self, job_id):
        """Waits for the finished job's track to be all in its file (see TRACK_WAIT), as it is after that long."""
        if not job_id or TRACK_WAIT <= 0.0:
            return
        with self.job_change:
            ended = self.job_change.wait_for(lambda: self.job != job_id, TRACK_WAIT)
        if ended:
            time.sleep(TRACK_SETTLE)
        else:
            self.logwarn('MowBite Planner: job %s not ended after %.0f s, its track as it is', job_id, TRACK_WAIT)

    def leftovers_after(self, job_id):
        """leftovers_auto: planner.leftovers of the finished mow, armed. Where its passes mow LEFTOVERS_AUTO_MIN m2 at
        least the mower is sent out for them as soon as it's ready (the mow's JOB_COMPLETE comes before it docks), while
        they're armed and not asked for yet."""
        self.worker.run(AFTER_MOW, self.rpc_leftovers, {'job_id': job_id, 'arm': True}, job='after_mow')
        armed = self.armed
        area = sum(p.stats['leftovers']['area'] for p in armed['plans'].values() if p is not None) if armed else 0.0
        if area < LEFTOVERS_AUTO_MIN:
            if self.armed is armed:
                self.keep_armed(None)
            self.loginfo('MowBite Planner: %.2f m2 of leftovers to mow, under %.2f: the mower stays in', area,
                         LEFTOVERS_AUTO_MIN)
            return
        held = ''
        while self.armed is armed and not armed['served'] and time.time() < (armed['until'] or math.inf):
            if self.mower_ready():
                why = self.held()
                if not why:
                    self.start_mowing()
                    self.loginfo('MowBite Planner: the mower sent out for %.2f m2 of leftovers', area)
                    return
                if not held:
                    self.loginfo('MowBite Planner: the mower not sent out for the leftovers yet, held: %s', why)
                held = why
            time.sleep(READY_POLL)
        self.loginfo('MowBite Planner: the mower not sent out for the leftovers, %s while they were armed',
                     'held (%s)' % held if held else 'not ready')

    def drive_after(self, job_id, ended=None):
        """planner.drive of a finished mow, in the log, and how long it took (time_after)."""
        try:
            self.time_after(job_id, ended)
        except Exception as e:  # noqa: BLE001 - timing must not take the planner down
            self.logerr('MowBite Planner: timing the mow failed: %s: %s', type(e).__name__, e, exc_info=True)
        try:
            self.drive = got = self.worker.run(AFTER_MOW, self.drive_check, job_id, job='after_mow')
        except PlanningError as e:
            self.logwarn('MowBite Planner: drive not checked: %s', e)
            return
        except Exception as e:  # noqa: BLE001 - a check must not take the planner down
            self.logerr('MowBite Planner: checking the drive failed: %s: %s', type(e).__name__, e, exc_info=True)
            return
        for area_id, a in got['areas'].items():
            if a['places']:
                self.logwarn('MowBite Planner: area %s, the body got into the margins at %d places, %.0f cm the most, '
                             'at %s', area_id, len(a['places']), 100 * a['worst'],
                             ', '.join('(%.2f, %.2f) %.0f cm' % (x, y, 100 * m) for x, y, m in a['places'][:5]))
            else:
                self.loginfo('MowBite Planner: area %s, the body kept its margins (%d points)', area_id, a['checked'])

    def drive_check(self, job_id=None, name=None, area_id=None):
        """planner.drive: the track of a job (its position_history file, the latest without one) against the plans'
        body_space: where the body at a driven point (facing the next one, the one before where the plan backs up: drive
        reverse all of it, slope going down the lanes) got into the margins."""
        if self.current.planner == 'slic3r':
            raise PlanningError('planner slic3r: no body checked')
        files = self.track_files(job_id, name)
        track = read_track(files[-1])
        with self.lock:
            data = self.map
        ids = [area_id] if area_id is not None else [a['id'] for a in active_areas(data)]
        docks = (data or {}).get('docking_stations')
        docks = [d.get('position') for d in docks if isinstance(d, dict)] if isinstance(docks, list) else []
        docks = [(p.get('x'), p.get('y')) for p in docks if isinstance(p, dict)]
        docks = [(x, y) for x, y in docks if isinstance(x, (int, float)) and isinstance(y, (int, float))]
        track = [(p, on) for p, on in track if all(math.hypot(p.x - x, p.y - y) > DOCK_REACH for x, y in docks)]
        # OpenMower's own drives (the blade off: to an area, between them, home) go over the lawns and the navigation
        # areas too, the plan's body_space has only the lawns it may drive through
        ground = []
        for a in _areas(data):
            if _props(a).get('type') in ('mow', 'nav'):
                try:
                    ground.append(clean_polygon(_points(a.get('outline'))))
                except ValueError:
                    continue
        ground = union(ground)
        out = {}
        for i in ids:
            keep = []
            try:
                self.rpc_plan({'area_id': i}, keep)
            except PlanningError:
                if area_id is not None:
                    raise
                continue  # (an area with no plan: the others still)
            fp = plan_footprint(keep[-1]) if keep else None
            if fp is None:
                continue
            near = keep[-1].area.buffer(0.5)
            poses = [(p, on) for p, on in track if near.contains(Spot(p.x, p.y))]
            if keep[-1].config.drive == 'reverse':  # (mowing it backs up: facing the point before)
                poses = [(Pose2D(p.x, p.y, p.yaw + math.pi) if on else p, on) for p, on in poses]
            elif keep[-1].stats.get('drive') == 'slope':  # (facing up the lanes all along it: backing down there)
                up = keep[-1].angle
                poses = [(Pose2D(p.x, p.y, p.yaw + math.pi) if on and math.cos(p.yaw - up) < 0.0 else p, on)
                         for p, on in poses]
            outs = fp.out_by([(None, p) for p, _ in poses]) if poses else []
            # (with the blade off out of it: against those too, their real edges robot_width / 2 out from the lines)
            off = [k for k, ((m, _), (_, on)) in enumerate(zip(outs, poses)) if m > DRIVE_OFF and not on]
            if off and not ground.is_empty:
                cfg = keep[-1].config
                wide = Footprint(fp.space.union(ground.buffer(fp.half - cfg.edge_margin)), cfg.body_outline,
                                 cfg.spin_margin)
                for k, got in zip(off, wide.out_by([(None, poses[k][0]) for k in off])):
                    outs[k] = got
            places, run = [], None
            for (p, _), (m, _) in zip(poses, outs):
                if m > DRIVE_OFF:  # (a place: points one after the other out there, where it's the furthest)
                    if run is None or m > run[2]:
                        run = [round(p.x, 2), round(p.y, 2), m]
                elif run is not None:
                    places.append(run)
                    run = None
            if run is not None:
                places.append(run)
            out[i] = {'checked': len(poses), 'places': places, 'worst': max([m for _, _, m in places], default=0.0)}
        return {'file': os.path.basename(files[-1]), 'areas': out}

    def set_map(self, text):
        """The robot's map as it sent it (OpenMower's JSON object). One not readable isn't planned with, nor the one
        before it (unusable) till a readable one comes."""
        try:
            data = json.loads(text)
            if not isinstance(data, dict):
                raise ValueError('not an object')
        except ValueError as e:
            self.logwarn('MowBite Planner: map not readable, nothing planned with a map till it is: %s', e)
            with self.lock:
                self.map_error = 'the map isn\'t readable: %s' % e
            self.worker.stale()
            return
        with self.lock:
            changed, self.map, self.map_text = data != self.map, data, text
            self.map_error = ''
            if changed:
                self.armed_changed()  # (the leftovers were another map's)
            self.map_came.set()
        if changed:
            self.plan_ahead_later()

    def plan_ahead_later(self, changed=True):
        """Every active mowing area planned ahead PLAN_AHEAD_DELAY s from now (from then on, after another change).
        changed: what's planned changed, a planning ahead that runs stops (Worker.stale)."""
        if changed:
            self.worker.stale()
        with self.ahead_lock:
            if self.ahead is not None:
                self.ahead.cancel()
            self.ahead_count += 1
            self.ahead = threading.Timer(PLAN_AHEAD_DELAY, self.plan_ahead, (self.ahead_count,))
            self.ahead.daemon = True
            self.ahead.start()

    def plan_ahead(self, count=None):
        """Every active mowing area of the map planned like planner.plan would (the app's preview), kept for it. Not
        with planner slic3r (it plans at once, nothing kept). One run at a time, its areas one after the other in the
        worker (each its plans in the processes, see workers): after another change (count isn't the latest any more)
        it begins no more of them, the next one plans them as they are then."""
        with self.ahead_run:
            if self.current.planner == 'slic3r':
                return
            reason = self.unusable()
            if reason:  # (planned ahead once it can be)
                self.loginfo('MowBite Planner: not planned ahead: %s', reason)
                return
            areas = [a for a in active_areas(self.current_map()) if _props(a).get('mowable') is not False]
            t0 = time.perf_counter()

            def ahead(area):
                return (count is None or count == self.ahead_count) and self.ahead_plan(area['id'])

            done = sum(ahead(area) for area in areas)
            if count is not None and count != self.ahead_count:
                return
            if areas:
                self.loginfo('MowBite Planner: %d areas planned ahead in %.1f s', done, time.perf_counter() - t0)

    def ahead_plan(self, area_id):
        """area_id planned in the worker (with planner.plan's ahead for it: the same request)."""
        params = {'area_id': area_id}
        try:
            got = self.worker.run(AHEAD, self.rpc_plan, params, area_id=area_id, same=same_request(params))
            if got.get('duration'):
                self.loginfo('MowBite Planner: area %s takes about %.0f min', area_id, got['duration']['min'])
            return True
        except PlanningError as e:
            self.logwarn('MowBite Planner: area %s not planned ahead: %s', area_id, e)
        except Exception as e:  # noqa: BLE001 - planning ahead must not take the planner down
            self.logerr('MowBite Planner: planning area %s ahead failed: %s: %s', area_id, type(e).__name__, e,
                        exc_info=True)
        return False

    def plan_coverage(self, boundary, obstacles, cfg, start=None, end=None, drivable=(), keep_out=(), margins=(),
                      reach_over=(), unmowed=(), bodies=()):
        """plan_coverage(), the plans last asked for kept, in memory (CACHED_PLANS, CACHED_MB) and in plans_dir: the
        plan only depends on what it's given and the planner's code, the same again (to the last bit) is the same
        plan."""
        return self.kept_plan(boundary, obstacles, cfg, start, end, drivable, keep_out, margins, reach_over,
                              unmowed, bodies=bodies)[0]

    def driven(self, cfg, boundary=(), bodies=()):
        """(cfg, []) or, where drive slope or reverse or back_out is ignored here (drive_ignored, back_out_ignored) or
        recorded_loop for the area of boundary (recorded_loop_ignored; bodies: plan_coverage's), (cfg with drive
        forward, back_out or recorded_loop off, why for the plan's warnings)."""
        drive = self.drive_ignored() if cfg.drive != 'forward' else ''
        out = self.back_out_ignored() if cfg.back_out else ''
        loop = self.recorded_loop_ignored(body_known(bodies, boundary)) if cfg.recorded_loop else ''
        if not drive and not out and not loop:
            return cfg, []
        told = ['; '.join(['drive %s ignored: %s' % (cfg.drive, drive)] * bool(drive) +
                          ['back_out ignored: %s' % out] * bool(out))] * bool(drive or out) + \
            ['recorded_loop ignored: %s' % loop] * bool(loop)
        self.logwarn_throttle(60, 'MowBite Planner: ' + '; '.join(told))
        return cfg.with_overrides(dict({'drive': 'forward'} if drive else {}, **{'back_out': False} if out else {},
                                       **{'recorded_loop': False} if loop else {})), told

    @staticmethod
    def told(plan, why):
        """plan, with why (driven's) last among its warnings."""
        return dataclasses.replace(plan, warnings=list(plan.warnings) + list(why)) if why else plan

    def left_plan(self, boundary, obstacles, cfg, start, end, drivable, keep_out, margins=(), *args, **kw):
        """plan_coverage of what a mow left (not kept), the robot's bounds, drive and arrives as kept_plan takes
        them."""
        cfg = self.bound(cfg)
        cfg, why = self.driven(cfg, boundary, kw.get('bodies', ()))
        arrives = self.arrives(boundary, obstacles, cfg, drivable, keep_out, kw.get('reach_over', ()),
                               kw.get('unmowed', ()), margins, kw.get('bodies', ()))
        return self.told(self.checked(plan_coverage(boundary, obstacles, cfg, start, end, drivable, keep_out, margins,
                                                    *args, arrives=arrives, **kw)), why)

    def kept_plan(self, boundary, obstacles, cfg, start=None, end=None, drivable=(), keep_out=(), margins=(),
                  reach_over=(), unmowed=(), from_start=False, bodies=()):
        """(plan, whether it was kept from before), see plan_coverage; with remote_planning planned on the house
        computer where it can (remote_plan). The robot's bounds (bound). drive slope or reverse forwards where it's
        ignored (driven), said in its warnings. With the adapter's arrives every section starts where the robot gets to
        it (its key another), from_start: the first from start (x, y, yaw; arrives.start). bodies: plan_coverage's, the
        body a ring was recorded with. self.local: kept, the plan's key and where it was planned (on: mower or
        house)."""
        cfg = self.bound(cfg)
        cfg, why = self.driven(cfg, boundary, bodies)
        if cfg.perimeter_direction != 'random' and cfg.finished_mows:  # (counts only there: the plan kept stays one)
            cfg = cfg.with_overrides({'finished_mows': 0})
        # the obstacles in one order, every way there (the request, the map) gives the same, each with its margin
        margins = list(margins) + [None] * (len(obstacles) + len(keep_out) - len(margins))
        pairs = sorted(zip(canon(obstacles), canon(margins)), key=lambda pair: pair[0])
        obstacles = [o for o, _ in pairs]
        margins = tuple(m for _, m in pairs) + canon(margins[len(obstacles):])
        args = (canon(boundary), obstacles, cfg, canon(start), canon(end), canon(drivable), canon(keep_out), margins)
        reach_over, unmowed, bodies = canon(reach_over), canon(unmowed), tuple(sorted(canon(bodies)))
        arrives = self.arrives(args[0], args[1], cfg, args[5], args[6], reach_over, unmowed, args[7], bodies)
        form = request_json(args, reach_over, unmowed, bodies)
        if arrives is not None:
            form = dict(form, arrives=getattr(arrives, 'key', True))
            if from_start and args[3] is not None and len(args[3]) >= 3:
                arrives.start = args[3]
                form['from_start'] = True
        key = self.local.key = plan_key(self.code, form)
        with self.plans_lock:
            got = self.plans.get(key)
            if got is not None:
                self.plans.move_to_end(key)
            elif key in self.refused:
                kind, message, word = self.refused[key]
                raise kind(message, word=word)
            else:
                busy = self.planning.setdefault(key, threading.Lock())
        if got is not None:
            self.used(key)
            self.local.kept, self.local.on = True, got[1]
            return self.told(got[0], why), True
        with busy:  # (the same asked for meanwhile, planning ahead and a mow: it waits for this one's)
            try:
                with self.plans_lock:
                    got = self.plans.get(key)
                if got is None:
                    got = self.read_plan(key)
                kept = self.local.kept = got is not None
                if got is None:
                    area_id = self.area_of(boundary)
                    self.progress.begin(area_id, key)
                    try:
                        plan = self.remote_plan(key, form, args, reach_over, unmowed, area_id, bodies) \
                            if arrives is None else None
                        got = (plan, 'house') if plan is not None else (plan_coverage(
                            *args, reach_over=reach_over, unmowed=unmowed, arrives=arrives,
                            progress=self.progress.step, **({'bodies': bodies} if bodies else {})), 'mower')
                        got = (self.checked(got[0]), got[1])
                    except Stopped:
                        self.progress.end(stopped=True)
                        raise
                    except Exception as e:
                        self.progress.end(str(e))
                        if isinstance(e, PlanningError):  # (a no from the request alone, never a crash)
                            with self.plans_lock:
                                self.refused[key] = (type(e), str(e), e.word)
                                while len(self.refused) > REFUSED:
                                    self.refused.popitem(last=False)
                        raise
                    self.progress.end()
                    got += (self.write_plan(key, got),)
                else:
                    self.used(key)
                self.local.on = got[1]
                with self.plans_lock:
                    self.plans[key] = got
                    self.plans.move_to_end(key)
                    gone = self.shed(key)
            finally:
                with self.plans_lock:
                    self.planning.pop(key, None)
        if gone:
            self.loginfo('MowBite Planner: kept plans out of memory: %d, they took more than %.0f MB', gone, CACHED_MB)
        return self.told(got[0], why), kept

    def shed(self, keep):
        """(plans_lock held) The plans in memory down to CACHED_PLANS and CACHED_MB, the ones asked for longest ago
        first: over the count not the job's (pin), over the room those too where plans_dir has them (read from there
        again), never keep's. How many went for the room."""
        for old in [k for k in self.plans if k not in self.pinned][:max(0, len(self.plans) - CACHED_PLANS)]:
            del self.plans[old]
        room, gone = PLAN_ROOM * sum(got[2] for got in self.plans.values()), 0
        for old in list(self.plans):
            if room <= CACHED_MB * 1e6:
                break
            if old != keep and (old not in self.pinned or
                                (self.plans_dir and os.path.exists(self.plan_file(old)))):
                room -= PLAN_ROOM * self.plans.pop(old)[2]
                gone += 1
        return gone

    def used(self, key):
        """The plan of key was asked for again: its file in plans_dir the last used (the oldest go first, write_plan
        and a robot's own tidying)."""
        if self.plans_dir:
            try:
                os.utime(self.plan_file(key))
            except OSError:
                pass

    def pin(self, key):
        """The plan of key is the job's (mower_logic got it): kept, in memory and in plans_dir, till the job ends
        (JOB_COMPLETE or JOB_RESET, unpin), over a restart too (in PINNED_FILE). Resuming the job finds it, even where
        it came from the house computer and that's gone."""
        with self.plans_lock:
            if key in self.pinned:
                return
            self.pinned.add(key)
            pinned = sorted(self.pinned)
        self.write_pinned(pinned)

    def unpin(self):
        with self.plans_lock:
            if not self.pinned:
                return
            self.pinned = set()
        self.write_pinned([])

    def write_pinned(self, keys):
        if not self.plans_dir:
            return
        try:
            os.makedirs(self.plans_dir, exist_ok=True)
            path = os.path.join(self.plans_dir, PINNED_FILE)
            with open(path + '.tmp', 'w') as f:
                json.dump({'keys': keys}, f)
            os.replace(path + '.tmp', path)
        except OSError as e:
            self.logwarn('MowBite Planner: the job\'s plans not noted in %s: %s', self.plans_dir, e)

    def read_pinned(self):
        try:
            with open(os.path.join(self.plans_dir, PINNED_FILE)) as f:
                keys = json.load(f)['keys']
            return {k for k in keys if isinstance(k, str)}
        except (OSError, ValueError, KeyError, TypeError):
            return set()

    def area_of(self, boundary):
        """The id of the map's mowing area with this outline (None: none)."""
        with self.lock:
            data = self.map
        index = match_area(data, boundary) if data else None
        return mow_areas(data)[index].get('id') if index is not None else None

    def plan_file(self, key):
        return os.path.join(self.plans_dir, '%s-%s.pkl' % (self.build, key))

    def drop_old_plans(self):
        """The kept plans of another planner code or of other versions of what they depend on (remote.stack) go, and
        what a write cut off left, the job's too."""
        if not self.plans_dir:
            return
        own = self.build + '-'
        for path in glob.glob(os.path.join(self.plans_dir, '*.pkl')) + glob.glob(os.path.join(self.plans_dir, '*.tmp')):
            if not (path.endswith('.pkl') and os.path.basename(path).startswith(own)):
                try:
                    os.remove(path)
                except OSError:
                    pass

    def read_plan(self, key):
        """(plan, where it was planned, its pickle's bytes) kept in plans_dir, None: none."""
        if not self.plans_dir:
            return None
        try:
            with open(self.plan_file(key), 'rb') as f:
                data = f.read()
            kept = pickle.loads(data)  # (written here only: write_plan, never one of the house computer's)
            return kept['plan'], kept['on'], len(data)
        except FileNotFoundError:
            return None
        except Exception as e:  # noqa: BLE001 - a broken or old file is planned again
            self.logwarn('MowBite Planner: kept plan %s not readable: %s', key, e)
            return None

    def armed_plan(self, job, mowing=True):
        """planner.leftovers armed, for the area of job (mapfile.AreaJob, None: an outline of no area): (True, its
        leftovers' plan, None where it left nothing or they're of other areas), (False, None) where they aren't. mowing:
        the robot asks for a job of its own (not a preview): that job's from now on (bound to its id, LEFTOVERS_JOB s
        without one, see armed_job; the same again while it runs, it asks again when it goes on after a break or a
        restart)."""
        armed = self.armed
        if armed is not None and armed['until'] is not None and time.time() > armed['until']:
            armed = None
            self.keep_armed(None)
        if armed is None or job is None:
            return False, None
        if mowing and not armed['served']:
            job_id = self.job or None
            armed.update(served=True, job=job_id, until=None if job_id else time.time() + LEFTOVERS_JOB)
            self.keep_armed(armed)
        return True, armed['plans'].get(job.area_id)

    def armed_job(self, job_id):
        """A job runs (job_id, STATE): the leftovers served without a job's id are that job's from now on, the ones of
        another job gone."""
        armed = self.armed
        if armed is None or not armed['served'] or not job_id:
            return
        if armed['until'] is not None and time.time() > armed['until']:
            self.keep_armed(None)
        elif armed['job'] is None:
            armed.update(job=job_id, until=None)
            self.keep_armed(armed)
        elif armed['job'] != job_id:
            self.loginfo('MowBite Planner: the leftovers armed gone, job %s isn\'t theirs (%s)', job_id, armed['job'])
            self.keep_armed(None)

    def armed_changed(self):
        """The map changed (set_map): the leftovers armed gone where they're another map's (or settings')."""
        armed = self.armed
        if armed is not None and armed.get('key') != self.armed_key(self.map):
            self.keep_armed(None)

    def armed_key(self, data):
        """What the leftovers armed are of: the map data and the settings."""
        current = self.current
        return hashlib.sha1(json.dumps({'map': data, 'width': self.launch_width, 'launch': self.launch,
                                        'stored': current.stored, 'given': self.given() if self.given_by else {}},
                                       sort_keys=True, default=repr).encode()).hexdigest()

    def keep_armed(self, armed):
        """armed (None: none) from now on, kept in plans_dir (ARMED_FILE) over a restart."""
        self.armed = armed
        if not self.plans_dir:
            return
        path = os.path.join(self.plans_dir, ARMED_FILE)
        try:
            if armed is None:
                if os.path.exists(path):
                    os.remove(path)
                return
            os.makedirs(self.plans_dir, exist_ok=True)
            with open(path + '.tmp', 'wb') as f:
                pickle.dump({'build': self.build, 'armed': armed}, f, protocol=4)
            os.replace(path + '.tmp', path)
        except Exception as e:  # noqa: BLE001 - they're still armed in memory
            self.logwarn('MowBite Planner: the leftovers armed not kept in %s: %s: %s', self.plans_dir,
                         type(e).__name__, e)

    def read_armed(self):
        """The leftovers armed before a restart (keep_armed), None: none (or another planner code's, or ended)."""
        if not self.plans_dir:
            return None
        try:
            with open(os.path.join(self.plans_dir, ARMED_FILE), 'rb') as f:
                kept = pickle.load(f)  # (written here only: keep_armed)
            armed = kept['armed'] if kept.get('build') == self.build else None
        except FileNotFoundError:
            return None
        except Exception as e:  # noqa: BLE001 - a broken or old file: none
            self.logwarn('MowBite Planner: the leftovers armed before not readable: %s: %s', type(e).__name__, e)
            return None
        if armed is None or armed['until'] is not None and time.time() > armed['until']:
            return None
        self.loginfo('MowBite Planner: the leftovers armed before the restart still are (job %s)', armed['job'])
        return armed

    def serve(self, area_id, plan):
        """plan went to the robot for area_id (None: an outline of no area): the next finished job's (time_after)."""
        try:
            self.served.append((time.time(), area_id, self.estimate(plan)))
        except Exception as e:  # noqa: BLE001 - the estimate must not keep the plan from the robot
            self.logwarn('MowBite Planner: no time estimate: %s: %s', type(e).__name__, e, exc_info=True)

    def write_plan(self, key, got):
        """(plan, where it was planned) kept in plans_dir, the DISK_PLANS last used (used) and the job's (pin). Its
        pickle's bytes, written or not (0: it has none)."""
        try:
            data = pickle.dumps({'plan': got[0], 'on': got[1]}, protocol=4)
        except Exception as e:  # noqa: BLE001 - a plan without a pickle is still the answer
            self.logwarn('MowBite Planner: plan kept in memory only, no pickle of it: %s: %s', type(e).__name__, e)
            return 0
        if not self.plans_dir:
            return len(data)
        try:
            os.makedirs(self.plans_dir, exist_ok=True)
            path = self.plan_file(key)
            with open(path + '.tmp', 'wb') as f:
                f.write(data)
            os.replace(path + '.tmp', path)
            with self.plans_lock:
                pinned = {self.plan_file(k) for k in self.pinned}
            files = [f for f in sorted(glob.glob(os.path.join(self.plans_dir, '*.pkl')), key=os.path.getmtime)
                     if f not in pinned]
            for old in files[:-DISK_PLANS]:
                os.remove(old)
        except OSError as e:
            self.logwarn('MowBite Planner: plan not kept in %s: %s', self.plans_dir, e)
        return len(data)


def read_track(path):
    """The driven points of a position_history file (OpenMower's: new_segment and append_points lines of [x, y]),
    each facing the next one (DRIVE_STEP apart at least, the last one as the one before), with whether the blade ran
    there (its new_segment's blades): [(Pose2D, blade), ...]."""
    pts, blade = [], False
    with open(path) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if not isinstance(rec, dict):
                continue
            if rec.get('type') == 'new_segment':
                blade = bool((rec.get('attributes') or {}).get('blades'))
            for xy in rec.get('points') or []:
                if isinstance(xy, (list, tuple)) and len(xy) >= 2 and (
                        not pts or math.hypot(xy[0] - pts[-1][0], xy[1] - pts[-1][1]) >= DRIVE_STEP):
                    pts.append((float(xy[0]), float(xy[1]), blade))
    out = [(Pose2D(x, y, math.atan2(b[1] - y, b[0] - x)), on) for (x, y, on), b in zip(pts, pts[1:])]
    return out + [(Pose2D(pts[-1][0], pts[-1][1], out[-1][0].yaw), pts[-1][2])] if out else []


def read_runs(path):
    """The segments of a position_history file: [(whether the blade ran, its driven points [(x, y), ...]), ...]."""
    runs = []
    with open(path) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if not isinstance(rec, dict):
                continue
            if rec.get('type') == 'new_segment':
                runs.append((bool((rec.get('attributes') or {}).get('blades')), []))
            for xy in rec.get('points') or []:
                if runs and isinstance(xy, (list, tuple)) and len(xy) >= 2:
                    runs[-1][1].append((float(xy[0]), float(xy[1])))
    return runs


def track_start(path):
    """When a position_history file began (unix s, its name's first part)."""
    return float(os.path.basename(path).split('_', 1)[0])


def mowing_time(paths, ended=None):
    """s the blade ran in a job's position_history files (one path or several, the first first; their new_segment
    lines with started_at and blades), the runs added up: each to the next segment, the last one of a file to the next
    file's first (its last write at the latest), the last one of all to ended (unix s; None: not counted); None without
    a run that ended."""
    runs = []
    paths = [paths] if isinstance(paths, str) else list(paths)
    for k, path in enumerate(paths):
        segments = []
        with open(path) as f:
            for line in f:
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                if isinstance(rec, dict) and rec.get('type') == 'new_segment' and \
                        isinstance(rec.get('started_at'), (int, float)) and not isinstance(rec['started_at'], bool):
                    segments.append((float(rec['started_at']), bool((rec.get('attributes') or {}).get('blades'))))
        runs += [b - a for (a, blades), (b, _) in zip(segments, segments[1:]) if blades]
        if segments and segments[-1][1]:  # (the blade still ran as the file ended)
            if k + 1 < len(paths):
                end = min(os.path.getmtime(path), track_start(paths[k + 1]))
            else:
                end = ended
            if end is not None:
                runs.append(max(0.0, end - segments[-1][0]))
    return sum(runs) if runs else None


def run_calib(path, interim=False, timeout=CALIB_EVALUATE, cut=False):
    """mowbite_planner.calib on the recording at path in a process of its own (niced, timeout s at most; interim, cut:
    see calib.evaluate): its result, else {'error': why}."""
    command = [sys.executable, '-m', 'mowbite_planner.calib'] + (['--interim'] if interim else []) + \
        (['--cut'] if cut else []) + [path]
    if shutil.which('nice'):
        command = ['nice', '-n', str(CALIB_NICE)] + command
    roots = [os.path.dirname(p) for p in mowbite_planner.__path__ if os.path.exists(os.path.join(p, 'calib.py'))]
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(roots + [os.environ.get('PYTHONPATH', '')]).rstrip(os.pathsep))
    try:
        done = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout,
                              env=env)
    except subprocess.TimeoutExpired:
        return {'error': 'the evaluation took over %.0f s' % timeout}
    except OSError as e:
        return {'error': 'no evaluation: %s' % e}
    try:
        result = json.loads(done.stdout.decode() or 'null')
    except ValueError:
        result = None
    if not isinstance(result, dict):
        lines = done.stderr.decode(errors='replace').strip().splitlines()
        return {'error': 'the evaluation failed: %s' % (lines[-1] if lines else 'exit code %d' % done.returncode)}
    return result
