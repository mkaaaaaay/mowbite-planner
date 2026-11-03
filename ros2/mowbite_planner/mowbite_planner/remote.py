"""Planning on a computer in the house (the setting remote_planning, house.py there): what goes between the mower
and the house. A request: plan_coverage's arguments (request_json), its key and the planner's code; the house plans it
only with the same code and key. A plan: its form FORMAT, all of it but the geometry and the config, the floats as they
are; the mower works those out itself (planned_with, plan_geometry) and drives the plan only where its own checks find
nothing (refused). Read strictly: sizes, nesting, finite numbers, every field as it must be. Never pickle."""

import dataclasses
import hashlib
import json
import math
import os
import platform
import zlib
from typing import Any, Dict, List, Optional, Tuple

from . import backing
from .backing import CORNER_STEP
from .calibread import _constant, _float, _int
from .config import LANE, PERIMETER, TRANSIT, TURN, CoveragePlan, Lane, PlannerConfig, PlanningError, SegmentInfo
from .levels import ring_levels
from .mathutil import Pose2D, angle_diff
from .planner import _fixed, _narrow, _recorded_lines, _spacings, _turning
from .report import rules
from .space import build_space, margins_of, recorded_space

FORMAT = 'mowbite-plan/1'
# over MQTT, the mower's own broker without a prefix: the topics under TOPIC (hello, want, request, progress/<id>,
# answer/<id>, answer/<id>/<part>), s between the house's hellos, bytes of an answer's part at most (the broker drops a
# message of about 2 MB, see export.MAX_ANSWER)
TOPIC = 'mowbite/planner/remote/'
HELLO_EVERY = 30.0
PART = 1000000
# bytes packed and as JSON at most, how deep it nests at most, characters of a string
MAX_PACKED = 16 * 1024 * 1024
MAX_TEXT = 48 * 1024 * 1024
MAX_DEPTH = 12
MAX_STRING = 4096
KINDS = (PERIMETER, LANE, TURN, TRANSIT)
# rad, a pose faces the way it's driven to the next one (test/planrules.py)
FACING = math.radians(6.0)
# the plan's fields in its form; the rest the mower works out: lane_spacing, config and the geometry (plan_geometry)
SENT = ('poses', 'segments', 'angle', 'perimeter_loops', 'lanes', 'stats', 'warnings', 'angles', 'section_starts',
        'cells', 'origins', 'recorded_loops')


class RemoteError(ValueError):
    """What came from the other side isn't taken, why in the message."""


def canon(value):
    """Points, lists of them and so on as tuples of floats: the same whichever way they came."""
    if isinstance(value, (list, tuple)):
        return tuple(canon(v) for v in value)
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    return value


def code_version() -> str:
    """A digest of the planner's code: a plan counts for that code only. (Built with catkin, the package first found
    is a stand-in that only points to the code, its __path__ has both.)"""
    import mowbite_planner
    digest = hashlib.sha1()
    for root in mowbite_planner.__path__:
        for name in sorted(os.listdir(root)):
            if name.endswith('.py'):
                with open(os.path.join(root, name), 'rb') as f:
                    digest.update(name.encode() + b'\0' + f.read())
    return digest.hexdigest()


def stack() -> Dict[str, str]:
    """What the plans depend on besides the code: Python's, shapely's, GEOS' and numpy's versions."""
    import numpy
    import shapely
    return {'python': platform.python_version(), 'shapely': shapely.__version__,
            'geos': str(getattr(shapely, 'geos_version_string', '')), 'numpy': numpy.__version__}


def plan_key(code: str, form: Dict[str, Any]) -> str:
    """The key of a plan: the code and its request's form (request_json), all it's planned from."""
    return hashlib.sha1((code + json.dumps(form, sort_keys=True, separators=(',', ':'))).encode()).hexdigest()


def pack(form: Any) -> bytes:
    """form as JSON, zlib'd. ValueError where a number isn't finite."""
    return zlib.compress(json.dumps(form, allow_nan=False, separators=(',', ':')).encode(), 6)


def unpack(data: bytes) -> Any:
    """What pack packed, strictly: RemoteError where it's more than MAX_PACKED or MAX_TEXT bytes, else see loads."""
    if len(data) > MAX_PACKED:
        raise RemoteError(f'more than {MAX_PACKED} bytes')
    stream = zlib.decompressobj()
    try:
        text = stream.decompress(data, MAX_TEXT + 1)
    except zlib.error as e:
        raise RemoteError(f'not zlib: {e}') from None
    if len(text) > MAX_TEXT or stream.unconsumed_tail:
        raise RemoteError(f'more than {MAX_TEXT} bytes unpacked')
    if not stream.eof:
        raise RemoteError('cut off')
    return loads(text)


def loads(text: bytes) -> Any:
    """JSON, strictly: RemoteError where it isn't JSON, has a number that isn't finite (or an integer of more than 16
    digits) or nests deeper than MAX_DEPTH."""
    try:
        value = json.loads(text, parse_float=_float, parse_int=_int, parse_constant=_constant)
    except (ValueError, RecursionError) as e:
        raise RemoteError(f'not JSON: {e}') from None
    todo = [(value, 0)]
    while todo:
        v, depth = todo.pop()
        if isinstance(v, (dict, list)):
            if depth >= MAX_DEPTH:
                raise RemoteError(f'nested deeper than {MAX_DEPTH}')
            todo.extend((x, depth + 1) for x in (v.values() if isinstance(v, dict) else v))
    return value


# -- the request ---------------------------------------------------------------------------------------------------

def request_json(args: tuple, reach_over: tuple = (), unmowed: tuple = ()) -> Dict[str, Any]:
    """The request's form: plan_coverage's arguments (args its first eight, canon'd, the obstacles in one order with
    their margins, as the ROS 1 node's kept_plan has them)."""
    boundary, obstacles, cfg, start, end, drivable, keep_out, margins = args
    return {'boundary': boundary, 'obstacles': obstacles, 'start': start, 'end': end, 'drivable': drivable,
            'keep_out': keep_out, 'margins': margins, 'reach_over': reach_over, 'unmowed': unmowed,
            'config': {f.name: getattr(cfg, f.name) for f in dataclasses.fields(cfg)}}


def request_from(form: Any) -> Tuple[tuple, tuple, tuple]:
    """(args, reach_over, unmowed) of a request's form, as request_json takes them. RemoteError where it isn't one."""
    form = _object(form, 'the request')
    rings = {name: canon(_nested(form.get(name), name, depth)) for name, depth in (
        ('boundary', 2), ('obstacles', 3), ('drivable', 3), ('keep_out', 3), ('reach_over', 3), ('unmowed', 3))}
    ends = {name: None if form.get(name) is None else canon(_nested(form[name], name, 1)) for name in ('start', 'end')}
    margins = tuple(None if m is None else _number(m, 'margins') for m in _list(form.get('margins'), 'margins'))
    args = (rings['boundary'], rings['obstacles'], _config(form.get('config')), ends['start'], ends['end'],
            rings['drivable'], rings['keep_out'], margins)
    return args, rings['reach_over'], rings['unmowed']


def _config(values: Any) -> PlannerConfig:
    values = dict(_object(values, 'config'))
    fields = {f.name for f in dataclasses.fields(PlannerConfig)}
    if set(values) - fields:
        raise RemoteError(f'config: unknown settings {sorted(set(values) - fields)}')
    for name, v in values.items():
        if name == 'turn_types':
            values[name] = tuple(_string(t, name) for t in _list(v, name))
        elif name == 'robot_outline' and v is not None:
            _nested(v, name, 2)
            values[name] = tuple(tuple(p) for p in v)
        elif v is not None and not isinstance(v, (bool, int, float, str)):
            raise RemoteError(f'config: {name} is no number, string or bool')
        elif isinstance(v, str):
            _string(v, name)
    return PlannerConfig(**values)


# -- the plan ------------------------------------------------------------------------------------------------------

def plan_json(plan: CoveragePlan, cfg: PlannerConfig) -> Dict[str, Any]:
    """The plan's form (FORMAT) with the lane spacing lane_spacing_mode auto took (cfg: what it was planned for).
    PlanningError where its config isn't one planned_with gives."""
    return {'format': FORMAT, 'spacing': _spacing_of(plan, cfg), 'poses': [v for p in plan.poses for v in p],
            'segments': [[s.kind, s.start, s.end, s.reverse, s.maneuver, s.section, s.source, s.blade]
                         for s in plan.segments],
            'angle': plan.angle, 'perimeter_loops': [[v for p in loop for v in p] for loop in plan.perimeter_loops],
            'lanes': [v for a, b in plan.lanes for v in (a[0], a[1], b[0], b[1])], 'stats': plan.stats,
            'warnings': plan.warnings, 'angles': plan.angles, 'section_starts': plan.section_starts,
            'cells': [[angle, [[[lane.row, lane.x0, lane.x1, lane.y,
                                 [[x0, x1, [v for p in way for v in p]] for x0, x1, way in lane.bends]]
                                for lane in cell] for cell in cells]] for angle, cells in plan.cells],
            'origins': plan.origins, 'recorded_loops': plan.recorded_loops}


def planned_with(cfg: PlannerConfig, spacing: Optional[float] = None) -> Tuple[PlannerConfig, PlannerConfig]:
    """(the config of plan_coverage's plan with cfg, the one its geometry is planned with) where it took the lane
    spacing spacing, one of spacings_tried(cfg); PlanningError where it's none of them."""
    used, own, tried = _asked(cfg)
    if spacing not in tried:
        raise PlanningError(f'the plan takes none of the lane spacings {tried}: {spacing!r}')
    used = _turning(_fixed(used, spacing) if spacing is not None else used)
    return own or used, used


def spacings_tried(cfg: PlannerConfig) -> List[Optional[float]]:
    """The lane spacings plan_coverage's plan with cfg may take: lane_spacing_mode auto's, [None] for fixed."""
    return _asked(cfg)[2]


def _asked(cfg: PlannerConfig) -> Tuple[PlannerConfig, Optional[PlannerConfig], List[Optional[float]]]:
    """(what plan_coverage plans with before the lane spacing: drive slope and reverse with backing's settings,
    forwards where they can't be planned; the config its plan has where that's another; spacings_tried)."""
    cfg.validate()
    if cfg.drive != 'forward' and backing.unavailable(cfg):
        cfg = cfg.with_overrides({'drive': 'forward'})
    used, own = backing.configs(cfg) if cfg.drive != 'forward' else (cfg, None)
    if used.lane_spacing_mode != 'auto':
        return used, own, [None]
    hi, widest = used.spacing_range[1], used.spacing_asked[1]
    return used, own, _spacings(used) + ([widest] if widest > hi + 1e-9 else [])


def plan_geometry(boundary, obstacles, cfg: PlannerConfig, drivable=(), keep_out=(), margins=(), reach_over=(),
                  unmowed=(), cells=()) -> Dict[str, Any]:
    """The geometry planner._plan_coverage plans on with cfg (planned_with's second) as a CoveragePlan's fields, for a
    plan whose poses come from elsewhere; cells: the plan's."""
    reach_over = list(reach_over) + list(unmowed)
    margin_of = margins_of(boundary, obstacles, drivable, keep_out, reach_over, margins, cfg)
    space, footprint = build_space(boundary, obstacles, drivable, keep_out, reach_over, margin_of, cfg, False,
                                   unmowed)
    rings_of = ring_levels(space, cfg, None, footprint)
    recorded = None
    if footprint is not None and cfg.edges == 'recorded' and rings_of.owners is not None:
        recorded = recorded_space(*_recorded_lines(cfg, [boundary] + list(drivable) + reach_over,
                                                   list(obstacles) + list(keep_out)), margin_of, cfg)
    return dict(area=space.area, free_space=space.free, robot_space=space.drive_space,
                lane_region=rings_of.lane_region, remaining=rings_of.remaining, transit_space=space.transit_space,
                body_space=space.body_space, levels=rings_of.levels, narrow=_narrow(rings_of, cells, cfg),
                recorded_space=recorded, unmowed=space.unmowed)


def _spacing_of(plan: CoveragePlan, cfg: PlannerConfig) -> Optional[float]:
    for spacing in spacings_tried(cfg):
        own, used = planned_with(cfg, spacing)
        if own == plan.config and used.lane_spacing == plan.lane_spacing:
            return spacing
    raise PlanningError('the plan has a config planned_with doesn\'t give')


def plan_from(form: Any, args: tuple, reach_over: tuple = (), unmowed: tuple = ()) -> CoveragePlan:
    """The plan of its form for the request args (as plan_key takes them), its config and geometry worked out here.
    RemoteError where it isn't one. Not checked yet: see refused."""
    form = _object(form, 'the plan')
    if form.get('format') != FORMAT:
        raise RemoteError(f'not a {FORMAT} plan')
    spacing = None if form.get('spacing') is None else _number(form['spacing'], 'spacing')
    try:
        own, used = planned_with(args[2], spacing)
    except PlanningError as e:
        raise RemoteError(str(e)) from None
    poses = [Pose2D(*p) for p in _flat(form.get('poses'), 'poses', 3)]
    segments = [SegmentInfo(*_segment(s)) for s in _list(form.get('segments'), 'segments')]
    loops = [_flat(loop, 'perimeter_loops', 2) for loop in _list(form.get('perimeter_loops'), 'perimeter_loops')]
    lanes = [((a, b), (c, d)) for a, b, c, d in _flat(form.get('lanes'), 'lanes', 4)]
    cells = [_cells(c) for c in _list(form.get('cells'), 'cells')]
    starts = [_index(i, 'section_starts') for i in _list(form.get('section_starts'), 'section_starts')]
    sent = dict(poses=poses, segments=segments, angle=_number(form.get('angle'), 'angle'), perimeter_loops=loops,
                lanes=lanes, stats=_object(form.get('stats'), 'stats'), warnings=_strings(form, 'warnings'),
                angles=[_number(a, 'angles') for a in _list(form.get('angles'), 'angles')], section_starts=starts,
                cells=cells, origins=_object(form.get('origins'), 'origins'),
                recorded_loops=_strings(form, 'recorded_loops'))
    boundary, obstacles, _, _, _, drivable, keep_out, margins = args
    return CoveragePlan(lane_spacing=used.lane_spacing, config=own, **sent, **plan_geometry(
        boundary, obstacles, used, drivable, keep_out, margins, reach_over, unmowed, cells))


def refused(plan: CoveragePlan) -> str:
    """Why a plan from elsewhere isn't driven here, '' where it is: its form (the segments in order over all poses, the
    sections where they start, every pose facing the way it's driven to the next (faces), no two on one spot) or what
    report.rules finds (with the geometry and body of here: plan_from)."""
    poses, segs = plan.poses, plan.segments
    n = len(poses)
    if n < 2 or not segs or segs[0].start != 0 or segs[-1].end != n - 1:
        return 'the segments don\'t go from the first pose to the last'
    for i, s in enumerate(segs):
        if s.start >= s.end or i + 1 < len(segs) and segs[i + 1].start not in (s.end, s.end + 1):
            return f'segment {i} isn\'t in order'
    if plan.section_starts != [s.start for i, s in enumerate(segs) if i == 0 or s.section != segs[i - 1].section]:
        return 'the sections don\'t start where the segments do'
    back = [False] * n
    for s in segs:
        back[s.start:s.end] = [s.reverse] * (s.end - s.start)
    jumps = {s - 1 for s in plan.section_starts[1:]}
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        if not -math.pi < a.yaw <= math.pi:
            return f'pose {i}: heading out of range'
        d = math.hypot(b.x - a.x, b.y - a.y)
        if d <= 1e-7:
            return f'poses {i} and {i + 1} on one spot'
        if i in jumps:
            continue
        travel = math.atan2(b.y - a.y, b.x - a.x) + (math.pi if back[i] else 0.0)
        if back[i] and d < 2 * CORNER_STEP and i:
            # (a sharp corner's turn pose backing up, backing.corners: facing against the ways in and out)
            before = math.atan2(a.y - poses[i - 1].y, a.x - poses[i - 1].x) + math.pi
            if angle_diff(travel, a.yaw) < 0.5 * math.pi and angle_diff(before, a.yaw) < 0.5 * math.pi:
                continue
        if not faces(a, b, travel):
            return f'pose {i} doesn\'t face the way to the next one'
    if not -math.pi < poses[-1].yaw <= math.pi:
        return f'pose {n - 1}: heading out of range'
    hard = rules(plan)['hard']
    return '; '.join(hard)


def faces(a: Pose2D, b: Pose2D, travel: float) -> bool:
    """Whether a pose faces the way (travel) to the next, b: within FACING, or on an arc (a quarter turn at most from a
    to b, a tight corner's steps of a few cm) of the heading halfway between both."""
    if angle_diff(travel, a.yaw) <= FACING:
        return True
    turn = math.remainder(b.yaw - a.yaw, 2.0 * math.pi)
    return abs(turn) <= 0.5 * math.pi and angle_diff(travel, a.yaw + 0.5 * turn) <= FACING


# -- reading strictly ----------------------------------------------------------------------------------------------

def _object(v: Any, what: str) -> Dict[str, Any]:
    if not isinstance(v, dict):
        raise RemoteError(f'{what} is no object')
    return v


def _list(v: Any, what: str) -> list:
    if not isinstance(v, list):
        raise RemoteError(f'{what} is no list')
    return v


def _number(v: Any, what: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise RemoteError(f'{what}: {v!r} is no number')
    return float(v)


def _index(v: Any, what: str) -> int:
    if isinstance(v, bool) or not isinstance(v, int) or v < 0:
        raise RemoteError(f'{what}: {v!r} is no index')
    return v


def _string(v: Any, what: str) -> str:
    if not isinstance(v, str) or len(v) > MAX_STRING:
        raise RemoteError(f'{what}: no string of {MAX_STRING} characters at most')
    return v


def _strings(form: Dict[str, Any], name: str) -> List[str]:
    return [_string(s, name) for s in _list(form.get(name), name)]


def _nested(v: Any, what: str, depth: int) -> Any:
    """v where it's lists depth deep of numbers (0: a number)."""
    if depth == 0:
        _number(v, what)
    else:
        for x in _list(v, what):
            _nested(x, what, depth - 1)
    return v


def _flat(v: Any, what: str, k: int) -> List[Tuple[float, ...]]:
    """[a, b, ..., a, b, ...] as tuples of k numbers."""
    values = [_number(c, what) for c in _list(v, what)]
    if len(values) % k:
        raise RemoteError(f'{what}: not {k} numbers each')
    return [tuple(values[i:i + k]) for i in range(0, len(values), k)]


def _segment(v: Any) -> tuple:
    s = _list(v, 'segments')
    if len(s) != 8:
        raise RemoteError('segments: not 8 fields each')
    kind, start, end, reverse, maneuver, section, source, blade = s
    if kind not in KINDS or not isinstance(reverse, bool) or not isinstance(blade, bool):
        raise RemoteError('segments: a kind or flag that isn\'t one')
    return (kind, _index(start, 'segments'), _index(end, 'segments'), reverse, _string(maneuver, 'segments'),
            _index(section, 'segments'), _string(source, 'segments'), blade)


def _cells(v: Any) -> Tuple[float, List[List[Lane]]]:
    if len(_list(v, 'cells')) != 2:
        raise RemoteError('cells: not an angle and its cells each')
    angle, cells = v
    return _number(angle, 'cells'), [[_lane(lane) for lane in _list(cell, 'cells')] for cell in _list(cells, 'cells')]


def _lane(v: Any) -> Lane:
    lane = _list(v, 'cells')
    if len(lane) != 5:
        raise RemoteError('cells: a lane not of 5 fields')
    row, x0, x1, y, bends = lane
    if isinstance(row, bool) or not isinstance(row, int):
        raise RemoteError('cells: a row that isn\'t one')
    out = []
    for bend in _list(bends, 'cells'):
        if len(_list(bend, 'cells')) != 3:
            raise RemoteError('cells: a bend not of 3 fields')
        out.append((_number(bend[0], 'cells'), _number(bend[1], 'cells'), tuple(_flat(bend[2], 'cells', 2))))
    return Lane(row, _number(x0, 'cells'), _number(x1, 'cells'), _number(y, 'cells'), tuple(out))
