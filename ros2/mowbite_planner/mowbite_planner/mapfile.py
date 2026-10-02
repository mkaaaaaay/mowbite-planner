"""OpenMower's map.json (what the mower keeps and publishes on map/json) and its ROS parameters (params/json):
which area to plan, its obstacles, where else the mower may drive, and the settings that apply to it.

Settings are layered like OpenMower does it, the later ones win: the planner's own defaults, the mower's
global parameters (tool_width, outline_count, ...), the area's own properties (angle, outline_count, ...,
and a "planner" object with any MowBite Planner setting), then whatever a request asks for.
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple, Union

from .geometry import clean_polygon
from .planner import PlannerConfig, PlanningError

XY = Tuple[float, float]


@dataclass
class AreaJob:
    """Everything plan_coverage() needs for one area of a map."""

    area_id: str
    name: str
    boundary: List[XY]
    obstacles: List[List[XY]]  # the map's obstacles and the areas to mow around in this one
    mow_around: List[List[XY]]  # those areas to mow around alone (OpenMower sends its planner the obstacles)
    drivable: List[List[XY]]  # the other mowing areas, mowed when crossed
    nav: List[List[XY]]  # navigation areas and mowing areas not to mow: not for crossing while mowing
    overrides: Dict[str, Any] = field(default_factory=dict)  # planner settings from the area's properties
    planner: Dict[str, Any] = field(default_factory=dict)  # MowBite's own of them: angle range, planner object
    dock: Optional[Tuple[float, float, float]] = None  # x, y, heading of the first docking station


def _points(outline: Any) -> List[XY]:
    """An outline, [{x, y}, ...] or [[x, y], ...]. ValueError if it isn't one."""
    if not isinstance(outline, (list, tuple)):
        raise ValueError('the outline is not a list of points')
    out = []
    for i, p in enumerate(outline):
        if isinstance(p, Mapping):
            p = p.get('x'), p.get('y')
        x, y = (_number(p[0]), _number(p[1])) if isinstance(p, (list, tuple)) and len(p) >= 2 else (None, None)
        if x is None or y is None:
            raise ValueError(f'point {i} of the outline has no x and y as finite numbers')
        out.append((x, y))
    return out


def _props(area: Mapping[str, Any]) -> Mapping[str, Any]:
    p = area.get('properties')
    return p if isinstance(p, Mapping) else {}


def _number(v: Any) -> Optional[float]:
    """A finite number as float, None for anything else: missing, NaN (OpenMower's 'automatic'), not a
    number."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    try:
        v = float(v)
    except OverflowError:  # an int too big for a float
        return None
    return v if math.isfinite(v) else None


def _areas(data: Any) -> List[Mapping[str, Any]]:
    """The map's areas, leaving out what isn't one."""
    areas = data.get('areas') if isinstance(data, Mapping) else None
    return [a for a in areas if isinstance(a, Mapping)] if isinstance(areas, list) else []


def mow_areas(data: Mapping[str, Any]) -> List[Mapping[str, Any]]:
    """The map's mowing areas in map order (OpenMower counts its area index in this list)."""
    return [a for a in _areas(data) if _props(a).get('type') == 'mow']


def find_area(data: Mapping[str, Any], which: Union[str, int]) -> Mapping[str, Any]:
    """A mowing area by id, by name, or by its index among the mowing areas."""
    areas = mow_areas(data)
    if isinstance(which, int) and not isinstance(which, bool):
        if 0 <= which < len(areas):
            return areas[which]
    else:
        for key in ('id', 'name'):
            for a in areas:
                if (a.get('id') if key == 'id' else _props(a).get('name')) == which:
                    return a
    names = ', '.join(str(_props(a).get('name') or a.get('id')) for a in areas) or 'none'
    raise PlanningError(f'no mowing area {which!r} in the map (there are: {names})')


def match_area(data: Mapping[str, Any], outline: Any, tolerance: float = 1e-3) -> Optional[int]:
    """The index among the mowing areas of the one with this outline (as OpenMower sends an area's outline
    to its planner, float32): the first that isn't set to inactive or not mowable, else the first. None if
    no area has it. Areas with a broken outline are left out."""
    def points(outline: Any) -> Optional[List[XY]]:
        try:
            pts = _points(outline)
        except ValueError:
            return None
        if len(pts) > 1 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < tolerance:
            pts = pts[:-1]
        return pts

    pts = points(outline)
    first = None
    for i, area in enumerate(mow_areas(data) if pts else []):
        own = points(area.get('outline'))
        if own is not None and len(own) == len(pts) and all(
                abs(a[0] - b[0]) <= tolerance and abs(a[1] - b[1]) <= tolerance for a, b in zip(own, pts)):
            if _props(area).get('active') is not False and _props(area).get('mowable') is not False:
                return i
            if first is None:
                first = i
    return first


def openmower_settings(params: Optional[Mapping[str, Any]], mower_width: float) -> Dict[str, Any]:
    """The planner settings the mower's global ROS parameters (keys like /mower_logic/tool_width) stand for.

    OpenMower's tool_width is the lane spacing, not the blade: with the blade's mower_width the overlap
    follows from it. mow_angle_offset (degrees, brought into -180..180 like OpenMower does) turns the lanes,
    with mow_angle_offset_is_absolute it is the angle itself. The increment OpenMower adds after every
    finished job is its own state, not in here."""
    params = params if isinstance(params, Mapping) else {}

    def get(key: str) -> Any:
        return params.get('/mower_logic/' + key, params.get(key))

    out: Dict[str, Any] = {}
    spacing = _number(get('tool_width'))
    if spacing is not None and spacing > 0:
        width = max(mower_width, spacing)
        out['mower_width'] = width
        out['overlap'] = 1.0 - spacing / width
    count = _number(get('outline_count'))
    if count is not None and count >= 0:
        out['perimeter_passes'] = int(count)
    overlap_count = _number(get('outline_overlap_count'))
    if overlap_count is not None and overlap_count >= 0:
        out['lane_overlap_passes'] = int(overlap_count)
    offset = _number(get('outline_offset'))
    if offset is not None:
        out['perimeter_offset'] = offset
    angle_offset = _number(get('mow_angle_offset'))
    if angle_offset is not None:
        # into -180..180 like MowingBehavior::plan_area, the same arithmetic gives the same angle to the last bit
        angle_offset = math.fmod(angle_offset + 180, 360)
        angle_offset = (angle_offset + 360 if angle_offset < 0 else angle_offset) - 180
        if get('mow_angle_offset_is_absolute') is True:
            out['angle'] = math.radians(angle_offset)
        else:
            out['angle_offset'] = math.radians(angle_offset)
    return out


def area_settings(area: Mapping[str, Any], openmower: bool = True) -> Dict[str, Any]:
    """The planner settings an area's properties stand for, like OpenMower reads them (angle in rad, NaN or
    missing = automatic, -1 or missing = the global value), plus MowBite's angle_min/angle_max and a
    "planner" object with any setting of the MowBite Planner by its name. An angle_strategy there without
    an angle works the angle out. openmower=False: only MowBite's, the ones OpenMower doesn't know."""
    p = _props(area)
    if not openmower:
        p = {k: v for k, v in p.items() if k in ('angle_min', 'angle_max', 'planner')}
    out: Dict[str, Any] = {}
    angle = _number(p.get('angle'))
    if angle is not None:
        out['angle'] = angle
    for prop, name in (('outline_count', 'perimeter_passes'), ('outline_overlap_count', 'lane_overlap_passes')):
        v = _number(p.get(prop))
        if v is not None and v >= 0:
            out[name] = int(v)
    offset = _number(p.get('outline_offset'))
    if offset is not None:
        out['perimeter_offset'] = offset
    lo, hi = _number(p.get('angle_min')), _number(p.get('angle_max'))
    if lo is not None and hi is not None:
        out['angle_min'], out['angle_max'] = lo, hi
    extra = p.get('planner') or {}
    if not isinstance(extra, Mapping):
        raise PlanningError('planner must be an object of planner settings')
    out.update(extra)
    if 'angle_strategy' in extra and 'angle' not in extra:
        out['angle'] = None  # the strategy works it out, not the area's angle
    return out


def _dock(data: Mapping[str, Any]) -> Optional[Tuple[float, float, float]]:
    """x, y and heading of the first docking station with a position."""
    stations = data.get('docking_stations')
    for d in stations if isinstance(stations, list) else []:
        pos = d.get('position') if isinstance(d, Mapping) else None
        if isinstance(pos, Mapping):
            x, y, heading = _number(pos.get('x')), _number(pos.get('y')), _number(d.get('heading') or 0.0)
            if None not in (x, y, heading):
                return x, y, heading
    return None


def area_job(data: Mapping[str, Any], which: Union[str, int],
             properties: Optional[Mapping[str, Any]] = None) -> AreaJob:
    """What to plan for one mowing area of a map (by id, name or index, see find_area). properties replace
    the area's own (a request's angle, outline_count, ... like in OpenMower's mowing.plan).

    obstacles: every active obstacle of the map (the planner leaves out what's outside the area), and the
    mowing areas set to not mowable with mow_around that lie in this one (MowBite's mow_around: mowed
    around, not across), these also in mow_around. drivable: the other active mowing areas, for getting
    from one part of this area to another. Not the navigation areas and the mowing areas not to mow (nav):
    OpenMower mows a path from start to end, a part only reachable over them is a section of its own.
    Other areas with a broken outline are left out, a broken area to plan is a PlanningError."""
    found = find_area(data, which)
    area = dict(found, properties=dict(_props(found), **properties)) if properties else found
    try:
        boundary = _points(area.get('outline'))
        if len(boundary) < 3:
            raise ValueError('the outline has fewer than 3 points')
        overrides = area_settings(area)
        PlannerConfig().with_overrides(overrides).validate()
        own = area_settings(area, openmower=False)
    except ValueError as e:  # PlanningError too
        raise PlanningError(f'area {area.get("id")!r}: {e}') from None
    shape = clean_polygon(boundary)
    obstacles, mow_around, drivable, nav = [], [], [], []
    for other in _areas(data):
        p = _props(other)
        if other is found or p.get('active') is False:
            continue
        try:
            pts = _points(other.get('outline'))
        except ValueError:
            continue
        if len(pts) < 3:
            continue
        kind = p.get('type')
        if kind == 'obstacle':
            obstacles.append(pts)
        elif kind == 'mow' and p.get('mowable') is not False:
            drivable.append(pts)
        elif kind == 'mow' and p.get('mow_around') is True:
            # one that covers this whole area doesn't count, the area is what's left of it
            if not clean_polygon(pts).buffer(1e-6).contains(shape):
                obstacles.append(pts)
                mow_around.append(pts)
        elif kind in ('nav', 'mow'):
            nav.append(pts)
    return AreaJob(area_id=str(area.get('id')), name=str(_props(area).get('name') or area.get('id')),
                   boundary=boundary, obstacles=obstacles, mow_around=mow_around, drivable=drivable, nav=nav,
                   overrides=overrides, planner=own, dock=_dock(data))


def openmower_angle(boundary: List[XY]) -> float:
    """OpenMower's lane angle for an area without one: towards the first point more than 2 m from the first."""
    for x, y in boundary[1:]:
        if math.hypot(x - boundary[0][0], y - boundary[0][1]) > 2.0:
            return math.atan2(y - boundary[0][1], x - boundary[0][0])
    return 0.0


def settings_for(base: PlannerConfig, params: Optional[Mapping[str, Any]], job: Optional[AreaJob],
                 request: Optional[Mapping[str, Any]] = None, own_angle: bool = False) -> PlannerConfig:
    """The planner settings for a job like the ROS 1 node gets them: base < the mower's parameters < the
    area < the request. The lanes go the way mower_logic asks for: the area's angle (without one OpenMower's
    own, see openmower_angle) plus mow_angle_offset, with mow_angle_offset_is_absolute the offset alone.
    Not when the area's planner object has an angle or angle_strategy, nor when base has one (own_angle:
    angle or angle_strategy in the settings for all areas) and the area has no angle of its own."""
    mower = openmower_settings(params, base.mower_width)
    overrides: Dict[str, Any] = {}
    if job is not None:
        absolute, offset = mower.pop('angle', None), mower.pop('angle_offset', 0.0)
        overrides = dict(job.overrides)
        if 'angle' not in job.planner:
            area = overrides.pop('angle', None)
            if area is not None or not own_angle:
                angle = openmower_angle(job.boundary) if area is None else area
                overrides['angle'] = absolute if absolute is not None else angle + offset
    cfg = base.with_overrides(mower).with_overrides(overrides)
    if request:
        cfg = cfg.with_overrides(request)
    cfg.validate()
    return cfg
