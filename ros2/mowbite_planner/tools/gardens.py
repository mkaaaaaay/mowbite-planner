"""Gardens and mowers for tools/simbench.py. Not part of the package: it doesn't go onto the mower.

A garden is a map like OpenMower's map.json (mowing areas, obstacles, navigation areas, a docking station) and the
mowing areas in it to plan, by name:

  real-mkay, real-ape  the gardens of ROS_HOME-like folders (map.json, mowbite_planner_angle.json), see REAL
  test-<name>          the gardens of the planner's tests (test/test_planner.py's GARDEN with TREE and BED, ...)
  <kind>-<seed>        made up, nasty on purpose (see KINDS): the same kind and seed give the same garden

A mower (see MOWERS) is what a mower keeps for the planner (mowbite_planner_settings.json), its mower_logic
parameters and its blade.

The real gardens and mowers are looked for next to the repo (MOWBITE_PROJECT, its parent folder by default):
scripts/omsim/<who>/data and logs/mower-tracks-20261022.
"""

import ast
import json
import math
import os
import random
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from shapely import affinity
from shapely.geometry import LinearRing, LineString, Point, Polygon
from shapely.ops import nearest_points, unary_union

XY = Tuple[float, float]
Pose = Tuple[float, float, float]

TOOLS = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(TOOLS)  # ros2/mowbite_planner
PROJECT = os.environ.get('MOWBITE_PROJECT', os.path.dirname(os.path.dirname(os.path.dirname(PACKAGE))))
OMSIM = os.path.join(PROJECT, 'scripts', 'omsim')
TRACKS = os.path.join(PROJECT, 'logs', 'mower-tracks-20261022')
# real gardens: name -> ROS_HOME-like folder
REAL = {'real-mkay': os.path.join(OMSIM, 'mkay', 'data', 'ros'),
        'real-ape': os.path.join(OMSIM, 'ape', 'data', 'ros')}
# the planner's test gardens: name -> (outline, obstacles, dock), expressions over test_planner.py's names
TEST_FILE = os.path.join(PACKAGE, 'test', 'test_planner.py')
TESTS = {
    'garden': ('GARDEN', '[TREE, BED]', (1.0, 1.0, 0.0)),
    'u': ('U_LAWN', '[]', (1.0, 1.0, 0.0)),
    'trees': ('rect(0, 0, 30, 20)', 'TREES', (1.0, 1.0, 0.0)),
    'shade': ('SHADE_LAWN', '[SHADE_TABLE, SHADE]', (0.54, 9.5, 0.0)),
    'pathend': ('PATH_END', '[]', (5.973, -4.3, -1.57)),
    'triangle': ("ANGLE_SHAPES['triangle']", '[]', None),
    'L': ("ANGLE_SHAPES['L']", '[]', None),
}
TEST_NAMES = ('circle', 'rect', 'GARDEN', 'TREE', 'BED', 'U_LAWN', 'TREES', 'SHADE_LAWN', 'SHADE_TABLE', 'SHADE',
              'PATH_END', 'ANGLE_SHAPES')

# mowers: name -> (settings files, parameter files with a mower_logic: section, the first there is taken), blade m
# (OM_BLADE_WIDTH, if the settings have no mower_width), settings on top
MOWERS: Dict[str, Tuple[Sequence[str], Sequence[str], float, Dict[str, Any]]] = {
    # the user's mower with its measured outline, settings and parameters as on the mower on 2026-10-22
    'mkay': ((os.path.join(TRACKS, 'mowbite_planner_settings.json'),
              os.path.join(REAL['real-mkay'], 'mowbite_planner_settings.json')),
             (os.path.join(TRACKS, 'params', 'mower_params.yaml'),
              os.path.join(OMSIM, 'mkay', 'data', 'params', 'custom_params.yaml')), 0.18, {}),
    # Apehaenger's John Deere Tango E5 (long nose) with its outline, his settings and parameters of 2026-10-17
    'tango': ((os.path.join(REAL['real-ape'], 'mowbite_planner_settings.json'),),
              (os.path.join(OMSIM, 'ape', 'data', 'params', 'custom_params.yaml'),), 0.18, {}),
    # the user's mower as a rectangle (no robot_outline), everything else the launch file's and the planner's defaults
    'rect': ((), (), 0.18, {'robot_width': 0.41, 'robot_front': 0.43, 'robot_rear': 0.18, 'blade_ahead': 0.185}),
    # no body at all (not among the defaults of simbench)
    'bare': ((), (), 0.18, {}),
}
# OpenMower's mower_logic defaults (MowerLogic.cfg) where a mower has no parameter file
MOWER_LOGIC_DEFAULTS = {'outline_count': 3, 'outline_overlap_count': 0, 'outline_offset': 0.0, 'mow_angle_offset': 0.0,
                        'mow_angle_offset_is_absolute': False, 'mow_angle_increment': 0.0, 'tool_width': 0.14}

# m2, the made-up gardens' mowing area (its outline) lies in this range
AREA_MIN, AREA_MAX = 15.0, 300.0
# m, necks, corridors, strips and gaps around the bodies' widths (0.41 m, the Tango 0.51 m)
NARROW = (0.45, 1.3)
# m, how recorded outlines are: a point every RECORD_STEP apart (or only the corners), off by RECORD_NOISE (1 sigma),
# RECORD_POINTS at most
RECORD_STEP = (0.08, 0.4)
RECORD_POINTS = 1500
RECORD_NOISE = (0.0, 0.004, 0.01)
# m, a made-up garden fits in a square this wide
EXTENT = 40.0
# drafts tried for a made-up garden before its seed is given up
TRIES = 40


@dataclass
class Garden:
    """A map to plan areas of: name (see the module), map like map.json, areas the ids of the mowing areas to plan,
    angle like mowbite_planner_angle.json ({'steps', 'finished'})."""

    name: str
    map: Dict[str, Any]
    areas: List[str]
    angle: Dict[str, Any] = field(default_factory=dict)
    kind: str = ''
    source: str = ''
    m2: float = 0.0  # the first area's outline
    problems: List[str] = field(default_factory=list)  # validate()'s, for the real and test gardens


@dataclass
class Mower:
    """What the node plans with for a mower: stored like mowbite_planner_settings.json, mower_logic its ROS
    parameters (outline_count, tool_width, ...), blade m (OM_BLADE_WIDTH)."""

    name: str
    stored: Dict[str, Any]
    mower_logic: Dict[str, Any]
    blade: float = 0.18
    source: str = ''


class _Retry(Exception):
    """A draft that doesn't work out, the generator tries the next."""


# reading ------------------------------------------------------------------------------------------------------------

def _scalar(text: str) -> Any:
    text = text.strip().strip('"\'')
    if text in ('true', 'false', 'True', 'False'):
        return text.lower() == 'true'
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return text


def yaml_section(path: str, section: str) -> Dict[str, Any]:
    """The plain key: value lines of a top-level section of a YAML file (no pyyaml on the mower's image)."""
    out: Dict[str, Any] = {}
    inside = False
    with open(path) as f:
        for line in f:
            text = line.split('#', 1)[0].rstrip()
            if not text.strip():
                continue
            if not line[0].isspace():
                inside = text == section + ':'
            elif inside and ':' in text:
                key, value = text.strip().split(':', 1)
                if value.strip():
                    out[key.strip()] = _scalar(value)
    return out


def _first(paths: Sequence[str]) -> Optional[str]:
    return next((p for p in paths if os.path.exists(p)), None)


def mower(name: str) -> Mower:
    """A mower of MOWERS by name."""
    if name not in MOWERS:
        raise KeyError(f'no mower {name!r} (there are: {", ".join(MOWERS)})')
    settings, params, blade, extra = MOWERS[name]
    found, stored = _first(settings), {}
    if settings and found is None:
        raise FileNotFoundError(f'mower {name}: none of {", ".join(settings)}')
    if found:
        with open(found) as f:
            stored = json.load(f)
    ml = _first(params)
    if params and ml is None:
        raise FileNotFoundError(f'mower {name}: none of {", ".join(params)}')
    mower_logic = yaml_section(ml, 'mower_logic') if ml else dict(MOWER_LOGIC_DEFAULTS)
    return Mower(name, dict(stored, **extra), mower_logic, blade, ', '.join(p for p in (found, ml) if p))


def _json(path: str, default: Any) -> Any:
    try:
        with open(path) as f:
            return json.load(f)
    except OSError:
        return default


def real_garden(name: str) -> Garden:
    """A ROS_HOME-like folder of REAL: the active mowing areas of its map.json (the not mowable ones too)."""
    root = REAL[name]
    data = _json(os.path.join(root, 'map.json'), None)
    if data is None:
        raise FileNotFoundError(f'{name}: no map.json in {root}')
    ids = [str(a.get('id')) for a in data.get('areas', []) if a.get('properties', {}).get('type') == 'mow'
           and a.get('properties', {}).get('active') is not False]
    garden = Garden(name, data, ids, _json(os.path.join(root, 'mowbite_planner_angle.json'), {}), 'real', root)
    garden.m2 = _outline_m2(data, ids[0]) if ids else 0.0
    garden.problems = validate(garden, limits=False)
    return garden


def _test_names() -> Dict[str, Any]:
    """circle, rect and the gardens of test_planner.py, its own lines (it needs pytest to be imported)."""
    with open(TEST_FILE) as f:
        tree = ast.parse(f.read())
    keep = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in TEST_NAMES or isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id in TEST_NAMES for t in n.targets)]
    names: Dict[str, Any] = {'math': math}
    exec(compile(ast.Module(body=keep, type_ignores=[]), TEST_FILE, 'exec'), names)
    return names


def from_tests(name: str) -> Garden:
    outline, obstacles, dock = TESTS[name[len('test-'):]]
    names = _test_names()
    return _garden(name, [eval(outline, names)], eval(obstacles, names), [], dock, [0], 'test', TEST_FILE,
                   limits=False)


# made up ------------------------------------------------------------------------------------------------------------

def _box(x0: float, y0: float, x1: float, y1: float) -> Polygon:
    return Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def _disk(x: float, y: float, r: float, n: int = 16) -> Polygon:
    return Polygon([(x + r * math.cos(2 * math.pi * i / n), y + r * math.sin(2 * math.pi * i / n)) for i in range(n)])


def _star(rnd: random.Random, n: int, lo: float, hi: float = 1.0) -> Polygon:
    """n corners round the origin, lo..hi from it, at about even angles (a simple polygon)."""
    angles = [2 * math.pi * (i + rnd.uniform(-0.35, 0.35)) / n for i in range(n)]
    return Polygon([(r * math.cos(a), r * math.sin(a)) for a, r in zip(angles, (rnd.uniform(lo, hi) for _ in angles))])


def _smooth(poly: Polygon, per: int = 10) -> Polygon:
    """The outline as quadratic Bezier curves from edge middle to edge middle, the corners their control points."""
    pts = list(poly.exterior.coords)[:-1]
    out = []
    for i, c in enumerate(pts):
        a, b = pts[i - 1], pts[(i + 1) % len(pts)]
        p0, p2 = ((a[0] + c[0]) / 2, (a[1] + c[1]) / 2), ((c[0] + b[0]) / 2, (c[1] + b[1]) / 2)
        out.extend(((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * c[0] + t * t * p2[0],
                    (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * c[1] + t * t * p2[1])
                   for t in (k / per for k in range(per)))
    return Polygon(out)


def _to_m2(poly: Polygon, m2: float) -> Polygon:
    f = math.sqrt(m2 / poly.area)
    return affinity.scale(poly, f, f, origin='centroid')


def _single(geom: Any) -> Polygon:
    """geom as one polygon without holes, _Retry if it isn't one."""
    if not isinstance(geom, Polygon) or geom.is_empty or not geom.is_valid or geom.interiors:
        raise _Retry
    return Polygon(geom.exterior.coords)


def _arm(rnd: random.Random, m2: float) -> float:
    """m, the width of an arm of an L, U or T: narrow now and then, else a share of the size."""
    return rnd.uniform(*NARROW) if rnd.random() < 0.4 else rnd.uniform(0.2, 0.45) * math.sqrt(m2)


def _rect(rnd: random.Random, m2: float) -> Polygon:
    w = math.sqrt(m2 * rnd.uniform(1.0, 4.0))
    return _box(0, 0, w, m2 / w)


def _L(rnd: random.Random, m2: float) -> Polygon:
    a, b, q = _arm(rnd, m2), _arm(rnd, m2), rnd.uniform(0.6, 1.6)
    w = (m2 + a * b) / (a * q + b)  # a * h + (w - a) * b = m2 with h = q * w
    if w < a + 1.0 or q * w < b + 1.0:
        raise _Retry
    return _single(_box(0, 0, w, q * w).difference(_box(a, b, w + 1, q * w + 1)))


def _U(rnd: random.Random, m2: float) -> Polygon:
    a, c, b, q = _arm(rnd, m2), _arm(rnd, m2), _arm(rnd, m2), rnd.uniform(0.5, 1.5)
    w = (m2 + b * (a + c)) / (b + q * (a + c))  # w * h - (w - a - c) * (h - b) = m2 with h = q * w
    if w < a + c + 0.6 or q * w < b + 0.6:
        raise _Retry
    return _single(_box(0, 0, w, q * w).difference(_box(a, b, w - c, q * w + 1)))


def _T(rnd: random.Random, m2: float) -> Polygon:
    a, b, w = _arm(rnd, m2), _arm(rnd, m2), math.sqrt(m2) * rnd.uniform(1.0, 2.0)
    stem = (m2 - w * b) / a
    if stem < 0.8 or w < a + 1.0:
        raise _Retry
    x = rnd.uniform(0, w - a)  # (at an end: nearly an L)
    return _single(unary_union([_box(0, stem, w, stem + b), _box(x, 0, x + a, stem + b)]))


def _concave(rnd: random.Random, m2: float) -> Polygon:
    return _to_m2(_star(rnd, rnd.randint(6, 16), rnd.uniform(0.3, 0.7)), m2)


def _curvy(rnd: random.Random, m2: float) -> Polygon:
    """Bezier curves round a star, or circles: a few run into each other (sharp inner corners), or one with a bite."""
    pick = rnd.random()
    if pick < 0.55:
        shape = _smooth(_star(rnd, rnd.randint(5, 10), rnd.uniform(0.35, 0.8)))
    elif pick < 0.8:
        shape = unary_union([_disk(0, 0, 1.0, 64)] + [_disk(rnd.uniform(0.8, 1.6) * math.cos(a),
                                                            rnd.uniform(0.8, 1.6) * math.sin(a),
                                                            rnd.uniform(0.4, 0.9), 64)
                                                      for a in (rnd.uniform(0, 2 * math.pi)
                                                                for _ in range(rnd.randint(1, 3)))])
    else:
        shape = _disk(0, 0, 1.0, 64).difference(_disk(rnd.uniform(0.9, 1.3), 0, rnd.uniform(0.4, 0.8), 64))
    return _to_m2(_single(shape), m2)


BASES: Dict[str, Callable[[random.Random, float], Polygon]] = {
    'rect': _rect, 'L': _L, 'U': _U, 'T': _T, 'concave': _concave, 'curvy': _curvy}


def _base(rnd: random.Random, m2: float, simple: bool = False) -> Polygon:
    names = ('rect', 'concave', 'curvy') if simple else tuple(BASES)
    return BASES[rnd.choice(names)](rnd, m2)


def _on_edge(rnd: random.Random, poly: Polygon) -> Tuple[XY, XY]:
    """A point on the outline and the normal out of the area there."""
    ring = poly.exterior
    s = rnd.uniform(0, ring.length)
    p, a, b = ring.interpolate(s), ring.interpolate(s - 0.05), ring.interpolate(s + 0.05)
    tx, ty = b.x - a.x, b.y - a.y
    n = math.hypot(tx, ty) or 1.0
    nx, ny = ty / n, -tx / n
    if poly.contains(Point(p.x + 0.02 * nx, p.y + 0.02 * ny)):
        nx, ny = -nx, -ny
    return (p.x, p.y), (nx, ny)


def _spike(rnd: random.Random, lawn: Polygon, inward: bool) -> Polygon:
    """A sharp corner: a thin triangle out of the lawn (a spike, 8..35 deg) or into it (a notch)."""
    (x, y), (nx, ny) = _on_edge(rnd, lawn)
    if inward:
        nx, ny = -nx, -ny
    turn = math.radians(rnd.uniform(-40, 40))
    dx, dy = nx * math.cos(turn) - ny * math.sin(turn), nx * math.sin(turn) + ny * math.cos(turn)
    length = rnd.uniform(1.0, 4.0)
    half = length * math.tan(math.radians(rnd.uniform(8, 35)) / 2)
    bx, by = x - 0.3 * dx, y - 0.3 * dy  # (the base a bit on the other side of the edge)
    tri = Polygon([(bx - half * dy, by + half * dx), (bx + half * dy, by - half * dx),
                   (x + length * dx, y + length * dy)])
    return _single(lawn.difference(tri) if inward else lawn.union(tri))


def _attach(rnd: random.Random, lawn: Polygon, part: Optional[Polygon], width: float) -> Polygon:
    """Another lawn part joined by a corridor width wide (a neck), or without a part a strip width wide (along a
    house), both bending now and then."""
    (x, y), (nx, ny) = _on_edge(rnd, lawn)
    length = rnd.uniform(0.3, 4.0) if part is not None else rnd.uniform(2.0, 12.0)
    way = [(x - 0.4 * nx, y - 0.4 * ny), (x + length * nx, y + length * ny)]
    if rnd.random() < 0.4:  # round a corner
        side = rnd.choice((-1, 1))
        bend = rnd.uniform(1.0, 6.0)
        way.append((way[-1][0] - side * ny * bend, way[-1][1] + side * nx * bend))
    corridor = LineString(way).buffer(width / 2, cap_style=2, join_style=2)
    pieces = [lawn, corridor]
    if part is not None:
        ex, ey = way[-1]
        r = math.sqrt(part.area / math.pi)
        c = part.centroid
        dx, dy = (ex - way[-2][0], ey - way[-2][1])
        d = math.hypot(dx, dy)
        pieces.append(affinity.translate(part, ex + dx / d * 0.8 * r - c.x, ey + dy / d * 0.8 * r - c.y))
    return _single(unary_union(pieces))


def _scatter(rnd: random.Random, lawn: Polygon, count: int, edge: float, big: bool = False) -> List[Polygon]:
    """count obstacles: trees (circles 0.08..0.6 m) and beds, edge of them at the outline (crossing it, touching it,
    a gap off it), some in pairs a gap apart. Not more than a third of the lawn."""
    out: List[Polygon] = []
    inner = lawn.buffer(-0.3)
    for _ in range(count * 4):
        if len(out) >= count:
            break
        if rnd.random() < (0.35 if big else 0.75):
            r = rnd.uniform(0.08, 0.6)
            shape = _disk(0, 0, r, rnd.choice((8, 12, 16, 24)))
        else:
            shape = affinity.rotate(_box(0, 0, rnd.uniform(0.4, 3.0), rnd.uniform(0.3, 1.5)) if rnd.random() < 0.6
                                    else _to_m2(_star(rnd, rnd.randint(5, 9), 0.5), rnd.uniform(0.3, 4.0)),
                                    rnd.uniform(0, 180), origin='centroid')
            r = math.sqrt(shape.area / math.pi)
        c = shape.centroid
        if rnd.random() < edge:
            (x, y), (nx, ny) = _on_edge(rnd, lawn)
            gap = rnd.choice((-0.5 * r, 0.0, rnd.uniform(0.05, 0.3), rnd.uniform(0.3, 0.8)))
            x, y = x - (r + gap) * nx, y - (r + gap) * ny
        elif out and rnd.random() < 0.3:  # next to the one before, a narrow way between them
            o = out[-1]
            a = rnd.uniform(0, 2 * math.pi)
            d = math.sqrt(o.area / math.pi) + r + rnd.uniform(0.1, 0.8)
            x, y = o.centroid.x + d * math.cos(a), o.centroid.y + d * math.sin(a)
        else:
            if inner.is_empty:
                continue
            x0, y0, x1, y1 = inner.bounds
            x, y = rnd.uniform(x0, x1), rnd.uniform(y0, y1)
        shape = affinity.translate(shape, x - c.x, y - c.y)
        if lawn.contains(shape.representative_point()) and \
                sum(o.area for o in out) + shape.area < lawn.area / 3:
            out.append(shape)
    return out


def _island(rnd: random.Random, lawn: Polygon) -> Polygon:
    """A big obstacle in the middle (a pond, a bed), a ring of lawn round it NARROW wide at the narrowest."""
    core = lawn.buffer(-rnd.uniform(*NARROW))
    c = lawn.centroid
    blob = affinity.translate(_to_m2(_star(rnd, rnd.randint(5, 9), 0.5), lawn.area * rnd.uniform(0.2, 0.7)), c.x, c.y)
    island = max(_polygons(core.intersection(blob)), key=lambda p: p.area, default=None)
    if island is None or island.area < 0.1 * lawn.area:
        raise _Retry
    return _single(island.buffer(0))


def _polygons(geom: Any) -> List[Polygon]:
    return [g for g in getattr(geom, 'geoms', [geom]) if isinstance(g, Polygon) and not g.is_empty]


def _dock(rnd: random.Random, lawn: Polygon) -> Pose:
    """At the edge like real ones (0.1..0.5 m outside, now and then inside), facing out (into the station)."""
    (x, y), (nx, ny) = _on_edge(rnd, lawn)
    d = rnd.uniform(0.1, 0.5) if rnd.random() < 0.7 else -rnd.uniform(0.1, 0.4)
    return x + d * nx, y + d * ny, math.atan2(ny, nx) + math.radians(rnd.uniform(-20, 20))


Draft = Tuple[List[Polygon], List[Polygon], List[Polygon], Optional[Pose], List[int]]


def _draft(kind: str, rnd: random.Random, m2: float) -> Draft:
    """lawns (the first the main one), obstacles, navigation areas, dock, the lawns to plan."""
    obstacles: List[Polygon] = []
    if kind in BASES:
        lawn = BASES[kind](rnd, m2)
        obstacles = _scatter(rnd, lawn, rnd.randint(0, 3), 0.3)
    elif kind == 'neck':
        f = rnd.uniform(0.3, 0.7)
        lawn = _attach(rnd, _base(rnd, f * m2, simple=True), _base(rnd, (1 - f) * m2, simple=True),
                       rnd.uniform(*NARROW))
    elif kind == 'strip':
        width = rnd.uniform(0.5, 1.6)
        if rnd.random() < 0.35:  # the strip alone
            length = max(1.1 * AREA_MIN / width, rnd.uniform(8.0, 25.0))
            way = [(0.0, 0.0), (length, 0.0)] + ([(length, rnd.uniform(2, 8))] if rnd.random() < 0.5 else [])
            lawn = _single(LineString(way).buffer(width / 2, cap_style=2, join_style=2))
        else:
            lawn = _attach(rnd, _base(rnd, 0.85 * m2, simple=True), None, width)
    elif kind == 'acute':
        lawn = _base(rnd, m2, simple=True)
        for _ in range(rnd.randint(1, 4)):
            lawn = _spike(rnd, lawn, rnd.random() < 0.35)
    elif kind == 'trees':
        lawn = _base(rnd, m2, simple=True)
        obstacles = _scatter(rnd, lawn, rnd.randint(6, 25), 0.3)
    elif kind == 'edge':
        lawn = _base(rnd, m2)
        obstacles = _scatter(rnd, lawn, rnd.randint(3, 10), 1.0, big=True)
    elif kind == 'island':
        lawn = _base(rnd, m2, simple=True)
        obstacles = [_island(rnd, lawn)]
        obstacles += [o for o in _scatter(rnd, lawn, rnd.randint(0, 4), 0.3) if not o.intersects(obstacles[0])]
    elif kind == 'two':
        return _two(rnd, m2)
    elif kind == 'mixed':
        lawn = _base(rnd, m2)
        for change in rnd.sample(('spike', 'notch', 'neck', 'strip', 'trees', 'edge', 'island'), rnd.randint(2, 4)):
            if change in ('spike', 'notch'):
                lawn = _spike(rnd, lawn, change == 'notch')
            elif change in ('neck', 'strip'):
                part = _base(rnd, rnd.uniform(0.2, 0.5) * lawn.area, simple=True) if change == 'neck' else None
                lawn = _attach(rnd, lawn, part, rnd.uniform(*NARROW) if part else rnd.uniform(0.5, 1.6))
            elif change == 'island':
                obstacles.append(_island(rnd, lawn))
            else:
                obstacles += _scatter(rnd, lawn, rnd.randint(4, 15), 1.0 if change == 'edge' else 0.3)
        obstacles = [o for i, o in enumerate(obstacles) if lawn.contains(o.representative_point())
                     and not any(o.intersects(p) and p.area > 5.0 for p in obstacles[:i])]
    else:
        raise KeyError(f'no garden kind {kind!r} (there are: {", ".join(KINDS)})')
    return [lawn], obstacles, [], _dock(rnd, lawn), [0]


def _two(rnd: random.Random, m2: float) -> Draft:
    """Two lawns: next to each other with an edge in common, or apart with a path (a navigation area) between them
    and the dock on it. Both planned."""
    f = rnd.uniform(0.25, 0.6)
    one, two = _base(rnd, (1 - f) * m2, simple=True), _base(rnd, f * m2, simple=True)
    x0, y0, x1, y1 = one.bounds
    if rnd.random() < 0.3:  # an edge in common: the second laid over the first, the first cut out of it
        two = affinity.translate(two, x1 - two.bounds[0] - rnd.uniform(0.5, 2.0), rnd.uniform(y0, y1) - two.centroid.y)
        two = max(_polygons(two.difference(one)), key=lambda p: p.area, default=None)
        if two is None or two.area < 0.5 * f * m2:
            raise _Retry
        return [one, _single(two)], [], [], _dock(rnd, one), [0, 1]
    two = affinity.translate(two, x1 + rnd.uniform(1.0, 6.0) - two.bounds[0], rnd.uniform(y0, y1) - two.centroid.y)
    a, b = nearest_points(one, two)
    d = math.hypot(b.x - a.x, b.y - a.y)
    ux, uy = (b.x - a.x) / d, (b.y - a.y) / d
    path = LineString([(a.x - 0.4 * ux, a.y - 0.4 * uy), (b.x + 0.4 * ux, b.y + 0.4 * uy)]).buffer(
        rnd.uniform(0.7, 1.5) / 2, cap_style=2)
    dock = (a.x + 0.3 * ux, a.y + 0.3 * uy, math.atan2(uy, ux))  # on the path, facing the second lawn
    obstacles = _scatter(rnd, one, rnd.randint(0, 4), 0.3) + _scatter(rnd, two, rnd.randint(0, 4), 0.3)
    return [one, two], obstacles, [path], dock, [0, 1]


KINDS = tuple(BASES) + ('neck', 'strip', 'acute', 'trees', 'edge', 'island', 'two', 'mixed')


def _recorded(rnd: random.Random, poly: Polygon) -> List[XY]:
    """The outline as recorded: a point every few cm to dm (the corners kept), a little off, from anywhere on it, either
    way round. The exact outline if that comes out broken."""
    pts = list(poly.exterior.coords)[:-1]
    step, noise = max(rnd.uniform(*RECORD_STEP), poly.length / RECORD_POINTS), rnd.choice(RECORD_NOISE)
    dense = rnd.random() < 0.7
    out: List[XY] = []
    for a, b in zip(pts, pts[1:] + pts[:1]):
        out.append(a)
        n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / step) if dense else 0
        out.extend((a[0] + (b[0] - a[0]) * k / n + rnd.gauss(0, noise), a[1] + (b[1] - a[1]) * k / n
                    + rnd.gauss(0, noise)) for k in range(1, n))
    k = rnd.randrange(len(out))
    out = out[k:] + out[:k]
    if rnd.random() < 0.5:
        out.reverse()
    ring = LinearRing(out)
    return out if ring.is_valid and ring.is_simple and Polygon(ring).is_valid else pts


def _garden(name: str, lawns: Sequence[Any], obstacles: Sequence[Any], nav: Sequence[Any], dock: Optional[Pose],
            plan: Sequence[int], kind: str, source: str, limits: bool = True) -> Garden:
    def area(i: int, what: str, ring: Sequence[Sequence[float]], props: Dict[str, Any]) -> Dict[str, Any]:
        return {'id': f'{what}{i}', 'properties': dict(props, name=f'{what} {i}'),
                'outline': [{'x': round(float(x), 4), 'y': round(float(y), 4)} for x, y in ring]}

    areas = [area(i, 'lawn', r, {'type': 'mow'}) for i, r in enumerate(lawns)]
    areas += [area(i, 'obstacle', r, {'type': 'obstacle'}) for i, r in enumerate(obstacles)]
    areas += [area(i, 'path', r, {'type': 'nav'}) for i, r in enumerate(nav)]
    data: Dict[str, Any] = {'areas': areas, 'docking_stations': []}
    if dock is not None:
        data['docking_stations'].append({'id': 'dock', 'properties': {'name': 'Docking Station'}, 'heading': dock[2],
                                         'position': {'x': dock[0], 'y': dock[1]}})
    ids = [f'lawn{i}' for i in plan]
    garden = Garden(name, data, ids, {}, kind, source, _outline_m2(data, ids[0]))
    garden.problems = validate(garden, limits)
    return garden


def generate(kind: str, seed: int) -> Garden:
    """A made-up garden of a kind (KINDS), the same for the same seed. ValueError if no try passes validate()."""
    if kind not in KINDS:
        raise KeyError(f'no garden kind {kind!r} (there are: {", ".join(KINDS)})')
    rnd = random.Random(f'{kind}-{seed}')
    why: List[str] = []
    for _ in range(TRIES):
        m2 = math.exp(rnd.uniform(math.log(AREA_MIN), math.log(AREA_MAX)))
        try:
            lawns, obstacles, nav, dock, plan = _draft(kind, rnd, m2)
        except _Retry:
            continue
        x0, y0, x1, y1 = unary_union(lawns).bounds
        if max(x1 - x0, y1 - y0) > EXTENT:
            continue
        # turned and moved, the outlines as recorded
        turn, dx, dy = rnd.uniform(-180, 180), rnd.uniform(-15, 15), rnd.uniform(-15, 15)
        lawns, obstacles, nav = ([affinity.translate(affinity.rotate(p, turn, origin=(0, 0)), dx, dy) for p in ps]
                                 for ps in (lawns, obstacles, nav))
        if dock is not None:
            x, y = affinity.translate(affinity.rotate(Point(dock[0], dock[1]), turn, origin=(0, 0)), dx, dy).coords[0]
            dock = (x, y, math.remainder(dock[2] + math.radians(turn), 2 * math.pi))
        garden = _garden(f'{kind}-{seed}', [_recorded(rnd, p) for p in lawns], [_recorded(rnd, p) for p in obstacles],
                         [list(p.exterior.coords)[:-1] for p in nav], dock, plan, kind, 'generated')
        if not garden.problems:
            return garden
        why = garden.problems
    raise ValueError(f'{kind}-{seed}: no garden in {TRIES} tries ({"; ".join(why[:3]) or "drafts failed"})')


# checks and lookups -------------------------------------------------------------------------------------------------

def _outline(area: Dict[str, Any]) -> List[XY]:
    pts = [(float(p['x']), float(p['y'])) if isinstance(p, dict) else (float(p[0]), float(p[1]))
           for p in area.get('outline', [])]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    return pts


def _outline_m2(data: Dict[str, Any], area_id: str) -> float:
    found = [a for a in data['areas'] if str(a.get('id')) == area_id]
    pts = _outline(found[0]) if found else []
    return abs(Polygon(pts).area) if len(pts) >= 3 else 0.0


def validate(garden: Garden, limits: bool = True) -> List[str]:
    """What's wrong with a garden's geometry: outlines with fewer than 3 points, crossing themselves, points on one
    another (1 mm), obstacles not in a mowing area or most of it, mowing areas over each other; with limits the areas
    to plan out of AREA_MIN..AREA_MAX (10 % slack), less than half of them lawn."""
    problems: List[str] = []
    lawns: Dict[str, Polygon] = {}
    others: List[Tuple[str, Polygon]] = []
    for a in garden.map.get('areas', []):
        what = f'{a.get("properties", {}).get("type")} {a.get("id")}'
        pts = _outline(a)
        if len(pts) < 3:
            problems.append(f'{what}: {len(pts)} points')
            continue
        ring = LinearRing(pts)
        if not ring.is_simple or not Polygon(ring).is_valid:
            problems.append(f'{what}: the outline crosses itself')
            continue
        if any(math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-3 for a, b in zip(pts, pts[1:] + pts[:1])):
            problems.append(f'{what}: points less than 1 mm apart')
        kind = a.get('properties', {}).get('type')
        if kind == 'mow':
            lawns[str(a.get('id'))] = Polygon(ring)
        elif kind == 'obstacle':
            others.append((what, Polygon(ring)))
    for what, o in others:
        home = [p for p in lawns.values() if p.contains(o.representative_point())]
        if not home and not garden.kind == 'real':  # (real maps have obstacles of areas not mowed)
            problems.append(f'{what}: not in a mowing area')
        elif home and o.area > 0.6 * home[0].area:
            problems.append(f'{what}: most of its mowing area')
    ids = list(lawns)
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            if lawns[a].intersection(lawns[b]).area > 1e-3 and garden.kind != 'real':
                problems.append(f'mowing areas {a} and {b} lie over each other')
    for area_id in garden.areas:
        lawn = lawns.get(area_id)
        if lawn is None:
            problems.append(f'no valid mowing area {area_id}')
        elif limits:
            if not 0.9 * AREA_MIN <= lawn.area <= 1.1 * AREA_MAX:
                problems.append(f'{area_id}: {lawn.area:.0f} m2')
            blocked = unary_union([o for _, o in others]).intersection(lawn).area if others else 0.0
            if lawn.area - blocked < 0.5 * lawn.area:
                problems.append(f'{area_id}: less than half of it lawn')
    if limits and not garden.map.get('docking_stations'):
        problems.append('no docking station')
    return problems


def garden(name: str) -> Garden:
    """A garden by name (see the module)."""
    if name in REAL:
        return real_garden(name)
    if name.startswith('test-') and name[len('test-'):] in TESTS:
        return from_tests(name)
    kind, _, seed = name.rpartition('-')
    if kind in KINDS and seed.isdigit():
        return generate(kind, int(seed))
    raise KeyError(f'no garden {name!r}: real-mkay, real-ape, test-<{"|".join(TESTS)}> or <kind>-<seed>, kinds '
                   f'{", ".join(KINDS)}')


def names(kinds: Sequence[str] = KINDS, seeds: Sequence[int] = range(5), real: bool = True,
          tests: bool = True) -> List[str]:
    """Garden names: the real ones, the tests' and kinds x seeds made up."""
    return ([n for n in REAL if os.path.exists(os.path.join(REAL[n], 'map.json'))] if real else []) + \
        ([f'test-{t}' for t in TESTS] if tests and os.path.exists(TEST_FILE) else []) + \
        [f'{k}-{s}' for k in kinds for s in seeds]


if __name__ == '__main__':  # every kind once, how they come out
    import sys
    for n in sys.argv[1:] or [f'{k}-0' for k in KINDS]:
        g = garden(n)
        print(f'{g.name:14} {g.m2:6.1f} m2  areas {len(g.areas)}  obstacles '
              f'{sum(a["properties"]["type"] == "obstacle" for a in g.map["areas"]):2}  points '
              f'{len(g.map["areas"][0]["outline"]):4}  {"; ".join(g.problems)}')
