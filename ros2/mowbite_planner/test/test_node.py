"""Tests of the ROS 2 node: with a sourced ROS 2 workspace (colcon test) against the real rclpy and the
built interfaces, otherwise against the stand-ins in fake_ros.py."""

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)

try:
    import rclpy
    from mowbite_planner_interfaces.srv import PlanCoverage, PlanPath
    REAL_ROS = True
except ImportError:
    import fake_ros
    fake_ros.install()
    import rclpy
    from mowbite_planner_interfaces.srv import PlanCoverage, PlanPath
    REAL_ROS = False

from geometry_msgs.msg import Point, Point32, Polygon  # noqa: E402
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from mowbite_planner.mathutil import angle_diff, quaternion_to_yaw  # noqa: E402
from mowbite_planner.node import MowbitePlannerNode, float32_warnings, pairs, parse_polygon  # noqa: E402

GARDEN = [(0.0, 0.0), (22.0, 0.0), (24.0, 9.0), (13.0, 9.0), (13.0, 16.0), (0.0, 16.0)]
BED = [(15.0, 2.5), (19.0, 2.5), (19.0, 5.0), (15.0, 5.0)]
TREE = [(5 + 0.9 * math.cos(a * math.pi / 8), 6 + 0.9 * math.sin(a * math.pi / 8)) for a in range(16)]
# a bed across the whole lawn splits it in two
LAWN = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
HEDGE = [(4.0, -1.0), (6.0, -1.0), (6.0, 7.0), (4.0, 7.0)]


def polygon(points):
    return Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in points])


@pytest.fixture
def node():
    if REAL_ROS:
        rclpy.init()
    n = MowbitePlannerNode()
    yield n
    n.destroy_node()
    if REAL_ROS:
        rclpy.try_shutdown()


def call_plan_coverage(node, **fields):
    request = PlanCoverage.Request()
    request.boundary = polygon(fields.pop('boundary', GARDEN))
    request.obstacles = [polygon(o) for o in fields.pop('obstacles', [TREE, BED])]
    for k, v in fields.items():
        setattr(request, k, v)
    return node._on_plan_coverage(request, PlanCoverage.Response())


def override(name, value):
    """A request's override, rcl_interfaces/Parameter of the value's type (None: not set)."""
    type_ = Parameter.Type.from_parameter_value(value)
    fields = {} if type_ == Parameter.Type.NOT_SET else {type_.name.lower() + '_value': value}
    return ParameterMsg(name=name, value=ParameterValue(type=type_.value, **fields))


def xy(path):
    return [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]


def check_path(path, frame='map'):
    assert path.header.frame_id == frame
    assert len(path.poses) > 10
    for ps in path.poses:
        assert ps.header.frame_id == frame
        assert ps.header.stamp == path.header.stamp
        q = ps.pose.orientation
        assert q.x == 0.0 and q.y == 0.0
        assert math.isclose(q.z * q.z + q.w * q.w, 1.0, abs_tol=1e-9)
    for a, b in zip(path.poses, path.poses[1:]):
        pa, pb = a.pose.position, b.pose.position
        assert math.hypot(pb.x - pa.x, pb.y - pa.y) > 1e-7


def test_plan_coverage_service(node):
    res = call_plan_coverage(node)
    assert res.success, res.message
    check_path(res.path)
    n = len(res.path.poses)
    assert res.path_length > 100 and res.lane_spacing == pytest.approx(0.22 * 0.85)
    assert 0.0 <= res.angle < math.pi
    # segments: in order, cover the path, consistent with the reverse flags
    starts, ends = list(res.segment_start), list(res.segment_end)
    assert starts[0] == 0 and ends[-1] == n - 1
    assert all(t in (e, e + 1) for e, t in zip(ends, starts[1:]))
    types = list(res.segment_type)
    assert set(types) <= {PlanCoverage.Response.SEGMENT_PERIMETER, PlanCoverage.Response.SEGMENT_LANE,
                          PlanCoverage.Response.SEGMENT_TURN, PlanCoverage.Response.SEGMENT_TRANSIT}
    assert types[0] == PlanCoverage.Response.SEGMENT_PERIMETER
    # backing up: the middle leg of a K-turn, between lanes or in place of a turn on the spot in any segment
    assert any(res.segment_reverse) == any(t == PlanCoverage.Response.SEGMENT_TURN and rev
                                           for t, rev in zip(types, res.segment_reverse)) or any(res.segment_reverse)
    # the orientation is the driving direction (or its opposite on reverse legs)
    reverse = [False] * n
    for s, e, rev in zip(starts, ends, res.segment_reverse):
        for i in range(s, e):
            reverse[i] = rev
    for i, (a, b) in enumerate(zip(res.path.poses, res.path.poses[1:])):
        pa, pb, q = a.pose.position, b.pose.position, a.pose.orientation
        travel = math.atan2(pb.y - pa.y, pb.x - pa.x)
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w) + (math.pi if reverse[i] else 0.0)
        assert angle_diff(travel, yaw) < math.radians(6)
    if not REAL_ROS:
        published = node.publishers['~/coverage_path'].published
        assert published == [res.path]


def test_plan_coverage_overrides(node):
    res = call_plan_coverage(node, mower_width=0.3, overlap=0.1, perimeter_passes=1, use_angle=True,
                             angle=math.radians(30))
    assert res.success, res.message
    assert res.lane_spacing == pytest.approx(0.27)
    assert res.angle == pytest.approx(math.radians(30))


def test_plan_coverage_start_pose(node):
    request = PlanCoverage.Request()
    request.boundary = polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)])
    request.use_start_pose = True
    request.start_pose.position.x = 10.5
    request.start_pose.position.y = 3.0
    request.start_pose.orientation.z = 1.0
    request.start_pose.orientation.w = 0.0
    res = node._on_plan_coverage(request, PlanCoverage.Response())
    assert res.success, res.message
    first = res.path.poses[0].pose.position
    assert first.x == pytest.approx(10.0 - 0.11) and abs(first.y - 3.0) < 0.35


def test_plan_coverage_bad_input(node):
    res = call_plan_coverage(node, boundary=[(0.0, 0.0), (1.0, 1.0)], obstacles=[])
    assert not res.success and res.message
    res = call_plan_coverage(node, overlap=1.5)
    assert not res.success and 'overlap' in res.message


def test_plan_path_like_openmower(node):
    request = PlanPath.Request()
    request.fill_type = PlanPath.Request.FILL_LINEAR
    request.angle = 0.3
    request.distance = 0.13
    request.outer_offset = 0.0
    request.outline_count = 3
    request.outline_overlap_count = 0
    request.outline = polygon(GARDEN)
    request.holes = [polygon(TREE), polygon(BED)]
    res = node._on_plan_path(request, PlanPath.Response())
    assert [p.is_outline for p in res.paths] == [1, 0]
    for piece in res.paths:
        check_path(piece.path)
        assert len(piece.path.poses) >= 3
    # the first loop runs on the recorded outline (outer_offset 0)
    first = res.paths[0].path.poses[0].pose.position
    assert min(abs(first.x), abs(first.y), abs(16 - first.y), abs(13 - first.x)) < 0.05 or first.y < 0.05

    request.skip_fill = True
    assert [p.is_outline for p in node._on_plan_path(request, PlanPath.Response()).paths] == [1]
    request.skip_fill = False
    request.skip_area_outline = True
    request.skip_obstacle_outlines = True
    assert [p.is_outline for p in node._on_plan_path(request, PlanPath.Response()).paths] == [0]
    request.fill_type = 7
    assert list(node._on_plan_path(request, PlanPath.Response()).paths) == []


def plan_path_request(**fields):
    request = PlanPath.Request()
    request.fill_type = PlanPath.Request.FILL_LINEAR
    request.distance = 0.2
    request.outline_count = 2
    request.outline = polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)])
    request.holes = [polygon([(3.0, 2.0), (4.5, 2.0), (4.5, 3.5), (3.0, 3.5)])]
    for k, v in fields.items():
        setattr(request, k, v)
    return request


def test_plan_path_concentric(node):
    request = plan_path_request(fill_type=PlanPath.Request.FILL_CONCENTRIC)
    res = node._on_plan_path(request, PlanPath.Response())
    # each part is mowed as a spiral, the loops around the bed come when the spiral gets there
    assert res.paths[0].is_outline == 1 and {p.is_outline for p in res.paths} == {0, 1}
    for piece in res.paths:
        check_path(piece.path)
    request.skip_fill = True
    assert {p.is_outline for p in node._on_plan_path(request, PlanPath.Response()).paths} == {1}


def test_plan_path_split(node):
    request = plan_path_request()
    whole = node._on_plan_path(request, PlanPath.Response()).paths
    assert [p.is_outline for p in whole] == [1, 0]
    assert node.set_parameters([Parameter('max_path_poses', value=50)])[0].successful
    split = node._on_plan_path(request, PlanPath.Response()).paths
    assert len(split) > 10
    joined = []
    for piece in split:
        pts = xy(piece.path)
        assert 3 <= len(pts) <= 50
        if joined and joined[-1][0] == piece.is_outline:
            # the next piece of a run starts on the pose the last one ended on
            assert pts[0] == joined[-1][1][-1]
            joined[-1][1].extend(pts[1:])
        else:
            joined.append((piece.is_outline, pts))
    assert [o for o, _ in joined] == [1, 0]
    for (_, pts), piece in zip(joined, whole):
        # all of it
        assert xy(piece.path) == pts
    # the least it takes: no piece gets fewer than the 3 poses OpenMower's FTC controller needs
    assert node.set_parameters([Parameter('max_path_poses', value=4)])[0].successful
    assert all(3 <= len(p.path.poses) <= 4 for p in node._on_plan_path(request, PlanPath.Response()).paths)


def test_plan_path_ignores_angle_offset_and_range(node):
    # OpenMower adds its mow_angle_offset to the request's angle itself
    assert all(r.successful for r in node.set_parameters([
        Parameter('angle_offset_deg', value=30.0), Parameter('angle_min_deg', value=40.0),
        Parameter('angle_max_deg', value=50.0)]))
    res = call_plan_coverage(node, use_angle=True, angle=0.0)
    assert res.success and math.radians(40.0) - 1e-9 <= res.angle <= math.radians(50.0) + 1e-9
    paths = node._on_plan_path(plan_path_request(angle=0.0, outline_count=0), PlanPath.Response()).paths
    assert [p.is_outline for p in paths] == [0]
    yaws = [quaternion_to_yaw(0.0, 0.0, ps.pose.orientation.z, ps.pose.orientation.w) for ps in paths[0].path.poses]
    # the lanes run along x
    assert sum(angle_diff(2 * yaw, 0.0) < 1e-6 for yaw in yaws) > 0.8 * len(yaws)


def test_plan_path_many_loops(node):
    # OpenMower's outline_count is a uint8, all of it is fine
    paths = node._on_plan_path(plan_path_request(outline_count=60), PlanPath.Response()).paths
    assert paths and paths[0].is_outline == 1
    for piece in paths:
        check_path(piece.path)
    # the loops go on until the middle is mowed
    assert min(math.hypot(x - 5.0, y - 3.0) for p in paths for x, y in xy(p.path)) < 1.5
    assert node.set_parameters([Parameter('perimeter_passes', value=255)])[0].successful
    assert node.set_parameters([Parameter('lane_overlap_passes', value=255)])[0].successful


def test_far_from_the_origin(node):
    # Point32 is float32: 20 km out it has 2 mm steps
    far = [(x + 10000.0, y + 20000.0) for x, y in GARDEN]
    res = call_plan_coverage(node, boundary=far, obstacles=[])
    assert res.success, res.message
    assert 'coordinates up to 20016 m from the map origin' in res.message and 'about 2 mm' in res.message
    assert 'float32' not in call_plan_coverage(node).message
    paths = node._on_plan_path(plan_path_request(outline=polygon(far), holes=[]), PlanPath.Response()).paths
    assert paths
    if not REAL_ROS:
        assert any(level == 'warning' and 'float32' in msg for level, msg in node.get_logger().lines)
    assert float32_warnings([[(8192.0, -8192.0)]]) == []
    assert 'about 1 mm' in float32_warnings([[(0.0, 0.0)], [(1.0, -8200.0)]])[0]
    assert 'about 500 mm' in float32_warnings([[(500000.0, 5500000.0)]])[0]


def test_plan_coverage_fingerprint_and_angles(node):
    res = call_plan_coverage(node)
    assert res.success, res.message
    assert len(res.fingerprint) == 64 and list(res.angles) == [res.angle]
    assert call_plan_coverage(node).fingerprint == res.fingerprint
    assert call_plan_coverage(node, mower_width=0.25).fingerprint != res.fingerprint


def test_plan_coverage_request_overrides(node):
    res = call_plan_coverage(node, overrides=[
        override('fill_pattern', 'crosshatch'), override('crosshatch_angle_deg', 60.0),
        override('angle_strategy', 'optimal'), override('angle_min_deg', 10.0), override('angle_max_deg', 20.0),
        override('perimeter_passes', 1), override('allow_reverse', False), override('turn_types', ['u_turn', 'pivot'])])
    assert res.success, res.message
    assert len(res.angles) == 2 and res.angles[0] == res.angle
    assert math.radians(10.0) - 1e-9 <= res.angle <= math.radians(20.0) + 1e-9
    assert angle_diff(2 * res.angles[1], 2 * (res.angle + math.radians(60.0))) < 1e-6
    assert 'bulb' not in res.message and 'k-turn' not in res.message and not any(res.segment_reverse)
    # the request's fields go on top of the overrides, the node's parameters stay as they were
    res = call_plan_coverage(node, overrides=[override('mower_width', 0.4)], mower_width=0.3)
    assert res.success and res.lane_spacing == pytest.approx(0.3 * 0.85)
    assert node.get_parameter('fill_pattern').value == 'lanes'
    assert len(call_plan_coverage(node).angles) == 1


@pytest.mark.parametrize('msg', [
    override('blade_height', 0.05), override('turn_radius', 1), override('fill_pattern', 'spiral'),
    override('mower_width', 6.0), override('turn_types', ['u_turn', 'zigzag']), override('angle_min_deg', math.inf),
    override('overlap', None), override('perimeter_passes', 256),
])
def test_plan_coverage_bad_overrides(node, msg):
    res = call_plan_coverage(node, overrides=[override('turn_radius', 0.3), msg])
    assert not res.success and msg.name in res.message
    if not REAL_ROS:
        assert node.publishers['~/coverage_path'].published == []


def test_split_lawn(node):
    # no way from one half to the other: two sections, the path jumps between them
    res = call_plan_coverage(node, boundary=LAWN, obstacles=[HEDGE])
    assert res.success and 'the area has 2 parts with no way between them' in res.message, res.message
    k = res.section_start[1]
    assert list(res.section_start) == [0, k] and 0 < k < len(res.path.poses)
    pts = xy(res.path)
    first, second = {x < 5.0 for x, _ in pts[:k]}, {x < 5.0 for x, _ in pts[k:]}
    assert len(first) == len(second) == 1 and first != second
    # OpenMower gets loops and lanes for each half, every path in one of them
    paths = node._on_plan_path(plan_path_request(outline=polygon(LAWN), holes=[polygon(HEDGE)]),
                               PlanPath.Response()).paths
    left = {p.is_outline for p in paths if max(x for x, _ in xy(p.path)) < 5.0}
    right = {p.is_outline for p in paths if min(x for x, _ in xy(p.path)) > 5.0}
    assert left == right == {0, 1} and len(paths) == 4
    for piece in paths:
        check_path(piece.path)


def test_plan_coverage_drivable_areas(node):
    # the way round the hedge is outside the lawn
    res = call_plan_coverage(node, boundary=LAWN, obstacles=[HEDGE],
                             drivable_areas=[polygon([(2.0, 4.0), (8.0, 4.0), (8.0, 10.0), (2.0, 10.0)])])
    assert res.success and 'jumps' not in res.message, res.message
    outside = [i for i, (x, y) in enumerate(xy(res.path)) if y > 6.0]
    assert outside
    for i in outside:
        k = max(k for k, s in enumerate(res.segment_start) if s <= i)
        assert res.segment_type[k] == PlanCoverage.Response.SEGMENT_TRANSIT


def test_plan_coverage_end_point(node):
    res = call_plan_coverage(node, use_end_point=True, end_point=Point(x=22.0, y=8.0, z=0.0),
                             overrides=[override('perimeter_order', 'last')])
    assert res.success, res.message
    last = res.path.poses[-1].pose.position
    assert math.hypot(last.x - 22.0, last.y - 8.0) < 1.0


def test_plan_from_parameters(node):
    res = node._on_plan_from_parameters(Trigger.Request(), Trigger.Response())
    assert not res.success  # nothing set yet
    flat = [float(c) for p in GARDEN for c in p]
    results = node.set_parameters([
        Parameter('boundary_polygon', Parameter.Type.DOUBLE_ARRAY, flat),
        Parameter('obstacles', Parameter.Type.STRING_ARRAY, ['15,2.5; 19,2.5; 19,5; 15,5']),
    ])
    assert all(r.successful for r in results)
    res = node._on_plan_from_parameters(Trigger.Request(), Trigger.Response())
    assert res.success, res.message
    if not REAL_ROS:
        published = node.publishers['~/coverage_path'].published
        assert len(published) == 1 and len(published[0].poses) > 100
        assert node.set_parameters([Parameter('publish_path', value=False)])[0].successful
        assert node._on_plan_from_parameters(Trigger.Request(), Trigger.Response()).success
        assert len(published) == 1
    node.set_parameters([Parameter('obstacles', Parameter.Type.STRING_ARRAY, ['1,2,3'])])
    res = node._on_plan_from_parameters(Trigger.Request(), Trigger.Response())
    assert not res.success and 'obstacles' in res.message


@pytest.mark.parametrize('name, value', [
    ('overlap', 1.0), ('mower_width', -0.1), ('lane_order', 'spiral'), ('angle_strategy', 'random'),
    ('turn_radius', -1.0), ('frame_id', ''), ('perimeter_passes', -2), ('fill_pattern', 'spiral'),
    ('crosshatch_angle_deg', math.nan), ('perimeter_order', 'middle'), ('perimeter_direction', 'up'),
    ('turn_types', ['u_turn', 'zigzag']), ('angle_min_deg', -math.inf), ('angle_max_deg', math.inf),
    ('angle_step_deg', 0.0), ('lane_overlap_passes', -1), ('max_path_poses', 3), ('perimeter_passes', 256),
    ('lane_overlap_passes', 256), ('headland_corners', 'round'),
])
def test_bad_parameters_are_refused(node, name, value):
    result = node.set_parameters([Parameter(name, value=value)])[0]
    assert not result.successful


def test_parameters_are_used(node):
    assert all(r.successful for r in node.set_parameters([
        Parameter('frame_id', value='garden'), Parameter('allow_reverse', value=False),
        Parameter('lane_order', value='skip'), Parameter('angle_strategy', value='fixed'),
        Parameter('fixed_angle_deg', value=45.0)]))
    res = call_plan_coverage(node)
    assert res.success, res.message
    check_path(res.path, 'garden')
    assert res.angle == pytest.approx(math.radians(45))
    assert not any(res.segment_reverse)


def test_new_parameters_are_used(node):
    assert all(r.successful for r in node.set_parameters([
        Parameter('fill_pattern', value='crosshatch'), Parameter('crosshatch_angle_deg', value=45.0),
        Parameter('angle_strategy', value='optimal'), Parameter('angle_step_deg', value=10.0),
        Parameter('angle_min_deg', value=30.0), Parameter('angle_max_deg', value=40.0),
        Parameter('perimeter_order', value='last'), Parameter('perimeter_direction', value='cw'),
        Parameter('turn_types', value=['k_turn', 'u_turn']), Parameter('lane_overlap_passes', value=1)]))
    res = call_plan_coverage(node)
    assert res.success, res.message
    assert len(res.angles) == 2 and math.radians(30.0) - 1e-9 <= res.angle <= math.radians(40.0) + 1e-9
    types = list(res.segment_type)
    assert types[-1] == PlanCoverage.Response.SEGMENT_PERIMETER and types[0] != types[-1]
    # a range needs both ends
    assert node.set_parameters([Parameter('angle_max_deg', value=math.nan)])[0].successful
    res = call_plan_coverage(node)
    assert not res.success and 'angle_min' in res.message


def test_headland_corners_parameter(node):
    assert node.config(node.values()).headland_corners == 'rounded'
    assert node.set_parameters([Parameter('headland_corners', value='sharp')])[0].successful
    assert node.config(node.values()).headland_corners == 'sharp'


def test_robot_outline_parameter(node):
    """robot_outline [x0, y0, x1, y1, ...]: the body's outline in place of the rectangle of robot_width, robot_front and
    robot_rear, also as a request's override; refused where it's no outline round base_link; empty: the rectangle."""
    assert node.config(node.values()).robot_outline is None
    flat = [0.43, -0.03, 0.43, 0.03, 0.26, 0.2, -0.14, 0.2, -0.14, -0.2, 0.26, -0.2]
    assert node.set_parameters([Parameter('robot_outline', value=flat)])[0].successful
    cfg = node.config(node.values())
    assert cfg.robot_outline == tuple(zip(flat[0::2], flat[1::2])) and cfg.body
    for bad in ([0.4, 0.2, -0.1], [1.0, 0.2, 0.5, 0.2, 0.5, -0.2], [0.4, 0.2, -0.1, -0.2, -0.1, 0.2, 0.4, -0.2]):
        assert not node.set_parameters([Parameter('robot_outline', value=bad)])[0].successful
    assert node.set_parameters([Parameter('robot_outline', Parameter.Type.DOUBLE_ARRAY, [])])[0].successful
    assert node.config(node.values()).robot_outline is None
    res = call_plan_coverage(node, boundary=LAWN, obstacles=[], overrides=[override('robot_outline', flat)])
    assert res.success, res.message
    res = call_plan_coverage(node, boundary=LAWN, obstacles=[], overrides=[override('robot_outline', [1.0, 0.2])])
    assert not res.success and 'robot_outline' in res.message


def test_parsing():
    assert parse_polygon('1,2; 3,4;5 6') == [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]
    assert pairs([0.0, 1.0, 2.0, 3.0]) == [(0.0, 1.0), (2.0, 3.0)]
    with pytest.raises(ValueError):
        parse_polygon('1,2,3')
    with pytest.raises(ValueError):
        pairs([1.0])
