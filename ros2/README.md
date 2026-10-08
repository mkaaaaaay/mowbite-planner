# MowBite Planner (ROS 2)

The planner as a ROS 2 node (Humble and Jazzy, Python, shapely). It mows an area the way a careful person would:
a few loops along the edge and around every bed and tree, then straight lanes, with turns that keep the wheels
rolling instead of tearing up the lawn. Out comes a `nav_msgs/Path` in the `map` frame.

It doesn't need OpenMower or the MowBite app, but it fits OpenMower: besides its own service it answers OpenMower's
`PlanPath` field by field. For OpenMower as it is today (ROS 1) use [`../ros1`](../ros1/README.md).

## Packages

| Package | What's in it |
| --- | --- |
| `mowbite_planner_interfaces` | `PlanCoverage.srv`, OpenMower's `PlanPath.srv` and `Path.msg` (ament_cmake, a `.srv` needs it) |
| `mowbite_planner` | the planner (`planner.py` with `config.py`, `lanes.py`, `route.py`, `bodyfit.py`, `maneuvers.py`, `transit.py`, `footprint.py`; only shapely), the node, the MQTT service, `tools/plot_coverage.py`, tests |

The planner itself imports nothing from ROS, so it runs and plots on any PC with shapely.

## How it plans

1. **Loops** along the outline and around every obstacle, the first half a blade width in. Where a bed is close to
   the edge the loops go round both instead of crossing.
2. **What's left** after the loops. A strip too narrow for lanes gets one more loop down its middle.
3. **Lanes** across it, `mower_width × (1 − overlap)` apart, at the angle `angle_strategy` picks. `crosshatch` adds a
   second set across, `concentric` does loops all the way in.
4. **Route.** Lanes split by obstacles become cells, each mowed as a snake, the next cell the closest one not mowed
   yet. Drives take the shortest way, their centre 15 cm in from where it may go where that isn't a lot longer.
   With a docking station the plan ends close to it.

Turns, gentlest first: U-turn (lanes two radii apart), bulb (a loop of more than half a circle, forwards only),
smaller radii, detour along the edge, pivot on the spot. `lane_order: skip` mows every k-th lane first so every turn
is a plain forward U-turn; that's the mode for controllers that can't back up.

Parts with no way between them (a hedge across the lawn) are mowed one after the other as **sections**; the path
jumps between them and getting there is up to whoever drives it.

The same input always gives exactly the same poses (on the same shapely and GEOS version).

## The node

- **`~/plan_coverage`** (`PlanCoverage`): `boundary`, `obstacles`, `drivable_areas` (other lawns for drives
  between parts), `overrides` (any parameter, this request only), optional fixed `angle`, `start_pose` (the path
  starts on the loop next to it) and `end_point` (ends close to it). Answers the path, `section_start`, the angle,
  lane spacing, length, a SHA-256 `fingerprint` and the segments (loop, lane, turn, transit).
- **`~/plan_path`** (`PlanPath`): OpenMower's request and answer, field by field (`Path[] paths`, loops
  `is_outline` 1, lanes with their turns 0). Paths longer than `max_path_poses` (30000) are cut, OpenMower counts
  poses in an int16.
- **`~/plan_from_parameters`** (`Trigger`): plans `boundary_polygon` and `obstacles` from the parameters, for trying
  things out.
- **`~/coverage_path`** (`nav_msgs/Path`, transient local): every plan.

Poses are 0.1 m apart, each facing the next one, never two on the same spot (FTC would skip one).

## Build and run

```bash
cd ~/ros2_ws/src
ln -s /path/to/mowbite/ros2/mowbite_planner_interfaces /path/to/mowbite/ros2/mowbite_planner .
cd .. && rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to mowbite_planner && source install/setup.bash
ros2 launch mowbite_planner mowbite_planner.launch.py
```

```bash
ros2 service call /mowbite_planner/plan_coverage mowbite_planner_interfaces/srv/PlanCoverage "{
  boundary: {points: [{x: 0.0, y: 0.0}, {x: 20.0, y: 0.0}, {x: 20.0, y: 15.0}, {x: 0.0, y: 15.0}]},
  obstacles: [{points: [{x: 14.0, y: 2.0}, {x: 17.0, y: 2.0}, {x: 17.0, y: 4.0}, {x: 14.0, y: 4.0}]}]}"
```

## Parameters

All in `config/mowbite_planner.yaml` (doubles need a decimal point there).

| Parameter | Default | |
| --- | --- | --- |
| `frame_id` | `map` | frame of the polygons and the path |
| `mower_width` | 0.22 | blade width (m) |
| `overlap` | 0.15 | how much neighbouring lanes overlap (share of the width) |
| `perimeter_passes` | 2 | loops along the edge and around obstacles, -1: as many as the turns need |
| `perimeter_offset` | -1.0 | first loop's distance from the outline (m), negative: half the blade |
| `perimeter_order` | `first` | loops `first` or `last` |
| `perimeter_direction` | `auto` | `ccw`, `cw` or `auto` (the way the mower faces) |
| `lane_overlap_passes` | 0 | lanes reach this many loops further out (OpenMower's `outline_overlap_count`) |
| `fill_pattern` | `lanes` | `lanes`, `crosshatch` or `concentric` |
| `crosshatch_angle_deg` | 90.0 | angle of crosshatch's second lanes |
| `angle_strategy` | `longest_edge` | `longest_edge`, `min_width`, `optimal` (fewest lanes) or `fixed` |
| `fixed_angle_deg`, `angle_offset_deg` | 0.0 | lane direction for `fixed`, and an offset on top |
| `angle_min_deg`, `angle_max_deg` | `.nan` | keep the lanes in this range |
| `angle_step_deg` | 5.0 | step `optimal` tries |
| `turn_radius` | 0.25 | radius of the turns (m) |
| `turn_types` | all | `[u_turn, bulb, detour, pivot]`, allowed ones, preferred first |
| `turn_on_spot` | false | turn on the spot where it would drive a loop |
| `lane_order` | `snake` | `snake` or `skip` (forward U-turns only) |
| `route_order` | `nearest` | cells `nearest` first, or `optimized` (shortest drives) |
| `narrow_parts` | `lanes` | strips narrower than a U-turn: `lanes` anyway, or `loops` |
| `headland_turns` | false | clean stripes: turns and drives only in the headland |
| `headland_corners` | `rounded` | corners of headland drives `rounded` or `sharp` (on the spot) |
| `robot_width`, `robot_front`, `robot_rear` | -1.0 | body size (m from the middle of the drive wheels), negative: no body check |
| `robot_outline` | not set | the body's outline `[x0, y0, x1, y1, ...]` instead of the rectangle |
| `edges` | `recorded` | `recorded`: lines are where the centre drove; `hard`: walls |
| `edge_margin` | 0.05 | gap between body and the real edge (m) |
| `obstacle_margin` | 0.10 | gap to obstacles (m) |
| `spin_margin` | 0.05 | extra gap turning on the spot and in the turns between lanes (m) |
| `body_fit` | true | where the body would stick out, go another way (or leave the bit out) |
| `mop_up` | false | extra round at the end over the biggest unmowed bits |
| `blade_ahead`, `blade_offset` | 0.0 | where the blade sits relative to the mower's centre (m) |
| `bend_max_gap`, `bend_max_offset` | 0.0, 0.5 | let lanes bend round small obstacles up to this gap (m) |
| `transit_edge_distance`, `transit_edge_cost` | 0.0, 2.0 | make drives keep further off the edges where cheap |
| `publish_path` | true | publish every plan on `~/coverage_path` |
| `max_path_poses` | 30000 | `~/plan_path` cuts longer paths |

The MQTT service and the ROS 1 node take every planner setting (`PlannerConfig` in `config.py`), also
`lane_spacing_mode` (`auto` picks the spacing), `nested_areas`, `angle_increment` and friends.

## Without ROS

```bash
cd ros2/mowbite_planner
pip install shapely matplotlib pytest
python3 tools/plot_coverage.py                                       # a demo garden
python3 tools/plot_coverage.py --map map.json --area "Front lawn"   # an area of OpenMower's map.json
python3 -m pytest test/
```

Tests run against shapely 1.7 (Python 3.8, ROS Noetic), 1.8 and 2.x. A 300 m² garden plans in well under a second on
a PC, a Raspberry Pi CM4 takes four to six times as long.

## MQTT service

`mqtt_service.py` answers JSON-RPC 2.0 over MQTT like OpenMower's own RPC (`rpc/request`, `rpc/response`), with
OpenMower's map and parameters from the broker:

| Method | |
| --- | --- |
| `ext.mowbite_planner.plan` | `area_id` (or `outline`, `obstacles`, ...), optional `settings`, `poses`, `report`: a plan like `mowing.plan`, with what the mower would get |
| `ext.planner.plan` | Planner Studio's request (OpenMower's app), answered with its stages and stats |
| `ext.mowbite_planner.info` | version, settings, whether map and parameters are in |

`docker compose up -d` in `ros2/mowbite_planner/docker` runs it on the mower (host network, broker on
127.0.0.1:1883). `PLANNER_SETTINGS` takes a JSON object of planner settings, `compose.yaml` sets
`{"lane_order": "skip"}` for OpenMower's FTC.

## Known limits

- very pointed corners keep a small tuft
- loops follow the GPS jitter of a recorded outline point by point: thin out its points first (the MowBite map
  editor does)
- `geometry_msgs/Polygon` is float32, fine up to a few km from the map origin
