# MowBite Planner for OpenMower (ROS 1 Noetic)

OpenMower, also on its new OS, runs on ROS 1 Noetic and plans every area with its slic3r planner through the
service `/slic3r_coverage_planner/plan_path`. `mowbite_planner_ros1` answers that service with the MowBite
Planner instead. It uses the same request and gives the same answer, so `mower_logic` and the `mowing.plan`
RPC (the app's preview) use it as they are. It runs in a container of its own next to OpenMower's, which doesn't
start slic3r with `OM_PLANNER=external` (see [Setup](#setup)).

What comes on top, without touching `mower_logic`:

- **Gentle turns.** OpenMower's FTC controller only drives forwards, so this node plans without K-turns and
  with `lane_order: skip`, which mows lanes k apart (k = ⌈2 · turn_radius / lane spacing⌉) and makes every
  turn a forward U-turn of `turn_radius`. A group of fewer than about 2k lanes gets bulb turns, also
  forwards only. Where the edge leaves no room it uses a smaller arc, a detour along the edge, or a pivot
  as the last resort.
- **Areas in areas.** A mowing area lying in another (90 % of it at least) is left out of that one and mowed
  round like an obstacle, it gets its own plan with its own settings (`nested_areas`, on by default). Areas
  next to each other with an edge in common don't count.
- **Narrow parts.** Where the lanes would lie in a part narrower than a U-turn (a path, a strip between
  beds), the loops go on in instead, and the pieces they come apart into are driven on the way round
  (`narrow_parts: loops` in the launch file's settings, `lanes` plans lanes there too). Areas without such
  parts keep their plan.
- **Clean stripes.** With `headland_turns: true` (off by default, for all areas with `planner.settings.set`, for
  one in its `planner` property) the path runs only straight along the lanes in the lanes' field: the drives between
  cells and parts, the turns between lanes far apart and the ways round the body go along the headland the loops
  mow, across the field only straight where that way is more than twice as long or there's none. Where nothing
  does it goes the way it would without: `stats.headland_turns` of a plan says where a drive turns in the field
  (`places` [x, y], also a warning) and how often it crosses it (`crossings`). With the body set, `headland_corners`
  `rounded` (the default) rounds the corners of the drives along the headland where the body fits on an arc (into the
  field only where none fits out of it, 12 cm deep at most: `arcs_in_field`, `arc_depth`), `sharp` turns on the spot
  there like before.
- **Turning on the spot.** With `turn_on_spot: true` (off by default, like `headland_turns` for all areas or one)
  the mower turns on the spot where the path would drive a loop: no loops out, no bulbs between
  lanes, drives arriving on the spot rather than round a loop, for lawns that take it. The body keeps its margins
  turning, like everywhere.
- **Settings per area.** The node reads OpenMower's map from `/mower_map_service/json_map` and finds the
  area of each request by its outline. An area's `planner` property can hold any MowBite Planner setting,
  for example:

  ```json
  "properties": {"name": "Front", "type": "mow", "planner": {"fill_pattern": "crosshatch",
                 "angle_strategy": "optimal", "perimeter_order": "last", "turn_radius": 0.3}}
  ```

  With `angle` or `angle_strategy` in it, the lanes go that way instead of the angle `mower_logic` sends
  (its `mow_angle_offset` included). An unknown name or a bad value there and the area isn't planned (no
  paths), the log says why. The area's `angle_min` and `angle_max` (rad) keep the lanes in that range,
  and mowing areas set to not mowable with `mow_around` that lie in it are mowed around. They need a
  `mower_map` that keeps the extra area properties (open_mower_ros #364, in `edge`), an older one drops them when it
  reads the map and they're gone after the next save.
- **Obstacles** come with the request, like for slic3r: a `mowing.plan` preview of an edited obstacle
  shows the edit. One with a `margin` property in the map (m, found by its outline) keeps the body that far off
  it in place of `obstacle_margin`, an area set to not mowable or inactive too.
- **Shortest ways** between the parts of an area, also through the other lawns, 15 cm further off the edges than
  the margins where that's not a lot longer (5 cm where it is). Not over navigation areas or areas set to not
  mowable: `mower_logic` mows a path from start to end, so a transit over a
  driveway would run with the blade on.
- **Sections.** Parts of an area with no way between them, not even through the other lawns (a hedge
  across the lawn, a passage narrower than the mower, a driveway in between), are each mowed completely,
  one after the other, as paths of their own. `mower_logic` drives to the start of every path itself.
- **Up to 255 loops** (`outline_count`), they stop where nothing is left.
- **The docking station.** The plan starts on the loop next to it and ends close to it.
- **Robust inputs.** Overlapping obstacles, obstacles over the edge and self-intersecting outlines work
  (OpenMower issues #345, #348, #46). Broken entries in the map are left out.
- **Path length.** OpenMower counts the poses of a path in an int16, so a longer run is cut into equal
  pieces of at most 30000 poses (`max_path_poses`, at least 4).

The plan never depends on where the mower is: the same map and settings give exactly the same poses. That
lets `mower_logic` resume an interrupted mow from its checkpoint. A new shapely or GEOS version (a new
image) can change the last bits of a coordinate or a vertex, an interrupted area then starts again.

Planning on Noetic's shapely 1.7 takes longer than with shapely 2. On a PC: a garden of 300 m² 0.07 s, a
lawn of 1700 m² with a recorded outline 1.5 s; a Raspberry Pi CM4 needs roughly four to six times as long.

## Setup

OpenMower's official image stays as it is, the planner runs next to it. It needs `OM_PLANNER=external` in
open_mower_ros (open_mower_ros #363) and, for an area's own settings and an obstacle's margin, a `mower_map` that
keeps the extra area properties (#364). Both are in OpenMower's `edge` image since 8 October 2026, not in a release
yet.

On OpenMowerOS v2:

1. In OpenMower's `.env` (Dockge, stack `openmower`): `OM_PLANNER=external` (open_mower_ros doesn't start slic3r
   then) and `VERSION="edge"`.
2. The planner as a stack of its own, e.g. `/opt/stacks/mowbite-planner/`: [`compose.yaml`](compose.yaml) and a `.env`
   with `OPENMOWER_VERSION` (the `VERSION` of OpenMower's `.env`, `edge`) and `OM_BLADE_WIDTH`. It builds
   the planner on the mower on that OpenMower image (ROS, OpenMower's messages and slic3r come from it, about 11 MB on
   top) from this repository. While it's private: `git clone` it into the stack's folder (e.g. as `src`) and set
   `context: ./src` in `compose.yaml`.
3. `docker compose up -d` there (Dockge: Deploy), then OpenMower's stack again. A newer planner or OpenMower version:
   `git pull` in the clone, then `docker compose up -d --build`.

Going back: `VERSION` as before and `OM_PLANNER` out of OpenMower's `.env`, the planner's stack stopped.

It keeps its settings and plans where OpenMower's container keeps its ROS data (`/home/openmower/ros`). When
OpenMower restarts (a new ROS master) the container ends and comes back (`restart: unless-stopped`) to register
again; it waits for OpenMower's master and never starts one of its own.

Back to slic3r: the setting `planner` `slic3r` (the MowBite app's switch), at once and for all areas, the planner
starts OpenMower's slic3r planner itself and hands it `mower_logic`'s request as it came. Or without the planner:
`OM_PLANNER` out of OpenMower's `.env` and the planner's stack stopped.

### First run

- [ ] A small, simple area, the mower docked, no mow interrupted.
- [ ] The planner's log (`docker logs -f mowbite_planner 2>&1 | grep MowBite`) says `MowBite Planner answers
  /slic3r_coverage_planner/plan_path` at the start, and OpenMower's (`docker logs open_mower_ros`) doesn't start
  `slic3r_coverage_planner`: else `OM_PLANNER=external` didn't arrive.
- [ ] The plan before mowing: in MowBite, Map, the area, mowing settings, "show mowing plan" says "from
  the mower". Or over MQTT:

  ```bash
  mosquitto_sub -h <mower> -t rpc/response &
  mosquitto_pub -h <mower> -t rpc/request \
    -m '{"jsonrpc": "2.0", "id": "1", "method": "mowing.plan", "params": {"area_id": "<area id>"}}'
  ```

  Loops along the edge and around the obstacles, then lanes. Every plan logs
  `MowBite Planner: area <id>, <n> paths, <n> poses, <n> m, lanes at <deg> deg, <ms> ms` and any warnings.
  `MowBite Planner: <reason>` instead: the area gets no paths and isn't mowed, the reason says why (e.g. a
  bad value in its `planner` property).
- [ ] Mow it, emergency stop at hand. The turns are forward arcs of `turn_radius` (0.25 m). If the mower
  struggles in them, give the area a larger one in its `planner` property, no new image needed.
- [ ] Afterwards: no uncut strips along the edge, around obstacles or between loops and lanes.

## Settings

Settings for all areas go in `launch/mowbite_planner.launch` (`settings`), they're part of the image. It
takes any name from `PlannerConfig` in `ros2/mowbite_planner/mowbite_planner/planner.py`. Changing one
changes the plans, so an interrupted mow starts its area again. Some come with every request of
`mower_logic`, a value in `settings` doesn't count for them: `mower_width` and `overlap` (from
`tool_width` and `OM_BLADE_WIDTH`), `perimeter_passes`, `perimeter_offset`, `lane_overlap_passes` and the
`skip_*` flags. `angle` too, unless `settings` has `angle` or `angle_strategy` and the area has no `angle`
of its own (then `mow_angle_offset` doesn't count). Whether it has one comes from the saved map: a preview
in the map editor (`mowing.plan` with the outline and the edited angle) of an area whose `angle` was just
added or removed follows the saved area until the edit is saved. An area's `planner` property can set all
of them, without a new image.

### Over RPC

Without a new image, through OpenMower's JSON-RPC (MQTT `rpc/request`, like `mowing.plan`):

- `planner.settings`: every setting with its value, default, whether it's stored, type, unit, range or
  choices, and whether it can be set here. The ones that come with `mower_logic`'s request say where from
  (`/mower_logic/tool_width`, ...). Settings of before (`min_turn_radius`, `smooth_spins`, `allow_reverse`,
  `waypoint_spacing`, ...) are left out where they still come, from the settings file or an app. `mower_width` is the
  blade and beats `OM_BLADE_WIDTH`. The body: `robot_width`, `robot_front`, `robot_rear` (m from `base_link`,
  the middle of the drive wheels), or its outline `robot_outline` (`type` `points`: `[[x, y], ...]` m round
  `base_link`, x ahead, y to the left, 3 at least; beats the three, null: the rectangle) and `edges` (`recorded`,
  `hard`, with the body set always `recorded`),
  `edge_margin` and `obstacle_margin` (m the body keeps off the real edges, an area's own in its `planner` property),
  `spin_margin` (m it keeps further off them turning on the spot and at a kink of more than 15°, anywhere on the last
  0.1 m of the way in, where FTC turns, and all along the turns between lanes; 0: only at the point), with
  `body_fit` (on) the path goes another way where the body would stick out (`stats.body_fit` of a plan says where,
  `poses` the pose there it sticks out the most at and how far, `body_space` the edges it keeps to), at a place too
  narrow to turn round in it turns round before it (the rest left out) instead of leaving the bit to `mower_logic`, the
  blade: `blade_ahead`, `blade_offset` (both may be negative). With the body set the corners of drives and of the ways
  round the body get arcs where the body fits on one, and a pose 2.5 cm before every turn on the spot left lets FTC
  turn at the point.
- `angle_increment` (rad, 0: off) turns the lanes on every `angle_increment_every` finished mows (`mower_logic`'s
  `JOB_COMPLETE` event), within an area's `angle_min`..`angle_max` (bouncing back at the ends); for one area in its
  `planner` property. The steps so far are kept next to the settings file, `planner.settings` says `angle_steps`,
  `angle_mows` (towards the next step) and `angle_turned` (rad), `planner.angle.reset` sets them back to 0. Leave
  `mower_logic`'s `mow_angle_increment` at 0, both add up.
- `planner` (not a planner setting, for all areas): `mowbite` or `slic3r`, OpenMower's slic3r planner plans then
  (see [Setup](#setup)), `planner.plan` asks it like `mower_logic` would and gives its paths as a plan (no body
  check).
- Plans are kept, the last 64 in `$ROS_HOME/mowbite_plans` (`/data/ros` in the container, parameter `plans_dir`), for
  this planner code only. 5 s after the map or the settings last changed every active mowing area is planned ahead
  like `planner.plan` plans it: the app's preview and the next mow find it at once (`kept: true` in `planner.plan`).
- `lane_spacing_mode: auto` (on and off again with `fixed`, also per area in its `planner` property) lets the
  planner pick the lane spacing, `perimeter_passes: -1` the loops (`planner.settings` reports `auto_value: -1`);
  what it picked is in `stats.chosen` of every plan.
- Lane spacing (`overlap`), loops (`perimeter_passes`, -1: as many as the lanes' turns need), `perimeter_offset`
  and `lane_overlap_passes` come from `mower_logic`'s parameters (`from` says which) until set here, then the
  planner's beat them (an area's own `outline_count`, ... still beat those).
- `planner.settings.set` with `{"turn_radius": 0.3, "fill_pattern": "crosshatch"}`: checked as a whole, a bad
  value changes nothing and the error says why. `null` goes back to the default. Kept in
  `$ROS_HOME/mowbite_planner_settings.json` (`/data/ros` in the container, it outlives a new image) and used
  from the next plan on, an interrupted area whose plan changes starts again. Answers like `planner.settings`.
- `planner.plan`: like the MQTT service's `ext.mowbite_planner.plan` (`area_id` or `outline`, `settings` to
  try before setting them, `poses`, `report`, `trace`), with the node's settings and `mower_logic`'s
  parameters. On top of `mowing.plan`'s answer: segments, warnings, stats and a fingerprint, with
  `report: true` a check of the plan (unmowed pieces, poses off the area, gaps, how much is mowed twice, and
  in `body_out` the poses `{x, y, yaw, kind}` where the body sticks out past the edges, `kind` one of `turn`,
  `spin` (turning on the spot), `lane_end`, `loop`, `transit`),
  with `trace: true` where every loop, lane and segment comes from. Without the increment `mower_logic`
  adds to the angle after every finished job.

```bash
mosquitto_pub -h <mower> -t rpc/request \
  -m '{"jsonrpc": "2.0", "id": "1", "method": "planner.settings.set", "params": {"turn_radius": 0.3}}'
```

## Tests

```bash
python3 -m pytest ros1/mowbite_planner_ros1/test   # rospy and the messages are stood in for, shapely is enough
```

They run on Python 3.8 with shapely 1.7 (Ubuntu 20.04, the ROS Noetic image) and on newer versions.
