# MowBite Planner for OpenMower (ROS 1 Noetic)

OpenMower plans every area through `/slic3r_coverage_planner/plan_path`. `mowbite_planner_ros1` answers that service
with the MowBite Planner instead: same request, same answer, so `mower_logic` and the app's preview (`mowing.plan`)
just use it. It runs in its own container next to OpenMower's.

## What you get on top

- **Forward-only turns.** OpenMower's FTC only drives forwards, so lanes are mowed in skip order (every k-th first)
  and every turn is a forward U-turn of `turn_radius`. Tight spots get a bulb, a smaller arc, a detour along the
  edge, or a pivot as the last resort.
- **The body check** with margins, see the [main README](../README.md#the-body-check-in-short).
- **Settings per area.** An area's `planner` property in the map takes any planner setting:

  ```json
  "properties": {"name": "Front", "type": "mow", "planner": {"fill_pattern": "crosshatch", "turn_radius": 0.3}}
  ```

  An obstacle's `margin` property (m) beats `obstacle_margin` for that one. A bad value there and the area gets no
  plan, the log says why.
- **Areas in areas.** A mowing area inside another is mowed round like an obstacle and gets its own plan.
- **Narrow parts.** Where lanes would lie in a strip narrower than a U-turn, loops go on in there instead.
- **Clean stripes** (`headland_turns`) and **turning on the spot** (`turn_on_spot`), for all areas or per area.
- **Sensible drives.** The shortest way between parts of an area, also through other lawns, never over navigation
  areas or areas set to not mowable (the blade stays on). Parts with no way between them get their own paths.
- **Navigation areas next to a lawn.** The body may reach over the edge into one (the path stays on the lawn), so a
  bed close to that edge is mowed round. Where only a real edge is beyond, a slot too narrow for the body is closed
  if going round the bed leaves less unmowed.
- **Docking station.** Plans start on the loop next to it and end close to it.
- **Same map, same plan.** The plan never depends on where the mower is, so an interrupted mow resumes.

## Setup

Needs OpenMower's `edge` image (open_mower_ros #363 for `OM_PLANNER=external`, #364 so `mower_map` keeps the extra
area properties). Both are in `edge` since 8 October 2026, not in a release yet.

On OpenMowerOS v2, in this order. The planner has to answer before OpenMower drops slic3r: with `OM_PLANNER=external`
and no planner, `mower_logic` gives up after 60 s and starts again every 70 s, and the mower does nothing.

1. OpenMower on `edge`: `VERSION="edge"` in its `.env` (Dockge, stack `openmower`), restarted. Still with slic3r.
2. A stack of its own, e.g. `/opt/stacks/mowbite-planner/`, with [`compose.yaml`](compose.yaml). It builds the planner
   on the mower from this repository, on top of OpenMower's image. `docker compose up -d` there (Dockge: Deploy) and
   wait until `docker logs -f mowbite_planner` says `MowBite Planner answers /slic3r_coverage_planner/plan_path`.
3. Only then `OM_PLANNER=external` in OpenMower's `.env`, and OpenMower's stack restarted (`docker compose up -d`).
   OpenMower then doesn't start slic3r, the planner container starts again with it and answers.

Updating: `docker compose up -d --build`. After an OpenMower update too, then `docker image prune -f`: the planner
image is built on OpenMower's, the old one would keep the old OpenMower image (GBs) on the disk.

Going back, the other way round: `OM_PLANNER` out of OpenMower's `.env`, OpenMower restarted (`VERSION` as before if
you like), then the planner stack stopped.

Settings and plans live where OpenMower keeps its ROS data (`/home/openmower/ros`). When OpenMower restarts, the
planner container ends and comes back on its own (`restart: unless-stopped`).

## First run

- [ ] A small, simple area, mower docked.
- [ ] `docker logs -f mowbite_planner 2>&1 | grep MowBite` says `MowBite Planner answers
  /slic3r_coverage_planner/plan_path`. If OpenMower's log still starts `slic3r_coverage_planner`,
  `OM_PLANNER=external` didn't arrive.
- [ ] Your mower's size set (the app's planner menu, or `robot_width`, `robot_front`, `robot_rear`), the blade too
  (`mower_width`, `blade_ahead`). Until then the log says `no body check` with every plan.
- [ ] Check the plan in the app (map, the area, "show mowing plan"): loops along the edge and around obstacles,
  then lanes. If the log says `MowBite Planner: <reason>` instead, the area isn't planned and the reason says why.
- [ ] Mow it, emergency stop at hand. If the mower struggles in the turns, give the area a larger `turn_radius`.

## Settings and RPCs

Set them in the MowBite app or over OpenMower's JSON-RPC (MQTT `rpc/request`), no new image needed. They're kept in
`/home/openmower/ros/mowbite_planner_settings.json` and used from the next plan on. An interrupted area whose plan
changes starts again.

Loops, `perimeter_offset` and `lane_overlap_passes` come from OpenMower's parameters (`outline_count`, ...) until you
set them here. An area's own values in the map beat both. The lane spacing the planner picks itself from the blade
width, `mower_width` (or `OM_BLADE_WIDTH`). With `lane_spacing_mode: fixed` it's OpenMower's `tool_width` until you
set `overlap`.

| Method | What it does |
| --- | --- |
| `planner.settings` | every setting with value, default, type, unit, range, and where it comes from |
| `planner.settings.set` | `{"turn_radius": 0.3, ...}`, checked as a whole, `null` resets one |
| `planner.plan` | a plan like the app's preview, plus segments, stats, warnings and how long mowing it takes (`duration`, gets better with every finished mow); `report: true` adds a check (unmowed bits, their outlines in `left` to draw them, where the body would stick out) |
| `planner.drive` | how close the body really got to the edges on a mow (from OpenMower's position history), also logged after every mow |
| `planner.leftovers` | what a mow really left (from its track) and the passes over it; with `arm: true` the next start of the mower only mows those |
| `planner.simulate` | a plan driven by a simulated mower (OpenMower's controller and mower_logic, the errors of a real one with RTK): what it would leave, where the body would get into the margins, how long it takes; `model` picks `realistic` (the default), `ideal` or `omsim`, `track: true` adds its track and events |
| `planner.angle.reset` | puts the stripe rotation (`angle_increment`) back to the start |
| `planner.log` | the planner's last 2000 log lines since it started (`since`: only newer ones, `limit`), e.g. for an app to download |

```bash
mosquitto_pub -h <mower> -t rpc/request \
  -m '{"jsonrpc": "2.0", "id": "1", "method": "planner.settings.set", "params": {"turn_radius": 0.3}}'
```

A few worth knowing:

- `planner`: `mowbite` or `slic3r`. With `slic3r` OpenMower's own planner plans every area (no body check).
- `angle_increment` (rad): turns the stripes a bit every `angle_increment_every` finished mows, within an area's
  `angle_min`..`angle_max`. Leave OpenMower's `mow_angle_increment` at 0, else both add up.
- `perimeter_direction: random`: the loops go round one way or the other at random, drawn again after every finished
  mow, the same for all areas of a mow. Resuming it and the preview go the same way.
- With the body's outline (`robot_outline`, a model's contour in the app) the outermost loop goes the way the edge
  was recorded (with `auto` and `random`): the body stays where it was while recording, a long front gets round the
  bends of the edge. The other loops go as set.
- `lane_spacing_mode: auto`: the planner picks the lane spacing, `perimeter_passes: -1` the number of loops.
- `lane_overlap_limit`: on the lanes and the turns between them the mower gets no further out than the lanes reach
  (`lane_overlap_passes`, OpenMower's `outline_overlap_count`), so it keeps further off the edge there. Without it
  the turns take the room there is in the headland, which looks like more overlap than set. Best with an overlap of
  2: with 0 or 1 about half of the turns are on the spot then.
- `leftovers_auto`: after every finished mow the planner looks at what it left (like `planner.leftovers`). If the
  passes over that mow 0.3 m² or more, it sends the mower out again once it's idle in the dock. Not in an emergency
  and, with OpenMower's `rain_mode` on, not in the rain. Once per mow.
- Body: `robot_width`, `robot_front`, `robot_rear` (m from the middle of the drive wheels) or `robot_outline`
  (`[[x, y], ...]`, x ahead, y left). Without them (or with only a part) the body isn't checked: every plan warns of
  it, in its `warnings` and the log. Blade: `blade_ahead`, `blade_offset`.

Plans are kept (the last 16, the ones of an older planner version go when it starts) and every active area is planned
ahead 5 s after the map or settings change, so the preview and the next mow find their plan at once. The full list of
settings is in [`../ros2/README.md`](../ros2/README.md#parameters).

## Tests

```bash
python3 -m pytest ros1/mowbite_planner_ros1/test   # rospy and the messages are stood in for, shapely is enough
```

They run on Python 3.8 with shapely 2.0.7 (what the planner's image has) and on newer versions. ROS Noetic's own
shapely 1.7 isn't enough any more: it plans, but worse, and 8 of the tests fail.
