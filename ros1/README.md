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
- **Docking station.** Plans start on the loop next to it and end close to it.
- **Same map, same plan.** The plan never depends on where the mower is, so an interrupted mow resumes.

## Setup

Needs OpenMower's `edge` image (open_mower_ros #363 for `OM_PLANNER=external`, #364 so `mower_map` keeps the extra
area properties). Both are in `edge` since 8 October 2026, not in a release yet.

On OpenMowerOS v2:

1. OpenMower's `.env` (Dockge, stack `openmower`): `VERSION="edge"` and `OM_PLANNER=external`. OpenMower then
   doesn't start slic3r.
2. A stack of its own, e.g. `/opt/stacks/mowbite-planner/`, with [`compose.yaml`](compose.yaml). It builds the planner
   on the mower from this repository, on top of OpenMower's image.
3. `docker compose up -d` there (Dockge: Deploy), then restart OpenMower's stack.

Updating: `docker compose up -d --build`. Going back: `VERSION` as before, `OM_PLANNER` out of OpenMower's `.env`, the
planner stack stopped.

Settings and plans live where OpenMower keeps its ROS data (`/home/openmower/ros`). When OpenMower restarts, the
planner container ends and comes back on its own (`restart: unless-stopped`).

## First run

- [ ] A small, simple area, mower docked.
- [ ] `docker logs -f mowbite_planner 2>&1 | grep MowBite` says `MowBite Planner answers
  /slic3r_coverage_planner/plan_path`. If OpenMower's log still starts `slic3r_coverage_planner`,
  `OM_PLANNER=external` didn't arrive.
- [ ] Check the plan in the app (map, the area, "show mowing plan"): loops along the edge and around obstacles,
  then lanes. If the log says `MowBite Planner: <reason>` instead, the area isn't planned and the reason says why.
- [ ] Mow it, emergency stop at hand. If the mower struggles in the turns, give the area a larger `turn_radius`.

## Settings and RPCs

Set them in the MowBite app or over OpenMower's JSON-RPC (MQTT `rpc/request`), no new image needed. They're kept in
`/home/openmower/ros/mowbite_planner_settings.json` and used from the next plan on. An interrupted area whose plan
changes starts again.

Lane spacing, loops, `perimeter_offset` and `lane_overlap_passes` come from OpenMower's parameters (`tool_width`,
`outline_count`, ...) until you set them here. An area's own values in the map beat both. The blade width is
`mower_width` (or `OM_BLADE_WIDTH`).

| Method | What it does |
| --- | --- |
| `planner.settings` | every setting with value, default, type, unit, range, and where it comes from |
| `planner.settings.set` | `{"turn_radius": 0.3, ...}`, checked as a whole, `null` resets one |
| `planner.plan` | a plan like the app's preview, plus segments, stats, warnings and how long mowing it takes (`duration`, gets better with every finished mow); `report: true` adds a check (unmowed bits, where the body would stick out) |
| `planner.drive` | how close the body really got to the edges on a mow (from OpenMower's position history), also logged after every mow |
| `planner.leftovers` | what a mow really left (from its track) and the passes over it; with `arm: true` the next start of the mower only mows those |
| `planner.angle.reset` | puts the stripe rotation (`angle_increment`) back to the start |

```bash
mosquitto_pub -h <mower> -t rpc/request \
  -m '{"jsonrpc": "2.0", "id": "1", "method": "planner.settings.set", "params": {"turn_radius": 0.3}}'
```

A few worth knowing:

- `planner`: `mowbite` or `slic3r`. With `slic3r` OpenMower's own planner plans every area (no body check).
- `angle_increment` (rad): turns the stripes a bit every `angle_increment_every` finished mows, within an area's
  `angle_min`..`angle_max`. Leave OpenMower's `mow_angle_increment` at 0, else both add up.
- `lane_spacing_mode: auto`: the planner picks the lane spacing, `perimeter_passes: -1` the number of loops.
- `leftovers_auto`: after every finished mow the planner looks at what it left (like `planner.leftovers`). If the
  passes over that mow 0.3 m² or more, it sends the mower out again once it's idle in the dock. Not in an emergency
  and, with OpenMower's `rain_mode` on, not in the rain. Once per mow.
- Body: `robot_width`, `robot_front`, `robot_rear` (m from the middle of the drive wheels) or `robot_outline`
  (`[[x, y], ...]`, x ahead, y left). Blade: `blade_ahead`, `blade_offset`.

Plans are kept (the last 64) and every active area is planned ahead 5 s after the map or settings change, so the
preview and the next mow find their plan at once. The full list of settings is in
[`../ros2/README.md`](../ros2/README.md#parameters).

## Tests

```bash
python3 -m pytest ros1/mowbite_planner_ros1/test   # rospy and the messages are stood in for, shapely is enough
```

They run on Python 3.8 with shapely 1.7 (the ROS Noetic image) and on newer versions.
