# MowBite Planner for OpenMower (ROS 1 Noetic)

OpenMower, also on its new OS, runs on ROS 1 Noetic and plans every area with its slic3r planner through the
service `/slic3r_coverage_planner/plan_path`. `mowbite_planner_ros1` answers that service with the MowBite
Planner instead. It uses the same request and gives the same answer, so `mower_logic` and the `mowing.plan`
RPC (the app's preview) use it as they are. Only the launch file decides which planner runs.

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
  and mowing areas set to not mowable with `mow_around` that lie in it are mowed around. Stock
  `mower_map` drops the properties it doesn't know when it reads the map, so they never reach the planner
  and are gone after the next save. The patch below keeps them.
- **Obstacles** come with the request, like for slic3r: a `mowing.plan` preview of an edited obstacle
  shows the edit.
- **Shortest ways** between the parts of an area, also through the other lawns. Not over navigation
  areas or areas set to not mowable: `mower_logic` mows a path from start to end, so a transit over a
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

Without rebuilding OpenMower's image, the planner's [MQTT service](../ros2/README.md#mqtt-service) plans the
mower's areas on request (Planner Studio, `ext.mowbite_planner.plan`), but `mower_logic` keeps mowing
slic3r's plans then.

## Your own OpenMower image

The planner goes into OpenMower's workspace as a package of its own, so the mower needs an image built from
a copy of open_mower_ros with this repository in it. The image goes straight from the PC to the mower, no
registry involved.

### 1. Which OS the mower runs

Check on the mower:

| What you find | OS | Build from | The image is picked in | Settings go in |
| --- | --- | --- | --- | --- |
| `grep 'image:.*open_mower_ros' /opt/stacks/openmower/compose.yaml` finds a line | OpenMowerOS v2, Docker (Dockge on port 5001) | `docker/Dockerfile` | that `image:` line | `/opt/stacks/openmower/.env` |
| `systemctl cat openmower` starts it with `podman run` | OpenMowerOS v1 | `docker/Dockerfile.Legacy` | that `podman run` line | the file mounted as `/config/mower_config.sh` |
| `systemctl cat openmower` starts `openmower-nspawn-start` | OpenMower OS on Buildroot (A/B updates) | `docker/Dockerfile` | nowhere on the mower, see step 4 | `/data/openmower/openmower.conf` |

The steps are written for OpenMowerOS v2, the other two differ where noted.

### 2. Build the image (on a PC)

```bash
MOWBITE=~/mowbite-planner  # your clone of this repository
git clone --recursive https://github.com/ClemensElflein/open_mower_ros.git
cd open_mower_ros          # main, or check out the tag you want and run git submodule update --init --recursive

# ros1/ and ros2/ side by side: ros1 takes the planner from ../../ros2, catkin skips ros2 (CATKIN_IGNORE)
mkdir -p src/lib/mowbite
git -C "$MOWBITE" archive HEAD ros1 ros2 | tar -x -C src/lib/mowbite

# OM_PLANNER switch in the launch file, mower_map keeps the area properties it doesn't know
git apply src/lib/mowbite/ros1/openmower.patch
```

`git archive` takes what's committed, without caches (`cp -r` works too). The patch is made for main at
71dd04f: if it doesn't fit a newer version, `git checkout 71dd04f && git submodule update --init --recursive`
gives that one. Settings for all areas go in
`src/lib/mowbite/ros1/mowbite_planner_ros1/launch/mowbite_planner.launch` now (see [Settings](#settings)),
they're part of the image.

The mower is arm64. On an x86 PC, Docker needs arm64 emulation once (Docker Desktop has it built in):

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

Then build with OpenMower's own Dockerfile (`docker/Dockerfile.Legacy` for OpenMowerOS v1):

```bash
docker buildx build --platform linux/arm64 -f docker/Dockerfile \
  --build-arg OM_SOFTWARE_VERSION="$(git describe --tags --always)-mowbite" \
  -t local/open_mower_ros:mowbite --load .
```

Emulated, that's slow, it builds all of OpenMower. On an arm64 machine (a Mac with Apple silicon, a
Raspberry Pi 5) it runs natively. rosdep installs python3-shapely. `OM_SOFTWARE_VERSION` is the software
version OpenMower reports, without it the build says v0.0.0 (the build context leaves out `.git`).

Check that the planner is in it, this prints the package's path:

```bash
docker run --rm --platform linux/arm64 --entrypoint bash local/open_mower_ros:mowbite -c \
  'source /opt/open_mower_ros/devel/setup.bash && rospack find mowbite_planner_ros1 && python3 -c "import mowbite_planner"'
```

### 3. Copy it to the mower

```bash
docker save local/open_mower_ros:mowbite | gzip | ssh openmower@<mower> docker load
```

The image takes a few GB, check `df -h` on the mower first. Keep the official image there, it's the way
back. OpenMowerOS v1: `... | ssh <user>@<mower> sudo podman load`, podman calls it
`localhost/local/open_mower_ros:mowbite` (`sudo podman images`). On Buildroot skip this step.

### 4. Run it instead of the official image

On the mower, keep copies of the two files you change:

```bash
cd /opt/stacks/openmower
sudo cp compose.yaml compose.yaml.official
sudo cp .env .env.official
```

In `compose.yaml` the service with `image: ghcr.io/clemenselflein/open_mower_ros:...` gets
`image: local/open_mower_ros:mowbite` (the whole value, also if the tag comes from a variable). If it has
`pull_policy: always`, remove that line. In `.env`, next to `HARDWARE_PLATFORM` and `MOWER`:

```bash
OM_PLANNER=mowbite
OM_BLADE_WIDTH=0.18
```

`OM_BLADE_WIDTH` is the blade's cutting width. OpenMower's `tool_width` (in `mower_params.yaml`,
`OM_TOOL_WIDTH` on OpenMowerOS v1) stays the lane spacing. Then:

```bash
sudo docker compose up -d                        # new container from your image
docker ps --format '{{.Names}} {{.Image}}'        # its name
docker exec <name> printenv OM_PLANNER            # mowbite
```

Nothing printed: the container doesn't get `.env`. Add the two settings under the service's
`environment:` (`- OM_PLANNER=mowbite`) and run `sudo docker compose up -d` again. Dockge
(`http://<mower>:5001`) does the same in the browser. Don't pull or update this stack while it runs your
image, no registry has it.

- **OpenMowerOS v1:** in the `podman run` line of the unit `systemctl cat openmower` shows, replace the
  image with `localhost/local/open_mower_ros:mowbite` and drop a `podman pull` before it. Add
  `export OM_PLANNER=mowbite` and `export OM_BLADE_WIDTH=0.18` to the config file, then
  `sudo systemctl daemon-reload && sudo systemctl restart openmower`.
- **OpenMower OS on Buildroot:** ROS runs from `/opt/openmower-root`, a copy of the image made when the OS
  is built, read only. Build the OS around your image on the same PC, in a clone of
  [OpenMower](https://github.com/ClemensElflein/OpenMower) (`git clone --recursive`):
  `cd os && OMR_SOURCE=local-image OMR_IMAGE=local/open_mower_ros:mowbite make image`, and flash
  `output/images/sdcard.img`. That replaces everything on the mower, back up `/data/openmower` first.
  Update bundles (`.raucb`) you build later only install on a mower flashed with your keys (`os/keys/`).
  Settings: `OM_PLANNER=mowbite` and `OM_BLADE_WIDTH=0.18` in `/data/openmower/openmower.conf` (no
  `export`), then `systemctl restart openmower`.

### 5. Going back

- **slic3r with your image:** remove `OM_PLANNER` from `.env` and `sudo docker compose up -d` (on the
  other two OS from the config file, then restart).
- **The official image:** `sudo cp compose.yaml.official compose.yaml && sudo cp .env.official .env &&
  sudo docker compose up -d`. Its `mower_map` drops the area properties it doesn't know (`planner`,
  `angle_min`, `angle_max`, `mow_around`) and saves the map without them next time: keep a map backup.

Every switch changes the plans, an interrupted mow then starts its area again.

### First run

- [ ] A small, simple area, the mower docked, no mow interrupted.
- [ ] The log (`docker logs -f <name> 2>&1 | grep MowBite`, on the other two OS
  `journalctl -u openmower -f`) says `MowBite Planner answers /slic3r_coverage_planner/plan_path` at the
  start. If not, `OM_PLANNER` didn't arrive and slic3r plans.
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
  (`/mower_logic/tool_width`, ...). `allow_reverse` (K-turns: backing up between lanes and in place of a turn on
  the spot where no loop out fits) is off: OpenMower's FTC controller with `forward_only` stops on a backing up
  leg until it times out, it needs the change that lets it back up where the plan does. `mower_width` is the
  blade and beats `OM_BLADE_WIDTH`. The body: `robot_width`, `robot_front`, `robot_rear` (m from `base_link`,
  the middle of the drive wheels) and `edges` (`recorded`, `hard`), the blade: `blade_ahead`, `blade_offset`
  (both may be negative).
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
