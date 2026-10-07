# MowBite Planner

A coverage path planner for OpenMower, written in Python (shapely). It replaces the slic3r planner: it answers the
same service with the same message, so `mower_logic` doesn't need any change. The main difference to slic3r is
that it plans with the mower's real size and checks that the body stays off the edges and obstacles.

**Status:** experimental. Tested on one mower (YardForce NX60i with RTK) and one garden. It mows, but expect rough
edges and settings that still change.

## Features

- loops along the edge and around obstacles, then lanes, crosshatch or rings
- automatic lane spacing, angle per area, optionally turning a bit further after every mow
- forward-only turns (lanes in skip order, a small loop where no U-turn fits), or turning on the spot if you want
- optional "clean stripes" mode: no turning inside the lanes, only in the headland
- collision check with the mower's body and adjustable margins (below)
- settings per area in the map, plans made ahead after every change, a preview for the app

## How the collision check works

You record the map by driving the mower's **centre** close along every edge and around every obstacle. The
recorded line is then where the centre was, and the real edge is where the body was. The planner rebuilds that
space by moving the body along the recorded lines, then checks every pose and every turn of the plan with the
whole body (a rectangle, or the measured outline of your mower) and keeps a margin to the edges.

Where the body doesn't fit, only that spot changes: the mower takes another way or turns back before a place that's
too narrow, and that bit stays unmowed. Turning on the spot and the turns between the lanes get some extra margin:
OpenMower's FTC controller starts turning a few cm before the turn point, and in a turn the mower keeps less
exactly to its path.

Expect a few % of the lawn near edges and obstacles to stay unmowed, depending on the margins.

## Setup on OpenMower

The full steps are in [`ros1/README.md`](ros1/README.md). In short:

1. Build your own OpenMower image with the planner in it (`ros1/` and `ros2/` copied into
   `open_mower_ros/src/lib/mowbite`, `ros1/openmower.patch` applied, OpenMower's Dockerfile for arm64).
2. Load it on the mower, point `compose.yaml` at it and set `OM_PLANNER=mowbite` and `OM_BLADE_WIDTH` in `.env`.
3. Set the body size (`robot_width`, `robot_front`, `robot_rear` from the middle of the rear axle) with
   `planner.settings.set` or in the MowBite app.
4. Record the map with the mower's centre along the edges.
5. Check the plan in the app before the first mow, and keep the emergency stop at hand.

Going back to slic3r: remove `OM_PLANNER` from `.env`. Keep a map backup before going back to the official image,
its `mower_map` drops the extra area properties.

Once open_mower_ros has `OM_PLANNER=external` and keeps the extra area properties (PRs #363 and #364), no own image
is needed: the planner runs as a container of its own next to the official image (`ros1/compose.yaml`, see
[`ros1/README.md`](ros1/README.md#a-container-of-its-own-no-own-openmower-image)).

## Main settings

| Setting | Default | |
| --- | --- | --- |
| `robot_width`, `robot_front`, `robot_rear` | none | body size in m around the rear axle, the check needs them |
| `robot_outline` | none | measured outline `[[x, y], ...]` instead of the rectangle |
| `edge_margin` | 0.05 | margin to the boundary (m) |
| `obstacle_margin` | 0.10 | margin to obstacles (m), `margin` on an obstacle in the map overrides it |
| `spin_margin` | 0.05 | extra margin when turning on the spot and in the turns between lanes (m) |
| `turn_radius` | 0.25 | radius of the turns (m) |
| `headland_turns` | false | clean stripes |
| `turn_on_spot` | false | turn on the spot instead of loops |
| `planner` | mowbite | `slic3r`: OpenMower's slic3r planner plans instead, for all areas (no body check) |

All settings, the RPCs and what the plan answer contains: [`ros1/README.md`](ros1/README.md),
[`ros2/README.md`](ros2/README.md).

## Known limits

- slower than slic3r: 5-20 s per area on the CM4, the plans are made ahead so you rarely wait
- the margins have to cover GPS and recording errors, ~5 cm with a good RTK fix. GPS jumps (house walls, trees)
  aren't covered
- obstacles closer to the edge or each other than the body plus margins can't be passed
- tufts stay in very pointed corners

## Development

The planner itself needs only shapely, the tests pytest:

```bash
python3 -m venv /tmp/venv && /tmp/venv/bin/pip install shapely pytest
/tmp/venv/bin/python -m pytest ros2/mowbite_planner/test ros1/mowbite_planner_ros1/test
```

`ros2/` also has a ROS 2 node, an MQTT service for OpenMower's Planner Studio and `tools/plot_coverage.py` to plot
a plan without ROS. `open_mower_ros/` and `mowbite/` are copies of my OpenMower workspace and the app, not built
from here.

## License

GPL-3.0, see [LICENSE](LICENSE).
