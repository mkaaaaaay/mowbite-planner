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
exactly to its path. Drives between the parts of the plan keep their centre 15 cm in from where it may go where
that's not a lot longer (5 cm where it is), the body some 5-10 cm further off the edges than before.

Expect a few % of the lawn near edges and obstacles to stay unmowed, depending on the margins.

## Setup on OpenMower

OpenMower's official image stays as it is, the planner runs in a container of its own next to it. The full steps are
in [`ros1/README.md`](ros1/README.md#setup). In short:

1. In OpenMower's `.env`: `OM_PLANNER=external`, with `VERSION="edge"` (open_mower_ros #363 and #364, in `edge`
   since 8 October 2026, not in a release yet).
2. The planner as a stack of its own with [`ros1/compose.yaml`](ros1/compose.yaml), built on the mower on the same
   OpenMower image (about 11 MB on top), `docker compose up -d`.
3. Set the body size (`robot_width`, `robot_front`, `robot_rear` from the middle of the rear axle) with
   `planner.settings.set` or in the MowBite app.
4. Record the map with the mower's centre along the edges.
5. Check the plan in the app before the first mow, and keep the emergency stop at hand.

Back to slic3r: the setting `planner` `slic3r` (the app's switch), or `OM_PLANNER` out of OpenMower's `.env` and the
planner's stack stopped (both would answer as `slic3r_coverage_planner`).

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
| `mop_up` | false | a round at the end over the biggest unmowed pieces the blade gets to |
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
a plan without ROS.

## License

GPL-3.0, see [LICENSE](LICENSE).
