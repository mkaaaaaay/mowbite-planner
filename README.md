# MowBite Planner

A coverage path planner for OpenMower, in Python with shapely. It drops in for the slic3r planner: same service,
same answer, so `mower_logic` doesn't notice. The big difference: it plans with the mower's real body and keeps it
off the edges and obstacles.

**Status:** experimental. Tested on one mower (YardForce NX60i, RTK) in one garden. It mows, but expect rough edges
and settings that still change.

## What it does

- loops along the edge and around obstacles, then lanes (or crosshatch, or rings)
- forward-only turns that keep the wheels rolling, turning on the spot only if you want it
- every pose and every turn checked against the body, with margins you set
- picks the lane spacing and angle on its own if you let it, turns the stripes a bit after every mow
- "clean stripes" mode: no turning inside the lanes, only in the headland
- optional extra round at the end over the biggest bits left unmowed (`mop_up`), and after a mow a look at the real
  track: what did it leave, mow just that again (`planner.leftovers`, by itself with `leftovers_auto`)
- settings per area in the map, plans made ahead so the app's preview is instant

## The body check in short

You record the map by driving the mower's **centre** along the edges. So the line is where the centre was, and the
real edge is where the body was. The planner rebuilds that from the lines, checks the whole body (a rectangle or the
measured outline) at every pose and turn, and keeps a margin. Turning on the spot and the turns between lanes get a
few cm extra, because OpenMower's FTC starts turning early and wobbles a bit there.

Where the body doesn't fit, only that spot changes: another way round, or turning back before it. That bit stays
unmowed. In our garden that's about 2 % of the lawn, mostly thin strips in corners.

## Setup

OpenMower's official `edge` image stays as it is, the planner runs in its own container next to it. Full steps in
[`ros1/README.md`](ros1/README.md#setup), the short version:

1. OpenMower's `.env`: `VERSION="edge"` and `OM_PLANNER=external` (needs open_mower_ros #363 and #364, both in `edge`
   since 8 October 2026, not in a release yet).
2. Start the planner stack from [`ros1/compose.yaml`](ros1/compose.yaml) (`docker compose up -d`, about 11 MB on top
   of OpenMower's image).
3. Tell it the body size (`robot_width`, `robot_front`, `robot_rear` from the middle of the rear axle) in the MowBite
   app or with `planner.settings.set`.
4. Record the map with the mower's centre along the edges.
5. Look at the plan in the app before the first mow, emergency stop at hand.

Back to slic3r: switch `planner` to `slic3r` in the app. Or take `OM_PLANNER` out of OpenMower's `.env` and stop the
planner stack (both, else two planners answer the same service).

## Main settings

| Setting | Default | |
| --- | --- | --- |
| `robot_width`, `robot_front`, `robot_rear` | none | body size in m around the rear axle, the check needs them |
| `robot_outline` | none | measured outline `[[x, y], ...]` instead of the rectangle |
| `edge_margin` | 0.05 | gap to the boundary (m) |
| `obstacle_margin` | 0.10 | gap to obstacles (m), a `margin` on an obstacle in the map beats it |
| `spin_margin` | 0.05 | extra gap when turning on the spot and in the turns between lanes (m) |
| `obstacle_passes` | -1 | loops round the islands in the lawn, -1: as many as along the edge, 0: none |
| `turn_radius` | 0.25 | radius of the turns (m) |
| `turn_radius_auto` | false | the planner picks each turn's radius, `turn_radius` at least: fewer tight turns, wider ones where there's room |
| `turn_on_spot` | false | turn on the spot instead of loops |
| `headland_turns` | false | clean stripes |
| `split_zones` | false | parts of the lawn better mowed another way get their own lane direction |
| `mop_up` | false | extra round at the end over the biggest unmowed bits |
| `leftovers_auto` | false | after a mow the mower goes out again by itself for what it really left, from 0.3 m² |
| `planner` | mowbite | `slic3r`: OpenMower's slic3r plans instead, for all areas (no body check) |

Everything else, the RPCs and what a plan answer contains: [`ros1/README.md`](ros1/README.md) and
[`ros2/README.md`](ros2/README.md).

## Known limits

- slower than slic3r: 5–20 s per area on the CM4, but plans are made ahead, so you rarely wait
- the margins have to cover GPS and recording errors, about 5 cm with a good RTK fix. GPS jumps (house walls, trees)
  aren't covered
- obstacles closer to the edge or to each other than body plus margins can't be passed
- thin strips stay in tight corners and along edges the mower can only reach head-on

## Development

The planner only needs shapely, the tests pytest:

```bash
python3 -m venv /tmp/venv && /tmp/venv/bin/pip install shapely pytest
/tmp/venv/bin/python -m pytest ros2/mowbite_planner/test ros1/mowbite_planner_ros1/test
```

`ros2/` also has a ROS 2 node, an MQTT service for OpenMower's Planner Studio and `tools/plot_coverage.py` to draw a
plan without ROS.

## License

GPL-3.0, see [LICENSE](LICENSE).
