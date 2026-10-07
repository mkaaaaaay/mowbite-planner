# MowBite Planner

Coverage path planner for robot mowers, Python with shapely. It plugs into OpenMower in place of the slic3r planner
(same service, same answer), so `mower_logic` stays as it is. On top it checks the mower's whole body against the
real edges of the lawn.

Status: work in progress. Runs on my mower (OpenMower, YardForce NX60i, RTK) since early October, settings may
still change.

## What it does

- loops along the edge and around every obstacle, then lanes (or crosshatch, or rings). Lane spacing fixed or
  automatic, angle per area, optionally turning a bit further after every mow.
- forward-only turns: lanes in skip order so a U-turn fits, where it doesn't a small loop instead of turning on
  the spot (my lanes are ~15 cm apart, the smallest arc is 25 cm).
- options: `turn_on_spot` (turn on the spot instead of loops), `headland_turns` (clean stripes, all turning happens
  in the headland), `headland_corners` (rounded or sharp).
- settings per area in the map (`planner` property).

## Collision check

The map is recorded with the mower's centre driven close along every edge, so the real edge is where the body was
while recording. The planner rebuilds that by sweeping the body along the recorded lines and checks every pose and
every turn with the whole body (a rectangle, or the measured contour in `robot_outline`).

- margins: `edge_margin` (5 cm), `obstacle_margin` (10 cm), a `margin` per obstacle in the map, plus `spin_margin`
  (5 cm) for turns on the spot, because OpenMower's FTC controller starts turning a few cm before the turn point.
- where the body doesn't fit, only that place changes: another way, or turning back before a place that's too
  narrow. What can't be reached stays unmowed.
- no jumps between paths, `mower_logic` would drive those without any check.
- corners of drives and repair ways are rounded where the body fits.

The price: a few % of the lawn near edges and obstacles stays unmowed, and planning takes ~5-20 s per area on the
CM4 (plans are made ahead after every change). For the app the plan answer has `body_space` (where the body may be)
and `stats.body_fit` (where the plan was changed and why).

## Running it

- OpenMower (ROS 1): [`ros1/`](ros1/README.md), built into your own OpenMower image, all settings and RPCs there.
- ROS 2 node, MQTT service for Planner Studio, plot tool: [`ros2/`](ros2/README.md).

## Todo

- repair ways searched with arcs too
- fewer settings (one turn radius, one switch for loops or turning on the spot, K-turns out)
- a sanity check for `robot_outline`

## Repo

`ros2/` and `ros1/` are the planner and the nodes. `open_mower_ros/` (OpenMower's workspace with my fixes) and
`mowbite/` (the app) are copies kept alongside, not built from here.

Tests need only shapely and pytest:

```bash
python3 -m venv /tmp/venv && /tmp/venv/bin/pip install shapely pytest
/tmp/venv/bin/python -m pytest ros2/mowbite_planner/test ros1/mowbite_planner_ros1/test
```

## License

GPL-3.0, see [LICENSE](LICENSE).
