# MowBite Planner

A coverage path planner for robot mowers as a ROS 2 node (Humble and Jazzy): loops along the edge and around
obstacles, then straight lanes with turns that keep the wheels rolling (arcs, K-turns) instead of spinning on the
spot, as a `nav_msgs/Path`. Besides its own service it answers OpenMower's `PlanPath`.

- [`ros2/`](ros2/README.md): the planner and the ROS 2 node, an MQTT service in a small container that plans for
  OpenMower's Planner Studio and previews the mower's areas without changing OpenMower, and
  `tools/plot_coverage.py`, which shows a plan without ROS.
- [`ros1/`](ros1/README.md): for OpenMower as it is today, a ROS 1 node that plans in place of its slic3r planner. It
  goes into your own build of OpenMower's image and runs with `OM_PLANNER=mowbite`, MowBite's "show mowing plan" then
  shows its plans.

Copies kept alongside, not built from here:

- [`open_mower_ros/`](open_mower_ros/): OpenMower's ROS workspace (main plus our fixes, `OM_PLANNER=mowbite` in the
  launch file, `mower_map` keeping unknown area properties). The planner goes into `src/lib/mowbite` (`ros1/` and
  `ros2/` of this repository), the submodules (`xbot_framework`, `xbot_driver_gps`, `ros_ntrip_client`,
  `slic3r_coverage_planner`) come from upstream.
- [`mowbite/`](mowbite/): the MowBite app as of its dev branch.

The tests need only shapely and pytest:

```bash
python3 -m venv /tmp/venv && /tmp/venv/bin/pip install shapely pytest
/tmp/venv/bin/python -m pytest ros2/mowbite_planner/test ros1/mowbite_planner_ros1/test
```

## License

GPL-3.0, see [LICENSE](LICENSE).
