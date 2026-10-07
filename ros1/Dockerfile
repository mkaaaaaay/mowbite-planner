# The MowBite Planner in a container of its own next to OpenMower's open_mower_ros (OM_PLANNER=external in its .env,
# see compose.yaml). Built on OpenMower's image of the same version: ROS, OpenMower's messages and its slic3r planner
# (for the setting planner slic3r) come from it, and its layers are on the mower already. Build context: this
# repository's root.
ARG OPENMOWER_IMAGE=ghcr.io/clemenselflein/open_mower_ros:edge
FROM ${OPENMOWER_IMAGE}

USER root
# shapely 2 (the newest for Python 3.8, its own GEOS), numpy comes with the image
RUN python3 -m pip install --no-cache-dir shapely==2.0.7
COPY ros1/mowbite_planner_ros1 /opt/mowbite/ros1/mowbite_planner_ros1
COPY ros2/mowbite_planner/mowbite_planner /opt/mowbite/ros2/mowbite_planner
USER openmower

# OpenMower's entrypoint sets up ROS and its workspace, the node comes from here (no catkin build: python only).
# --wait: OpenMower's master, never one of its own (it would take OpenMower's port)
CMD ["bash", "-c", "export ROS_PACKAGE_PATH=/opt/mowbite/ros1:$ROS_PACKAGE_PATH \
     PYTHONPATH=/opt/mowbite/ros2:$PYTHONPATH && exec roslaunch --wait mowbite_planner_ros1 mowbite_planner.launch \
     own_container:=true"]
