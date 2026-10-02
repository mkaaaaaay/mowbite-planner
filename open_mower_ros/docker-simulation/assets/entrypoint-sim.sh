#!/bin/bash
set -e

# HEADLESS=1: only the simulation node, no virtual desktop, VNC or rviz. For automatic tests, it saves most of
# the CPU. Its output goes straight to docker logs then.
if [ "${HEADLESS:-0}" = 1 ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/open_mower_ros/devel/setup.bash
    exec roslaunch /opt/mower_simulation_headless.launch --screen
fi

# GPU passthrough is opt-in (see docker-compose.yaml). If the host didn't map a render
# node in, fall back to software rendering so rviz still comes up everywhere.
if [ -d /dev/dri ] && ls /dev/dri/render* >/dev/null 2>&1; then
    echo "[entrypoint-sim] GPU render node found under /dev/dri - using hardware-accelerated rendering"
    export LIBGL_ALWAYS_SOFTWARE=0
else
    echo "[entrypoint-sim] No GPU render node found - falling back to software rendering (llvmpipe)"
    export LIBGL_ALWAYS_SOFTWARE=1
fi

exec /usr/bin/supervisord -n -c /etc/supervisor/conf.d/simulation.conf
