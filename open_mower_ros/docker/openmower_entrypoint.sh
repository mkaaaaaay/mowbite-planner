#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash

# setup om environment
source /opt/open_mower_ros/version_info.env

# OSv2 debugging get controlled via env var DEBUG and has the ROSCONSOLE_CONFIG_FILE embedded
shopt -s nocasematch
case "${DEBUG:-0}" in
    1|true|yes|on|y)
        export ROSOUT_DISABLE_FILE_LOGGING=False
        unset ROSCONSOLE_CONFIG_FILE
    ;;
    *)
        export ROSCONSOLE_CONFIG_FILE=/config/rosconsole.config
        export ROSOUT_DISABLE_FILE_LOGGING=True
    ;;
esac
shopt -u nocasematch || true

# LOG_LEVEL=INFO (or DEBUG) shows more than the warnings and errors of the default config, without the file
# logging DEBUG=1 switches on. For tests that look for certain log lines
case "${LOG_LEVEL^^}" in
    DEBUG|INFO|WARN|ERROR|FATAL)
        if [ -n "${ROSCONSOLE_CONFIG_FILE:-}" ]; then
            printf 'log4j.threshold=%s\n' "${LOG_LEVEL^^}" > /tmp/rosconsole.config
            export ROSCONSOLE_CONFIG_FILE=/tmp/rosconsole.config
        fi
    ;;
esac

# Ensure stdout and stderr are unbuffered to get logging in real time order
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
export PYTHONUNBUFFERED=1

exec -- "$@"
