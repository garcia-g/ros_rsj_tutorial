#!/bin/bash
set -ue

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"

exit 0
