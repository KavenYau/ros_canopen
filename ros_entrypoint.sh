#!/bin/bash
set -e

# setup ros2 environment
file="$CANOPEN_WS/install/setup.sh"
if [ -f "$file" ]
then
  . "$file"
# else
  # NOTE(sam): don't source dependencies to enable npm install for rlcnodejs
  . $ROS2_WS/install/setup.sh
  . $DEPENDENCIES_WS/install/setup.sh
fi

exec "$@"
