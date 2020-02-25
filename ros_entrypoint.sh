#!/bin/bash
set -e

# bring up can bus
/opt/overlay_ws/can0_bringup.sh

# setup ros2 environment
source "$ROS2_WS/install/local_setup.bash"
source "$DEPENDENCIES_WS/install/local_setup.bash"

file="$OVERLAY_WS/install/local_setup.sh"
if [ -f "$file" ]
then
  source "$file"
fi

case "$UDEV" in
	'1' | 'true')
		UDEV='on'
	;;
esac

exec "$@"
