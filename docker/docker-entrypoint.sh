#!/bin/bash
set -e

# Source ROS 2 base environment
source "/opt/ros/jazzy/setup.bash"

# Source workspace setup if built
if [ -f "/katana_ws/install/setup.bash" ]; then
    source "/katana_ws/install/setup.bash"
fi

exec "$@"
