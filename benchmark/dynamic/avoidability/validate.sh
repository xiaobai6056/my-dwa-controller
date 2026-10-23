#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$here/validate_motion.py" "$@"