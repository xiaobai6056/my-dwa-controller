#!/usr/bin/env bash
# Run inside WSL Ubuntu 22.04. Uses the stock Nav2 default world/spawn pose.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$(dirname -- "${BASH_SOURCE[0]}")/_baseline_common.sh"
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

baseline_preflight
# A separate process group lets cleanup also find children after launch has exited.
setsid ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False 9>&- &
launch_pid=$!
trap baseline_cleanup EXIT
trap 'exit 130' INT TERM

# Wait for AMCL activation before initializing localization at the default pose.
ready=false
for attempt in {1..30}; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo 'Launch exited before AMCL became active.' >&2
    exit 1
  fi
  if timeout 3 ros2 lifecycle get /amcl 2>/dev/null | grep -q 'active \[3\]'; then
    ready=true
    break
  fi
  sleep 1
done
if [ "$ready" != true ]; then
  echo 'AMCL did not activate within the bounded startup check.' >&2
  exit 1
fi
timeout 15 ros2 service call /set_initial_pose nav2_msgs/srv/SetInitialPose \
  '{pose: {header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685]}}}'
# Release the startup lock so a subsequent UI/headless invocation can replace us.
exec 9>&-
wait "$launch_pid"
