#!/usr/bin/env bash
# Fresh simulation for one scenario. Existing baseline/UI entry points stay intact.

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
source "${MYDWA_WS}/scripts/_baseline_common.sh"
bench="${REPO_ROOT}/benchmark"
case "${1:-}" in
  mydwa) params_file="$bench/../config/nav2_params.yaml" ;;
  dwb) params_file="$bench/../config/nav2_params_dwb.yaml" ;;
  *) echo "Usage: $0 mydwa|dwb S01|S02|S03|S04|S05|S06" >&2; exit 2 ;;
esac
python3 "$bench/scenes.py" launch-args "${2:-}" >/dev/null
mapfile -t scene_args < <(python3 "$bench/scenes.py" launch-args "$2")
initial_pose=$(python3 "$bench/scenes.py" initial-pose "$2")
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
baseline_preflight
setsid ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=True use_rviz:=False use_sim_time:=True params_file:="$params_file" \
  "${scene_args[@]}" 9>&- &
launch_pid=$!
trap baseline_cleanup EXIT
trap 'exit 130' INT TERM
ready=false
for attempt in {1..40}; do
  kill -0 "$launch_pid" 2>/dev/null || exit 1
  if timeout 3 ros2 lifecycle get /amcl 2>/dev/null | grep -q 'active \[3\]'; then
    ready=true
    break
  fi
  sleep 1
done
[[ "$ready" == true ]] || { echo 'AMCL activation timeout' >&2; exit 1; }
# A request can reach AMCL while its CLI response is lost. Do not tear down a
# running, localized simulation solely on that reply. The runner independently
# requires valid map TF and both map/world start checks before sending any goal.
if ! timeout 15 ros2 service call /set_initial_pose nav2_msgs/srv/SetInitialPose "$initial_pose"; then
  echo '[benchmark] Initial-pose reply unavailable; runner TF/start preflight remains required.' >&2
fi
exec 9>&-
wait "$launch_pid"
