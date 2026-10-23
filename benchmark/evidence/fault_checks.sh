#!/usr/bin/env bash
# Run after a fresh headless MyDWA launch. Explicitly resumes Gazebo on exit.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
bench=/home/user/ros2_ws/src/my_dwa_controller/benchmark
suffix=${1:-001}
python3 "$bench/run_trial.py" --trial-id "timeout_$suffix" --timeout 0.4 \
  > "$bench/evidence/timeout_$suffix.log" 2>&1 || test "$?" -eq 2
trap 'timeout 5 ros2 service call /unpause_physics std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true' EXIT
python3 "$bench/run_trial.py" --trial-id "clock_stall_$suffix" --goal -0.5 -0.5 0 \
  --timeout 20 --clock-stall-timeout 0.6 --cancel-timeout 2 \
  > "$bench/evidence/clock_stall_$suffix.log" 2>&1 &
trial_pid=$!
ready=false
for attempt in {1..200}; do
  if grep -q '^Trial ' "$bench/evidence/clock_stall_$suffix.log"; then ready=true; break; fi
  kill -0 "$trial_pid" 2>/dev/null || break
  sleep 0.1
done
if [[ "$ready" != true ]]; then
  cat "$bench/evidence/clock_stall_$suffix.log"
  exit 1
fi
sleep 0.4
timeout 5 ros2 service call /pause_physics std_srvs/srv/Empty '{}'
wait "$trial_pid" || test "$?" -eq 2
cat "$bench/evidence/timeout_$suffix.log" "$bench/evidence/clock_stall_$suffix.log"
