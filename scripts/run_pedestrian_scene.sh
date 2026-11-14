#!/usr/bin/env bash
set -eo pipefail

usage() { printf '%s\n' 'Usage: run_pedestrian_scene.sh {STATIC|EGO|PERSON|MULTI} [--gui]'; }
if [[ ${1:-} == --help || ${1:-} == -h ]]; then usage; exit 0; fi
scenario=${1:-}
case $scenario in
    STATIC|EGO|PERSON) motion=V01 ;;
    MULTI) motion=C ;;
    *) usage >&2; exit 2 ;;
esac
shift
gui=false
if [[ ${1:-} == --gui ]]; then gui=true; shift; fi
(($# == 0)) || { usage >&2; exit 2; }

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export R2_DEMO=${R2_DEMO:-"$HOME/ros2_experiments/r2_vision_lidar_20261112"}
export SORT_DEMO=${SORT_DEMO:-"$HOME/ros2_experiments/sort_ros2_prototype_20261112"}
export STATE_DEMO=${STATE_DEMO:-"$HOME/ros2_experiments/state_estimation_compare_20261112"}
export LIDAR_STATE_BIN=${LIDAR_STATE_BIN:-"$STATE_DEMO/.cache/lidar-build/lidar_state"}
export VISUAL_PROTOTYPE=${VISUAL_PROTOTYPE:-"$repo/visual_prototype"}
out=${PED_FUSION_OUT:-"$HOME/ros2_experiments/final_mydwa/local_runs"}
prefix=${PED_FUSION_PREFIX:-"$HOME/ros2_ws/install"}
results=${PED_FUSION_RUNS:-"$out/runs"}
[[ -f $prefix/local_setup.bash ]] || {
    echo "Build my_dwa_controller and set PED_FUSION_PREFIX to its install base." >&2
    exit 1
}
source "$SORT_DEMO/environment.sh"
source "$prefix/local_setup.bash"
ros2 pkg executables my_dwa_controller | grep -qx 'my_dwa_controller pedestrian_fusion' || {
    echo "Build my_dwa_controller and set PED_FUSION_PREFIX to its install base." >&2
    exit 1
}
[[ -x $LIDAR_STATE_BIN ]] || {
    echo "Build the R2 LiDAR adapter in STATE_DEMO first. See the R2 README." >&2
    exit 1
}

# Reserve the same domain range and lock files as the original R2 launcher.
mkdir -p "$R2_DEMO/.cache/locks" "$results"
for domain in {50..59}; do
    exec 9> "$R2_DEMO/.cache/locks/$domain.lock"
    flock -n 9 || continue
    base=$((7400 + 250 * domain))
    port=$((11800 + domain))
    if ss -H -lun | awk -v lo="$base" '{n=split($4,a,":"); p=a[n]+0; if(lo<=p && p<lo+250) used=1} END{exit !used}'; then continue; fi
    if [[ -n $(ss -H -ltn "sport = :$port") ]]; then continue; fi
    export ROS_DOMAIN_ID="$domain" GAZEBO_MASTER_URI="http://127.0.0.1:$port"
    reserved=true
    break
done
[[ ${reserved:-false} == true ]] || { echo "No free R2 ROS/Gazebo domain" >&2; exit 1; }
export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
run=$(mktemp -d "$results/$scenario-$(date +%Y%m%d-%H%M%S)-XXXXXX")
export YOLO_CONFIG_DIR="$run/ultralytics"
export ROS_LOG_DIR="$run/ros" GAZEBO_LOG_PATH="$run/gazebo" XDG_CONFIG_HOME="$run/gui-config"
mkdir -p "$YOLO_CONFIG_DIR" "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH" "$XDG_CONFIG_HOME"
printf 'source %q\nsource %q\nexport ROS_DOMAIN_ID=%q ROS_LOCALHOST_ONLY=1 GAZEBO_MASTER_URI=%q\nunset ROS_DISCOVERY_SERVER\n' \
    "$SORT_DEMO/environment.sh" "$prefix/local_setup.bash" "$ROS_DOMAIN_ID" "$GAZEBO_MASTER_URI" > "$run/ros_env.sh"
exec > >(trap '' INT; tee "$run/launch.log") 2>&1
echo "Run directory: $run"
python3 "$STATE_DEMO/prepare_world.py" "$scenario" "$run"
env --default-signal=INT ros2 launch my_dwa_controller pedestrian_scene.launch.py mode:="$motion" run:="$run" gui:="$gui" &
pid=$!
cleanup() { kill -INT "$pid" 2>/dev/null || true; wait "$pid" || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM
deadline=$((SECONDS + 150))
until grep -q '"event":"Scenario completed"\|"event": "Scenario completed"' "$run/timeline.jsonl" 2>/dev/null; do
    if ! kill -0 "$pid" 2>/dev/null || ((SECONDS >= deadline)); then
        echo "Scenario failed to complete; see $run/launch.log" >&2
        exit 1
    fi
    sleep .2
done
sleep .6
cleanup
trap - EXIT
for file in sort scan_input vision lidar odom tf_static tf_dynamic ground_truth pedestrians; do
    [[ -s $run/$file.jsonl ]] || { echo "Missing records in $run/$file.jsonl" >&2; exit 1; }
done
echo "Completed $run"
