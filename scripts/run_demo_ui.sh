#!/usr/bin/env bash
set -eo pipefail
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

usage() {
    cat <<'HELP'
Usage: scripts/run_demo_ui.sh [M1|same-direction|PERSON|S02]
M1 is the default. Press Enter when ready to start. Ctrl+C closes the entire demo.
Source the ROS workspace overlay or set MYDWA_SETUP to its setup.bash.
Dynamic demos use MYDWA_VISUAL_ASSETS, SORT_DEMO and STATE_DEMO.
Override ROS_DOMAIN_ID or GAZEBO_MASTER_URI to use different demo ports.
HELP
}

case "${1:-M1}" in
    -h|--help) usage; exit 0 ;;
    M1|same-direction|PERSON|S02) scene=${1:-M1} ;;
    *) usage >&2; exit 2 ;;
esac
(($# <= 1)) || { usage >&2; exit 2; }

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
setup=${MYDWA_SETUP:-"$root/../../install/setup.bash"}
if [[ -n ${MYDWA_SETUP:-} ]] || ! ros2 pkg prefix my_dwa_controller >/dev/null 2>&1; then
    [[ -r $setup ]] || { echo 'Build and source the MyDWA workspace first.' >&2; exit 1; }
    source "$setup"
fi

if [[ $scene != S02 ]]; then
    experiments=${MYDWA_EXPERIMENTS:-"$HOME/ros2_experiments"}
    export SORT_DEMO=${SORT_DEMO:-"$experiments/sort_ros2_prototype_20261112"}
    export STATE_DEMO=${STATE_DEMO:-"$experiments/state_estimation_compare_20261112"}
    export MYDWA_VISUAL_ASSETS=${MYDWA_VISUAL_ASSETS:-"$experiments/final_mydwa/local_assets/visual"}
    if [[ ! -r $SORT_DEMO/environment.sh || ! -r $STATE_DEMO/vision_state.py ||
          ! -x $MYDWA_VISUAL_ASSETS/.venv/bin/python ]]; then
        echo 'Set SORT_DEMO, STATE_DEMO and MYDWA_VISUAL_ASSETS to the prepared runtime directories.' >&2
        exit 1
    fi
    source "$SORT_DEMO/environment.sh"
    if [[ -r $setup ]]; then source "$setup"; fi
fi

share=$(ros2 pkg prefix --share my_dwa_controller)
[[ -r $share/launch/demo_ui.launch.py ]] || {
    echo 'Rebuild the my_dwa_controller package to install the demo launch.' >&2
    exit 1
}

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-137}
export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://127.0.0.1:11937}
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export LP_NUM_THREADS=${LP_NUM_THREADS:-4} OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
export OPENBLAS_NUM_THREADS=${MYDWA_OPENBLAS_THREADS:-1}

# During setup Bash owns the terminal signals. The supervisor owns them after exec.
exec python3 "$root/scripts/demo_ui_supervisor.py" "$share/launch/demo_ui.launch.py" "$scene"
