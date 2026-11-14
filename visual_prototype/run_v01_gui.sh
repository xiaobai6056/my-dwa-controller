#!/usr/bin/env bash
set -eo pipefail

usage() {
    cat <<'EOF'
Usage: ./run_v01_gui.sh {A|B|C} [--config FILE] [--no-viewers] [--headless]

  A  Stationary Waffle, pedestrian crosses at 2 m
  B  Waffle drives past a stationary pedestrian
  C  Waffle advances while the pedestrian crosses ahead

  --config FILE   Load editable motion presets (default: v01_modes.json)
  --no-viewers    Keep Gazebo GUI and skip the two rqt windows
  --headless      Skip all windows, still run and record the scenario
  -h, --help      Show this help

In GUI mode, arrange the windows and press Enter to start motion.
The GUI remains open after motion ends. Ctrl+C closes this run.
EOF
}

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi
mode=${1:-}
if [[ $mode != A && $mode != B && $mode != C ]]; then
    usage >&2
    exit 2
fi
shift
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
config="$root/v01_modes.json"
gui=true
viewers=true
while (($#)); do
    case $1 in
        --config)
            if [[ $# -lt 2 || ! -r $2 ]]; then
                echo '--config needs a readable JSON file' >&2
                exit 2
            fi
            config=$(realpath -- "$2")
            shift
            ;;
        --no-viewers) viewers=false ;;
        --headless) gui=false; viewers=false ;;
        *) usage >&2; exit 2 ;;
    esac
    shift
done

assets=$(realpath -m -- "${MYDWA_VISUAL_ASSETS:-$root}")
export MYDWA_VISUAL_ASSETS="$assets"
if [[ ${ROS_DISTRO:-} != humble ]]; then
    echo 'Source ROS 2 Humble before running this script.' >&2
    exit 1
fi
if [[ ! -r $assets/.cache/gems.world || ! -r $assets/.cache/waffle.urdf ||
      ! -r $assets/weights/yolov8n.pt || ! -x $assets/.venv/bin/python ]]; then
    echo 'Prepare the assets and environment described in requirements.txt.' >&2
    exit 1
fi
cd -- "$assets"

# Reserve a domain and its Gazebo port for the lifetime of this launcher.
# Check sockets as well as the lock so existing local ROS participants are skipped.
mkdir -p .cache/v01-locks .cache/ultralytics runs
for domain in {92..100}; do
    exec 9>".cache/v01-locks/$domain.lock"
    if ! flock -n 9; then
        continue
    fi
    base=$((7400 + 250 * domain))
    port=$((11400 + domain))
    if ss -H -lun | awk -v base="$base" '
        { n=split($4, fields, ":"); p=fields[n]+0; if (base<=p && p<base+250) found=1 }
        END { exit !found }'; then
        continue
    fi
    if [[ -n $(ss -H -ltn "sport = :$port") ]]; then
        continue
    fi
    export ROS_DOMAIN_ID=$domain
    export GAZEBO_MASTER_URI="http://127.0.0.1:$port"
    reserved=true
    break
done
if [[ ${reserved:-false} != true ]]; then
    echo 'No free V01 domain and Gazebo port in the configured range (92–100).' >&2
    exit 1
fi

export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
export YOLO_CONFIG_DIR="$assets/.cache/ultralytics"
run=$(mktemp -d "$assets/runs/v01-$mode-$(date +%Y%m%d-%H%M%S)-XXXXXX")
export ROS_LOG_DIR="$run/ros"
export GAZEBO_LOG_PATH="$run/gazebo"
export XDG_CONFIG_HOME="$run/gui-config"
mkdir -p "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH" "$XDG_CONFIG_HOME"
exec > >(trap '' INT; tee "$run/launch.log") 2>&1
cat > "$run/ros_env.sh" <<EOF
# Source your ROS 2 environment before loading these run settings.
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI
EOF

echo "Run directory: $run"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
echo "Topics: /clock /cmd_vel /visual_prototype/camera/image_raw /visual_prototype/camera/camera_info"
echo "        /visual_prototype/annotated /visual_prototype/detections /visual_prototype/metrics"
echo "GUI=$gui image_viewers=$viewers. Ctrl+C closes all processes in this launch."
echo "For another terminal: source '$run/ros_env.sh'"
python3 -u "$root/v01_config.py" "$mode" "$config" "$run"
(
    flock 8
    build=.cache/v01-gui-build
    plugin=$build/libv01_pose.so
    if [[ ! -f $plugin || $root/v01_gazebo/pose_plugin.cpp -nt $plugin || $root/v01_gazebo/CMakeLists.txt -nt $plugin ]]; then
        cmake -S "$root/v01_gazebo" -B "$build" -DCMAKE_BUILD_TYPE=Release
        cmake --build "$build" --parallel 2
    fi
) 8>.cache/v01-locks/build.lock
launch_args=("$root/prototype.launch.py" gui:="$gui" viewers:="$viewers" v01_run:="$run")
if [[ $gui == true ]]; then
    exec python3 "$root/../scripts/demo_ui_supervisor.py" --recording "$run" "${launch_args[@]}"
fi
exec ros2 launch "${launch_args[@]}"
