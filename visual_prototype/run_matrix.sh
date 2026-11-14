#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
root=$PWD
source /opt/ros/humble/setup.bash

if [[ $# -ne 1 || -e $1 ]]; then
    echo "Usage: bash run_matrix.sh NEW_OUTPUT_DIRECTORY" >&2
    exit 2
fi
output=$1
mkdir -p -- "$output"
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export ROS_LOCALHOST_ONLY=1
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
export YOLO_CONFIG_DIR="$root/.cache/ultralytics"
mkdir -p "$YOLO_CONFIG_DIR"
python_exec=${VISUAL_PYTHON:-"$root/.venv/bin/python"}
device=${VISUAL_DEVICE:-cuda:0}
conf=${VISUAL_CONF:-0.25}
imgsz=${VISUAL_IMGSZ:-640}
base_domain=${VISUAL_DOMAIN_BASE:-86}
base_port=${VISUAL_PORT_BASE:-11386}
sim=""

cleanup() {
    if [[ -n $sim ]]; then
        kill -INT "$sim" 2>/dev/null || true
        wait "$sim" || true
        sim=""
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

"$python_exec" - <<'PY' > "$output/environment.txt"
import platform
import cv2
import numpy
import torch
import torchvision
import ultralytics
print(platform.platform())
print(f"Python {platform.python_version()}, NumPy {numpy.__version__}, OpenCV {cv2.__version__}")
print(f"PyTorch {torch.__version__}, torchvision {torchvision.__version__}, "
      f"Ultralytics {ultralytics.__version__}")
print(f"CUDA built={torch.version.cuda}, available={torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU {torch.cuda.get_device_name(0)}")
PY
printf 'device=%s conf=%s imgsz=%s LIBGL_ALWAYS_SOFTWARE=%s OMP_NUM_THREADS=%s\n' \
    "$device" "$conf" "$imgsz" "$LIBGL_ALWAYS_SOFTWARE" "$OMP_NUM_THREADS" >> "$output/environment.txt"

failed=0
index=0
for model in gems subject; do
    export ROS_DOMAIN_ID=$((base_domain + index))
    export GAZEBO_MASTER_URI="http://127.0.0.1:$((base_port + index))"
    echo "Running $model on ROS domain $ROS_DOMAIN_ID and $GAZEBO_MASTER_URI"
    env --default-signal=INT ros2 launch "$root/prototype.launch.py" \
        model:="$model" python:="$python_exec" device:="$device" conf:="$conf" imgsz:="$imgsz" \
        >"$output/$model-launch.log" 2>&1 &
    sim=$!
    if ! python3 -u sweep.py "$model" "$output/$model" | tee "$output/$model-sweep.log"; then
        failed=1
    fi
    cleanup
    index=$((index + 1))
done
exit "$failed"
