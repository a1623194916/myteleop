#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATASET_DIR="${1:-/data2/kyz/kyzdata/raw/ur_vr}"
INSTRUCTION="${2:-UR5 VR teleoperation}"

source /home/dell/anaconda3/etc/profile.d/conda.sh
conda activate pika
source /opt/ros/humble/setup.bash
source /home/dell/pika_ros/install/setup.bash
export ROS_LOCALHOST_ONLY=1

cd "$ROOT_DIR"
if ! python3 wait_for_vr.py \
    --endpoint tcp://127.0.0.1:5557 \
    --controller-side right \
    --timeout 0; then
    exit 1
fi

CAPTURE_ARGS=()
if ros2 service list --no-daemon --spin-time 1 | grep -qx '/data_tools_dataCapture/capture_service'; then
    CAPTURE_ARGS=(--publish-ros --capture-on-a)
    echo "DataService detected: A button capture is enabled."
else
    echo "DataService is not running: starting pure teleoperation without camera/data capture."
fi

exec python3 teleop_ur_vr.py \
    --robot-ip 192.168.5.80 \
    --vr-endpoint tcp://127.0.0.1:5557 \
    --controller-side right \
    "${CAPTURE_ARGS[@]}" \
    --dataset-dir "$DATASET_DIR" \
    --instruction "$INSTRUCTION" \
    --max-linear-speed 0.10 \
    --max-angular-speed 0.40
