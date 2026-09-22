#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$ROOT_DIR/logs"
PID_FILE="$LOG_DIR/cameras.pid"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source /home/dell/pika_ros/install/setup.bash
export ROS_LOCALHOST_ONLY=1
set -u

if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    CAMERA_PID="$(cat "$PID_FILE")"
    CAMERA_HZ="$(timeout 4 ros2 topic hz --window 5 /gripper/camera/color/image_raw 2>&1 || true)"
    if grep -q "average rate" <<<"$CAMERA_HZ"; then
        echo "Camera stack is healthy and already running (PID $CAMERA_PID)."
        exit 0
    fi

    echo "Camera process $CAMERA_PID is alive but RealSense has no frames; restarting it."
    kill "$CAMERA_PID"
    for _ in {1..10}; do
        kill -0 "$CAMERA_PID" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 "$CAMERA_PID" 2>/dev/null; then
        kill -KILL "$CAMERA_PID"
    fi
fi

nohup bash -lc '
    source /home/dell/anaconda3/etc/profile.d/conda.sh
    conda activate pika
    source /opt/ros/humble/setup.bash
    source /home/dell/pika_ros/install/setup.bash
    export ROS_LOCALHOST_ONLY=1
    exec ros2 launch sensor_tools open_ur_teleop_camera.launch.py \
        gripper_depth_camera_no:=_412622271789
' >"$LOG_DIR/cameras.log" 2>&1 </dev/null &
echo $! >"$PID_FILE"
echo "Camera stack started (PID $!). Log: $LOG_DIR/cameras.log"
