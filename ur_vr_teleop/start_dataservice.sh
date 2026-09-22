#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$ROOT_DIR/logs"
PID_FILE="$LOG_DIR/dataservice.pid"
mkdir -p "$LOG_DIR"

if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "dataCapture service is already running (PID $(cat "$PID_FILE"))."
    exit 0
fi

nohup bash -lc '
    source /home/dell/anaconda3/etc/profile.d/conda.sh
    conda activate pika
    source /opt/ros/humble/setup.bash
    source /home/dell/pika_ros/install/setup.bash
    export ROS_LOCALHOST_ONLY=1
    exec ros2 run data_tools data_tools_dataCapture --ros-args \
        -p useService:=true \
        -p hz:=-1 \
        -p timeout:=300 \
        --params-file /home/dell/pika_ros/install/data_tools/share/data_tools/config/ur_teleop_data_params.yaml
' >"$LOG_DIR/dataservice.log" 2>&1 </dev/null &
echo $! >"$PID_FILE"
echo "dataCapture service started (PID $!). Log: $LOG_DIR/dataservice.log"
