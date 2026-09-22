#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$ROOT_DIR/logs"
SERVICE_DIR="$ROOT_DIR/vendor/roboticsservice"
SDK_DIR="$ROOT_DIR/vendor/xrobotoolkit"
GUI_DIR="$SERVICE_DIR/SDKDemo/RobotUnityDemo"
mkdir -p "$LOG_DIR"

if ! ss -ltn | grep -q ':60061'; then
    (
        cd "$SERVICE_DIR"
        export LD_LIBRARY_PATH="$SERVICE_DIR:$SERVICE_DIR/lib:$SERVICE_DIR/SDK/x64:${LD_LIBRARY_PATH:-}"
        export QT_PLUGIN_PATH="$SERVICE_DIR/plugins"
        export QT_QML_PATH="$SERVICE_DIR/qml"
        exec ./RoboticsServiceProcess
    ) >"$LOG_DIR/roboticsservice.log" 2>&1 </dev/null &
    echo $! >"$LOG_DIR/roboticsservice.pid"
    for _ in {1..20}; do
        ss -ltn | grep -q ':60061' && break
        sleep 0.25
    done
fi

GUI_PID_FILE="$LOG_DIR/xrobotoolkit.pid"
GUI_STARTED=false
if ! [[ -f "$GUI_PID_FILE" ]] || ! kill -0 "$(cat "$GUI_PID_FILE")" 2>/dev/null; then
    (
        cd "$GUI_DIR"
        export DISPLAY="${DISPLAY:-:1}"
        export XAUTHORITY="${XAUTHORITY:-/home/dell/.Xauthority}"
        export LD_LIBRARY_PATH="$SERVICE_DIR:$SERVICE_DIR/lib:$SERVICE_DIR/SDK/x64:${LD_LIBRARY_PATH:-}"
        exec ./RobotLinuxDemo.x86_64
    ) >"$LOG_DIR/xrobotoolkit.log" 2>&1 </dev/null &
    echo $! >"$GUI_PID_FILE"
    GUI_PID="$!"
    GUI_STARTED=true
    SERVICE_EVENT_LOG="/home/dell/.local/share/PICOBusinessSuitData/log/$(date +%Y%m%d).txt"
    for _ in {1..60}; do
        if grep -q "server start feedback of pid $GUI_PID" "$SERVICE_EVENT_LOG" 2>/dev/null; then
            break
        fi
        kill -0 "$GUI_PID" 2>/dev/null || break
        sleep 0.25
    done
    if ! kill -0 "$GUI_PID" 2>/dev/null; then
        echo "XRoboToolkit GUI failed to start; see $LOG_DIR/xrobotoolkit.log" >&2
        exit 1
    fi
fi

if ! ss -ltn | grep -q ':60061'; then
    echo "XRoboToolkit RoboticsService failed to listen on port 60061." >&2
    exit 1
fi

PUBLISHER_PID_FILE="$LOG_DIR/vr_publisher.pid"
if [[ -f "$PUBLISHER_PID_FILE" ]] && kill -0 "$(cat "$PUBLISHER_PID_FILE")" 2>/dev/null; then
    if [[ "$GUI_STARTED" == false ]]; then
        echo "XRoboToolkit GUI, XR service, and VR publisher are already running."
        exit 0
    fi
    # XR Service supports one feedback consumer. The GUI initialized after the
    # old publisher, so restart the publisher last to give teleoperation the stream.
    OLD_PUBLISHER_PID="$(cat "$PUBLISHER_PID_FILE")"
    kill -INT "$OLD_PUBLISHER_PID" 2>/dev/null || true
    for _ in {1..20}; do
        kill -0 "$OLD_PUBLISHER_PID" 2>/dev/null || break
        sleep 0.2
    done
    if kill -0 "$OLD_PUBLISHER_PID" 2>/dev/null; then
        kill -TERM "$OLD_PUBLISHER_PID" 2>/dev/null || true
        sleep 0.5
    fi
fi

nohup bash -lc "
    source /home/dell/anaconda3/etc/profile.d/conda.sh
    conda activate pika
    export PYTHONPATH='$SDK_DIR'
    export LD_LIBRARY_PATH='$SDK_DIR':\${LD_LIBRARY_PATH:-}
    cd '$ROOT_DIR'
    exec python3 vr_data_publisher.py --bind tcp://127.0.0.1:5557
" >"$LOG_DIR/vr_publisher.log" 2>&1 </dev/null &
echo $! >"$PUBLISHER_PID_FILE"
echo "XRoboToolkit GUI, XR service, and VR publisher are running (publisher PID $!)."
