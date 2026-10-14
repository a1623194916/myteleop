#!/usr/bin/env bash
# Start PC Service and optional Unity before the teleop SDK takes the feedback stream.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
service_dir=/opt/apps/roboticsservice
log_dir="$project_dir/logs/pico_vr"
unity=false
case "${1:-}" in
  --unity) unity=true ;;
  --help|-h) echo 'Usage: bash start_pico_vr.sh [--unity]'; exit 0 ;;
  '') ;;
  *) echo 'Usage: bash start_pico_vr.sh [--unity]' >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo 'Too many arguments' >&2; exit 2; fi
# Starting a new GUI steals the existing SDK consumer. Never do this mid-teleop.
if [[ "$unity" == true ]] && pgrep -f '^([^ ]*/)?python([0-9.]+)? .*scripts/hardware/teleop_piper_x_hardware.py' >/dev/null; then
  echo 'Stop Piper teleop before starting Unity; then launch teleop last.' >&2
  exit 1
fi
mkdir -p "$log_dir"
export DISPLAY="${DISPLAY:-:1}"
export LD_LIBRARY_PATH="$service_dir:$service_dir/lib:$service_dir/SDK/x64:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="$service_dir/plugins:${QT_PLUGIN_PATH:-}"
export QT_QML_PATH="$service_dir/qml:${QT_QML_PATH:-}"
service_ready() { ss -H -ltn 'sport = :60061' | rg -q LISTEN; }
if service_ready; then
  echo 'PC Service already listens on 60061; reusing it (including services owned by another user).'
else
  (cd "$service_dir"; exec ./RoboticsServiceProcess) >"$log_dir/service.log" 2>&1 </dev/null &
  service_pid=$!
  for ((attempt=0; attempt<40; attempt++)); do
    service_ready && break
    if ! kill -0 "$service_pid" 2>/dev/null; then
      echo "PC Service exited; see $log_dir/service.log" >&2; exit 1
    fi
    sleep 0.25
  done
  if ! service_ready; then echo "PC Service not ready; see $log_dir/service.log" >&2; exit 1; fi
fi
if [[ "$unity" == true ]]; then
  if pgrep -f '(^|/| )RobotLinuxDemo\.x86_64( |$)' >/dev/null; then
    echo 'Unity already running; reusing it. Do not restart it during teleop.'
  else
    (cd "$service_dir/SDKDemo/RobotUnityDemo"; exec ./RobotLinuxDemo.x86_64) >"$log_dir/unity.log" 2>&1 </dev/null &
    unity_pid=$!
    ready=false
    # Service log filenames follow the service start date, sometimes weeks ago.
    # Match the exact GUI PID in either local user's service logs.
    for ((attempt=0; attempt<80; attempt++)); do
      if ! kill -0 "$unity_pid" 2>/dev/null; then
        echo "Unity exited; see $log_dir/unity.log" >&2; exit 1
      fi
      if rg -q "server start feedback of pid $unity_pid " \
        "$HOME/.local/share/PICOBusinessSuitData/log" \
        /home/u22/.local/share/PICOBusinessSuitData/log 2>/dev/null; then
        ready=true; break
      fi
      sleep 0.25
    done
    if [[ "$ready" != true ]]; then
      echo "Cannot confirm Unity SDK initialization; check $log_dir/unity.log before starting teleop." >&2
      exit 1
    fi
    echo "Unity initialized (PID $unity_pid)."
  fi
fi
echo 'PC Service ready. Connect PICO to 192.168.5.41; launch teleop last to own the SDK stream.'
