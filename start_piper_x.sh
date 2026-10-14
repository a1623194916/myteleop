#!/usr/bin/env bash
# Launch independent camera and robot processes so encoding cannot stall CAN IK.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
video_pid=""
record_enabled=true
video_enabled=false
robot_args=()
teleop_mode=dual
while (( $# )); do
  case "$1" in
    --mode)
      if (( $# < 2 )); then echo '--mode needs single or dual' >&2; exit 2; fi
      teleop_mode="$2"; shift ;;
    --mode=*) teleop_mode="${1#--mode=}" ;;
    --single) teleop_mode=single ;;
    --dual) teleop_mode=dual ;;
    --help|-h)
      echo 'Usage: bash start_piper_x.sh --mode single|dual [--controller-side left|right] [robot options]'
      echo 'Default: dual; left=can0, right=can1; A-key recording; NO headset video.'
      echo 'Use --left-channel/--right-channel to change the mapping, or --channel in single mode.'
      echo '--no-camera disables capture/recording; --video enables optional headset video.'
      echo '--read-only checks feedback without motor enable; --enable-hardware enables real teleoperation.'
      exit 0 ;;
    --record) record_enabled=true ;;
    --no-record) record_enabled=false ;;
    --video) video_enabled=true ;;
    --no-video) video_enabled=false ;;
    --no-camera) record_enabled=false; video_enabled=false ;;
    *) robot_args+=("$1") ;;
  esac
  shift
done
if [[ "$teleop_mode" != single && "$teleop_mode" != dual ]]; then
  echo '--mode must be single or dual' >&2
  exit 2
fi
robot_args+=(--mode "$teleop_mode")
cleanup() {
  if [[ -n "$video_pid" ]]; then
    kill "$video_pid" 2>/dev/null || true
    wait "$video_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
if [[ "$record_enabled" == true || "$video_enabled" == true ]]; then
  camera_args=()
  ready_port=8765
  if [[ "$record_enabled" == true ]]; then
    camera_args+=(--jpeg-port 8765)
    if [[ "$teleop_mode" == dual ]]; then
      record_file=piper_dual_local_305.yaml
    else
      record_file=piper_local_305.yaml
    fi
    robot_args+=(--record-config "$project_dir/piper_teleop_recorder/configs/$record_file")
  else
    ready_port=13579
  fi
  if [[ "$video_enabled" == false ]]; then
    camera_args+=(--no-video)
  fi
  if (exec 3<>/dev/tcp/127.0.0.1/"$ready_port") 2>/dev/null; then
    echo "Camera port $ready_port is already in use. Stop the existing service first." >&2
    exit 1
  fi
  /usr/bin/python3 -u -m pico_video.server "${camera_args[@]}" &
  video_pid=$!
  ready=false
  for ((attempt=0; attempt<250; attempt++)); do
    if ! kill -0 "$video_pid" 2>/dev/null; then
      wait "$video_pid"
      exit 1
    fi
    if (exec 3<>/dev/tcp/127.0.0.1/"$ready_port") 2>/dev/null; then
      ready=true
      break
    fi
    sleep 0.1
  done
  if [[ "$ready" != true ]]; then
    echo 'Video service did not become ready; robot will not start.' >&2
    exit 1
  fi
fi
cd "$project_dir/XRoboToolkit-Teleop-Sample-Python"
export PYTHONPATH=".:${PYTHONPATH:-}"
.venv/bin/python scripts/hardware/teleop_piper_x_hardware.py "${robot_args[@]}"
