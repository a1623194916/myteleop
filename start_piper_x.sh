#!/usr/bin/env bash
# Launch independent camera and robot processes so encoding cannot stall CAN IK.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
video_pid=""
record_enabled=true
video_enabled=false
robot_args=()
for argument in "$@"; do
  case "$argument" in
    --record) record_enabled=true ;;
    --no-record) record_enabled=false ;;
    --video) video_enabled=true ;;
    --no-video) video_enabled=false ;;
    --no-camera) record_enabled=false; video_enabled=false ;;
    *) robot_args+=("$argument") ;;
  esac
done
cleanup() {
  if [[ -n "$video_pid" ]]; then
    kill "$video_pid" 2>/dev/null || true
    wait "$video_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
if [[ "${1:-}" == "--help" ]]; then
  echo 'Usage: bash start_piper_x.sh [--no-camera|--no-record] [--video] [Piper-X script arguments]'
  echo 'Default: local Gemini 305 JPEG capture + A-key recording, NO headset video.'
  echo 'Use --read-only to check CAN feedback; --enable-hardware for real teleoperation.'
  exit 0
fi
if [[ "$record_enabled" == true || "$video_enabled" == true ]]; then
  camera_args=()
  ready_port=8765
  if [[ "$record_enabled" == true ]]; then
    camera_args+=(--jpeg-port 8765)
    robot_args+=(--record-config "$project_dir/piper_teleop_recorder/configs/piper_local_305.yaml")
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
