#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$project_dir/XRoboToolkit-Teleop-Sample-Python/.venv/bin/python" \
  "$project_dir/disable_piper_x.py" "$@"
