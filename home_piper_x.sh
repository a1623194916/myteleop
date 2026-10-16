#!/usr/bin/env bash
# Move the Piper-X arms back to the saved home pose.
# Usage: bash home_piper_x.sh [show|go] [--mode single|dual] [...]
# See the full option list with: bash home_piper_x.sh --help
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$project_dir/XRoboToolkit-Teleop-Sample-Python/.venv/bin/python" \
  "$project_dir/XRoboToolkit-Teleop-Sample-Python/scripts/hardware/piper_x_home_tool.py" "$@"
