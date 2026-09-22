#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$ROOT_DIR/logs"

descendants_of() {
    local parent="$1"
    local child
    while read -r child; do
        [[ -z "$child" ]] && continue
        descendants_of "$child"
        echo "$child"
    done < <(pgrep -P "$parent" || true)
}

stop_tree() {
    local name="$1"
    local pid="$2"
    if ! kill -0 "$pid" 2>/dev/null; then
        return
    fi
    local -a descendants=()
    local -a processes=("$pid")
    mapfile -t descendants < <(descendants_of "$pid")
    processes+=("${descendants[@]}")

    kill -INT "${processes[@]}" 2>/dev/null || true
    for _ in {1..10}; do
        local alive=false
        for process in "${processes[@]}"; do
            kill -0 "$process" 2>/dev/null && alive=true
        done
        [[ "$alive" == false ]] && break
        sleep 0.2
    done
    kill -TERM "${processes[@]}" 2>/dev/null || true
    sleep 0.5
    kill -KILL "${processes[@]}" 2>/dev/null || true
    echo "Stopped $name process tree (root PID $pid)."
}

# Stop teleoperation first so its cleanup can request dataCapture STOP.
while read -r pid; do
    [[ -n "$pid" ]] && stop_tree teleop "$pid"
done < <(pgrep -f '[p]ython3 .*teleop_ur_vr.py' || true)

for name in vr_publisher xrobotoolkit roboticsservice dataservice cameras; do
    pid_file="$LOG_DIR/$name.pid"
    if [[ -f "$pid_file" ]]; then
        pid="$(cat "$pid_file")"
        [[ "$pid" =~ ^[0-9]+$ ]] && stop_tree "$name" "$pid"
        rm -f "$pid_file"
    fi
done

# Also remove children orphaned by an older launcher or stale PID file. Match
# executable paths and individual argv entries, never the containing shell text.
for proc_dir in /proc/[0-9]*; do
    pid="${proc_dir##*/}"
    kill -0 "$pid" 2>/dev/null || continue
    executable="$(readlink "$proc_dir/exe" 2>/dev/null || true)"
    managed=false
    case "$executable" in
        */data_tools_dataCapture|*/realsense2_camera_node|*/RoboticsServiceProcess|*/RobotLinuxDemo.x86_64)
            managed=true
            ;;
    esac
    if [[ "$managed" == false ]] && [[ -r "$proc_dir/cmdline" ]]; then
        while IFS= read -r -d '' argument; do
            case "$argument" in
                "$ROOT_DIR/vr_data_publisher.py"|*/orbbec_camera.py|open_ur_teleop_camera.launch.py)
                    managed=true
                    break
                    ;;
            esac
        done <"$proc_dir/cmdline"
    fi
    [[ "$managed" == true ]] && stop_tree orphan "$pid"
done

echo "All UR VR teleoperation processes have been stopped."
