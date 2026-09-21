"""Replay recorded FR3C ServoJ targets without XR/IK.

This isolates the Fairino SDK/controller path using a trajectory captured by
teleoperation. The first recorded target is checked against the live pose to
avoid an accidental jump.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import tyro


def load_targets(path: str, start_s: float = 0.0, duration_s: float | None = None):
    raw = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        if row.get("event") != "servoj":
            continue
        raw.append((float(row["t_monotonic"]), np.asarray(row["command_q_rad"], dtype=float)))
    if not raw:
        raise ValueError("no servoj rows found in the selected log interval")
    log_t0 = raw[0][0]
    rows = [(t, q) for t, q in raw if t - log_t0 >= start_s]
    if duration_s is not None:
        selected_t0 = rows[0][0]
        rows = [(t, q) for t, q in rows if t - selected_t0 <= duration_s]
    t0 = rows[0][0]
    return [(t - t0, q) for t, q in rows]


def main(
    robot_ip: str,
    jsonl_path: str,
    start_s: float = 0.0,
    duration_s: float = 20.0,
    max_start_delta_deg: float = 5.0,
    max_replay_gap_s: float = 0.05,
    reverse: bool = False,
):
    """Replay a captured command stream around the current robot pose."""
    from xrobotoolkit_teleop.hardware.interface.fr3c import Fr3cController

    targets = load_targets(
        jsonl_path,
        start_s=start_s,
        duration_s=None if reverse else duration_s,
    )
    robot = Fr3cController(robot_ip=robot_ip, cmd_t=0.01)
    current = robot.get_current_joint_positions()
    if reverse:
        original = targets
        targets = []
        elapsed = 0.0
        for index in range(len(original) - 1, -1, -1):
            if targets:
                elapsed += max(0.0, original[index + 1][0] - original[index][0])
            targets.append((elapsed, original[index][1]))
        if duration_s is not None:
            targets = [item for item in targets if item[0] <= duration_s]
        print("Replaying the recorded trajectory in reverse.")
    nearest_index = min(
        range(len(targets)),
        key=lambda i: float(np.max(np.abs(targets[i][1] - current))),
    )
    if nearest_index:
        targets = [(t - targets[nearest_index][0], q) for t, q in targets[nearest_index:]]
        print(f"Selected nearest recorded target at sample {nearest_index}.")
    first = targets[0][1]
    delta_deg = np.rad2deg(first - current)
    print("Current joints (deg):", np.round(np.rad2deg(current), 3).tolist())
    print("Replay first (deg):", np.round(np.rad2deg(first), 3).tolist())
    print("Start delta (deg):", np.round(delta_deg, 3).tolist())
    if float(np.max(np.abs(delta_deg))) > max_start_delta_deg:
        robot.close()
        raise SystemExit(
            f"Refusing replay: first target is {np.max(np.abs(delta_deg)):.2f} deg "
            f"from the live pose (limit {max_start_delta_deg:.2f} deg)."
        )

    print(f"Replaying {len(targets)} ServoJ targets for {targets[-1][0]:.2f}s ...")
    errors = 0
    rpc_ms = []
    robot.start_servo()
    t0 = time.monotonic()
    try:
        for relative_t, q in targets:
            deadline = t0 + relative_t
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            started = time.monotonic()
            robot.servo_joints(q)
            elapsed_ms = (time.monotonic() - started) * 1000.0
            rpc_ms.append(elapsed_ms)
            if robot.last_servo_error:
                errors += 1
                print(f"ServoJ error {robot.last_servo_error}, rpc={elapsed_ms:.1f} ms")
            if robot.servo_stream_dead:
                print("Servo stream dead; stopping replay.")
                break
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        robot.stop_servo()
        robot.close()
    if rpc_ms:
        print(
            f"Replay done: calls={len(rpc_ms)}, errors={errors}, "
            f"rpc p50={np.percentile(rpc_ms, 50):.2f} ms, "
            f"p95={np.percentile(rpc_ms, 95):.2f} ms, max={max(rpc_ms):.2f} ms"
        )


if __name__ == "__main__":
    tyro.cli(main)
