"""Measure FR3C ServoJ stream timing quality (send-period jitter + RPC latency).

The teleop stack only controls the SHAPE of the commanded trajectory; the robot
controller also depends on WHEN each ServoJ point arrives. This tool isolates
timing from teleop logic: it streams a slow, tiny sine (or --no-move read
probes) with an absolute-deadline metronome and reports the send-period
distribution the controller actually sees.

Usage (from XRoboToolkit-Teleop-Sample-Python):
    # offline sanity check of the tool itself (no robot):
    PYTHONPATH=. .venv/bin/python scripts/hardware/diag_fr3c_servo_timing.py --mock
    # read-only probes, arm does NOT move, measures RPC + metronome jitter:
    PYTHONPATH=. .venv/bin/python scripts/hardware/diag_fr3c_servo_timing.py --robot-ip 192.168.5.23 --no-move
    # same, with the real-teleop MuJoCo mirror running in-process to reproduce
    # the GIL contention of teleop_fr3c_hardware.py:
    PYTHONPATH=. .venv/bin/python scripts/hardware/diag_fr3c_servo_timing.py --robot-ip 192.168.5.23 --no-move --load mirror
    # ServoJ plus explicit MoveGripper load (gripper must already be active):
    PYTHONPATH=. .venv/bin/python scripts/hardware/diag_fr3c_servo_timing.py \
        --robot-ip 192.168.5.23 --gripper-load --duration-s 30
    # tiny motion test (2 deg sine at 0.2 Hz around the current pose):
    PYTHONPATH=. .venv/bin/python scripts/hardware/diag_fr3c_servo_timing.py --robot-ip 192.168.5.23

Compare the period p95/p99 of --no-move with and without --load mirror: if the
mirror blows up the tail, the low-frequency jitter you see in teleop is
client-side scheduling, not the robot.
"""
from __future__ import annotations

import csv
import random
import statistics
import threading
import time

import numpy as np
import tyro


def pct(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))]


class MockRpc:
    """Stands in for the Fairino XML-RPC object when no robot is reachable."""

    def __init__(self, base_rtt_s: float = 0.0012, spike_prob: float = 0.02, spike_s: float = 0.008):
        self._rng = random.Random(7)
        self._base = base_rtt_s
        self._spike_prob = spike_prob
        self._spike = spike_s

    def ServoJ(self, *_args, **_kwargs) -> int:
        time.sleep(self._base + (self._spike if self._rng.random() < self._spike_prob else 0.0))
        return 0

    def GetActualJointPosRadian(self, _flag: int = 1):
        time.sleep(self._base + (self._spike if self._rng.random() < self._spike_prob else 0.0))
        return 0, [0.0] * 6


class TimingLogger:
    def __init__(self, csv_path: str | None):
        self._csv_path = csv_path
        self._file = None
        self._writer = None

    def __enter__(self):
        if self._csv_path:
            self._file = open(self._csv_path, "w", newline="")
            self._writer = csv.writer(self._file)
            self._writer.writerow(
                ["tick", "schedule_s", "wake_late_ms", "send_s", "period_ms", "rpc_ms", "measured_deg0"]
            )
        return self

    def __exit__(self, *exc):
        if self._file:
            self._file.close()

    def row(self, **cols):
        if self._writer:
            self._writer.writerow([cols.get(k, "") for k in
                                   ["tick", "schedule_s", "wake_late_ms", "send_s",
                                    "period_ms", "rpc_ms", "measured_deg0"]])


def summarize(periods_ms: list[float], rpc_ms: list[float], wake_late_ms: list[float], label: str):
    def stats(v: list[float]) -> str:
        if not v:
            return "n/a"
        return (f"mean {statistics.mean(v):6.2f}  std {statistics.pstdev(v):5.2f}  "
                f"p50 {pct(v, 50):6.2f}  p95 {pct(v, 95):6.2f}  p99 {pct(v, 99):6.2f}  max {max(v):7.2f}")

    print(f"\n[{label}]")
    print(f"  period (ms)  : {stats(periods_ms)}")
    print(f"  rpc rtt (ms) : {stats(rpc_ms)}")
    print(f"  wake late (ms): {stats(wake_late_ms)}")


def main(
    robot_ip: str = "192.168.58.2",
    cmd_t: float = 0.010,
    servo_transport: str = "udp",
    duration_s: float = 20.0,
    amplitude_deg: float = 1.0,
    freq_hz: float = 0.2,
    no_move: bool = False,
    mock: bool = False,
    load: str = "none",
    gripper_load: bool = False,
    gripper_period_s: float = 0.25,
    gripper_position_percent: int = 50,
    csv_path: str | None = None,
):
    """Run the ServoJ timing diagnostic.

    Args:
        robot_ip: FR3C controller IP.
        cmd_t: ServoJ command period in seconds (matches teleop default 0.01).
        servo_transport: ``udp`` uses the SDK's UDPServoJ path; ``xmlrpc``
            retains the legacy round-trip path for comparison.
        duration_s: Streaming duration.
        amplitude_deg: Sine amplitude per joint when streaming (small by design).
        freq_hz: Sine frequency in Hz (0.2 Hz -> peak joint speed ~2.5 deg/s).
        no_move: Do not start a servo session; probe with a read-only RPC instead.
        mock: No robot; synthesize RPC latency to validate the tool offline.
        load: "none" or "mirror" (run the real-teleop MuJoCo mirror in-process
            to reproduce the GIL contention of teleop_fr3c_hardware.py).
        gripper_load: While ServoJ is running, send a MoveGripper command from
            a second thread. The gripper must already be activated. This is an
            intentionally stressful diagnostic, not a teleop mode.
        gripper_period_s: Interval between diagnostic MoveGripper calls.
        gripper_position_percent: Position sent by the diagnostic gripper load.
        csv_path: Optional CSV log of every tick.
    """
    if load not in {"none", "mirror"}:
        raise SystemExit("load must be 'none' or 'mirror'")
    transport = str(servo_transport).strip().lower()
    if transport not in {"udp", "xmlrpc"}:
        raise SystemExit("servo_transport must be 'udp' or 'xmlrpc'")
    cmd_type = 1 if transport == "udp" else 0
    if gripper_period_s <= 0.0:
        raise SystemExit("gripper_period_s must be positive")
    if not 0 <= gripper_position_percent <= 100:
        raise SystemExit("gripper_position_percent must be in [0, 100]")
    if gripper_load and (mock or no_move):
        raise SystemExit("--gripper-load requires a real ServoJ motion test")
    if not no_move and not mock:
        print(f"SAFETY: arm will track a {amplitude_deg} deg, {freq_hz} Hz sine around the CURRENT pose.")
        print("Keep the e-stop at hand. Ctrl+C ends the session (ServoMoveEnd is sent).")

    if mock:
        rpc = MockRpc()
        state_getter = lambda: [0.0] * 6  # noqa: E731
    else:
        from xrobotoolkit_teleop.hardware.interface.fr3c import _load_robot_module

        module = _load_robot_module()
        robot = module.RPC(robot_ip)
        rpc = robot.robot
        state_getter = lambda: list(robot.robot_state_pkg.jt_cur_pos)  # noqa: E731

    stop = threading.Event()
    gripper_stats = {"calls": 0, "errors": 0, "rpc_ms": []}
    gripper_thread = None

    def run_gripper_load():
        """Deliberately share the SDK connection to expose RPC contention."""
        while not stop.wait(gripper_period_s):
            started = time.monotonic()
            try:
                err = rpc.MoveGripper(
                    1, int(gripper_position_percent), 20, 20, 1000,
                    1, 0, 0.0, 0, 0,
                )
                gripper_stats["calls"] += 1
                gripper_stats["rpc_ms"].append((time.monotonic() - started) * 1000.0)
                if err != 0:
                    gripper_stats["errors"] += 1
                    print(f"Diagnostic MoveGripper error {err}")
            except Exception as exc:  # noqa: BLE001
                gripper_stats["calls"] += 1
                gripper_stats["errors"] += 1
                gripper_stats["rpc_ms"].append((time.monotonic() - started) * 1000.0)
                print(f"Diagnostic MoveGripper exception: {exc}")

    mirror = None
    if load == "mirror" and not mock:
        from xrobotoolkit_teleop.hardware.fr3c_mujoco_mirror import Fr3cMujocoMirror

        from pathlib import Path

        pico_root = Path(__file__).resolve().parents[3]
        mirror = Fr3cMujocoMirror(str(pico_root / "fr3c_assets/scene_fr3c.xml"))
        mirror_thread = threading.Thread(
            target=mirror.run, args=(stop, lambda: np.deg2rad(np.asarray(state_getter()))), daemon=True
        )
        mirror_thread.start()

    q0 = list(state_getter())
    probe_name = "ServoJ" if not no_move else "GetActualJointPosRadian"
    print(
        f"Streaming {probe_name} at cmd_t={cmd_t * 1000:.1f} ms for "
        f"{duration_s:.0f} s (transport={transport}, load={load}) ..."
    )

    periods_ms: list[float] = []
    rpc_ms: list[float] = []
    wake_late_ms: list[float] = []

    try:
        if not no_move and not mock:
            err = robot.ServoMoveStart(cmdType=cmd_type)
            if err != 0:
                raise SystemExit(f"ServoMoveStart failed, error {err}")
        if gripper_load:
            print("WARNING: diagnostic gripper load is active; keep the e-stop ready.")
            gripper_thread = threading.Thread(target=run_gripper_load, name="diag-gripper", daemon=True)
            gripper_thread.start()

        with TimingLogger(csv_path) as log:
            t_start = time.monotonic()
            next_t = t_start + cmd_t
            tick = 0
            prev_send: float | None = None
            while time.monotonic() - t_start < duration_s:
                wake = time.monotonic()
                wake_late_ms.append((wake - next_t) * 1000.0)

                if no_move or mock:
                    if mock:
                        err = rpc.ServoJ() if not no_move else rpc.GetActualJointPosRadian()
                    else:
                        err = robot.GetActualJointPosRadian(1)[0]
                else:
                    phase = 2.0 * np.pi * freq_hz * (wake - t_start)
                    target = [q + amplitude_deg * np.sin(phase) for q in q0]
                    err = robot.ServoJ(
                        target, [0.0, 0.0, 0.0, 0.0],
                        0.0, 0.0, cmd_t, 0.0, 0.0, 0, cmd_type,
                    )
                send = time.monotonic()
                rpc_ms.append((send - wake) * 1000.0)
                if prev_send is not None:
                    periods_ms.append((send - prev_send) * 1000.0)
                prev_send = send
                if log._writer:
                    log.row(tick=tick, schedule_s=f"{next_t:.4f}", wake_late_ms=f"{wake_late_ms[-1]:.3f}",
                            send_s=f"{send:.4f}", period_ms=(f"{periods_ms[-1]:.3f}" if periods_ms else ""),
                            rpc_ms=f"{rpc_ms[-1]:.3f}", measured_deg0=f"{state_getter()[0]:.4f}")
                tick += 1

                # Keep ServoJ call-start timing relative to the wake-up before
                # the RPC. If the RPC itself exceeds cmd_t, the next call
                # naturally starts late; it must never start early.
                next_t = wake + cmd_t
                remain = next_t - time.monotonic()
                if remain > 0:
                    time.sleep(remain)
                if err != 0 and not mock and not no_move:
                    print(f"ServoJ error {err}; aborting stream")
                    break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop.set()
        if not no_move and not mock:
            try:
                robot.ServoMoveEnd(cmdType=cmd_type)
                print("ServoMoveEnd sent.")
            except Exception as e:  # noqa: BLE001
                print(f"ServoMoveEnd failed: {e}")

    summarize(periods_ms, rpc_ms, wake_late_ms, f"{probe_name} @ cmd_t={cmd_t * 1000:.1f} ms, load={load}")
    if gripper_load:
        gripper_rpc = gripper_stats["rpc_ms"]
        print(
            f"  gripper calls: {gripper_stats['calls']}, errors: {gripper_stats['errors']}, "
            f"rpc p95: {pct(gripper_rpc, 95):.2f} ms, max: "
            f"{max(gripper_rpc) if gripper_rpc else float('nan'):.2f} ms"
        )
    over2 = sum(1 for v in periods_ms if v > cmd_t * 1000.0 + 2.0)
    over5 = sum(1 for v in periods_ms if v > cmd_t * 1000.0 + 5.0)
    print(f"  ticks: {len(periods_ms)}, period > cmd_t+2ms: {over2} ({100.0 * over2 / max(1, len(periods_ms)):.1f}%), "
          f"> cmd_t+5ms: {over5} ({100.0 * over5 / max(1, len(periods_ms)):.1f}%)")
    print("Rule of thumb: p95 within ~1-2 ms of cmd_t and few +2 ms outliers -> timing is clean;")
    print("a fat tail (+5..15 ms spikes) means client-side scheduling/network jitter,")
    print("which the controller interpolates into exactly the low-frequency wobble you feel.")
    if csv_path:
        print(f"Per-tick log written to {csv_path}")


if __name__ == "__main__":
    tyro.cli(main)
