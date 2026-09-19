"""Real-gripper follow-through test: drive the trigger depth as a time-varying
curve and confirm the gripper closure follows it continuously.

This is the on-hardware companion to ``test_fr3c_gripper_trigger_mapping.py``.
It connects to the real arm, activates the gripper, then feeds a smooth
``trigger(t)`` profile (open -> deep -> hold -> release) through the same
``Fr3cVrGripperController`` used by teleoperation, printing each step's target
so you can watch the gripper track the trigger depth. With
``recover_on_any_error`` a bottom-out 73/8-1 latch auto-clears instead of
freezing subsequent commands.

Run:
    PYTHONPATH=. .venv/bin/python \\
        scripts/hardware/test_fr3c_gripper_follow.py \\
        --robot-ip 192.168.5.23 --side left --duration 15 --hz 20
"""

import argparse
import sys
import time

sys.path.insert(0, "/home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python")

from xrobotoolkit_teleop.hardware.interface.fr3c import Fr3cController
from xrobotoolkit_teleop.hardware.fr3c_gripper import Fr3cVrGripperController


def trigger_curve(t: float) -> float:
    """Piecewise-continuous trigger profile in [0,1] over ~15 s."""
    if t < 2.0:
        return 0.0
    if t < 5.0:
        return (t - 2.0) / 3.0 * 0.6            # open -> 0.6
    if t < 6.5:
        return 0.6                              # hold shallow
    if t < 9.0:
        return 0.6 + (t - 6.5) / 2.5 * 0.4      # ramp to full close (1.0)
    if t < 10.5:
        return 1.0                              # hold closed
    if t < 13.0:
        return 1.0 - (t - 10.5) / 2.5           # release back toward 0.4
    if t < 14.0:
        return 0.4
    return 0.0


class CurvedClient:
    """Injects a time-varying trigger on the given side."""

    def __init__(self, side):
        self._side = side
        self.t = 0.0

    def get_key_value_by_name(self, name):
        if name == f"{self._side}_trigger":
            return trigger_curve(self.t)
        if name == f"{self._side}_grip":
            return 1.0
        return 0.0

    def get_timestamp_ns(self):
        return time.time_ns()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="192.168.5.23")
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--min-change", type=float, default=3.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--motion-gate", type=int, default=1)
    args = parser.parse_args()

    side = args.side
    robot = Fr3cController(robot_ip=args.robot_ip)
    client = CurvedClient(side)
    controller = Fr3cVrGripperController(
        xr_client=client,
        robot=robot,
        controller_side=side,
        trigger_threshold=0.05,
        open_position_percent=0.0,
        closed_position_percent=97.0,
        min_position_change_percent=args.min_change,
        min_command_interval_s=args.interval,
        recover_on_any_error=True,
    )

    hz = max(1, int(args.hz))
    period = 1.0 / hz
    log_every = 0.5
    deadline = time.monotonic() + args.duration
    dt = 0.0
    try:
        robot.reset_all_errors()
        for attempt in range(3):
            try:
                controller.activate()
                break
            except Exception as e:
                print(f"    activate attempt {attempt+1} failed ({e}); retrying")
                time.sleep(2.0)
                robot.reset_all_errors()
        else:
            raise RuntimeError("gripper activate failed after retries")
        print(f"[follow] {side} gripper active on {args.robot_ip}; "
              f"running {args.duration:.0f}s trigger curve @ {hz}Hz")
        t0 = time.monotonic()
        last_log = 0.0
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            dt = now - t0
            client.t = dt
            control = controller.update()
            if now - last_log >= log_every:
                print(
                    f"    t={dt:5.2f}s trigger={trigger_curve(dt):4.2f} "
                    f"-> target_pos={control.position_percent:6.2f}% "
                    f"width={control.width_mm:5.1f}mm"
                )
                last_log = now
            time.sleep(period)
    finally:
        # park open before disconnecting
        try:
            robot.move_gripper(0.0)
        except Exception:
            pass
        robot.close()
    print("[follow] done. Watch the gripper open/close each curve segment.")


if __name__ == "__main__":
    main()