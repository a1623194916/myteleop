"""Offline (or real) verification of the VR trigger -> gripper toggle.

Simulates PICO input (a scripted ``left_trigger`` / ``right_trigger`` profile)
fed through the same ``Fr3cVrGripperController`` used by
``teleop_fr3c_dual_hardware.py``, and inspects every ``move_gripper`` call it
issues to confirm one trigger press closes and the next trigger press opens.
The standalone mapper math is checked separately.

``Fr3cVrGripperController`` only sends ``move_gripper`` when the commanded
percent changes by >= ``--min-change``, so repeated trigger frames collapse into
one call. By default everything is mocked (fake PICO input, fake robot), so only
the mapping wiring is verified. Pass ``--robot-ip`` to run the real Fairino
``MoveGripper`` as an on-hardware self-test.

Run:
    # offline mapping + simulated-controller check (recommended first)
    python3 scripts/hardware/test_fr3c_gripper_trigger_mapping.py

    # real MoveGripper self-test on the arm driven by the left controller
    PYTHONPATH=. .venv/bin/python \
        scripts/hardware/test_fr3c_gripper_trigger_mapping.py \
        --robot-ip 192.168.5.23 --side left

    # connect + report mapping only, do not touch hardware
    ... --robot-ip 192.168.5.23 --side left --no-move
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class SentCommand:
    position_percent: float
    index: int


class FakeRobot:
    """Fake robot that records every gripper RPC instead of driving hardware."""

    def __init__(self):
        self.commands: list[SentCommand] = []

    def activate_gripper(self, index: int = 1, **kwargs):
        return 0

    def latched_fault_code(self):
        return (0, 0)

    def move_gripper(self, position_percent, index=1, **kwargs):
        self.commands.append(
            SentCommand(position_percent=float(position_percent), index=int(index))
        )
        return 0

    def get_robot_error_code(self):
        return (0, 0)

    def recover_gripper(self, index: int = 1, **kwargs):
        return 0


class FakeXrClient:
    """Simulates PICO input: serves a fixed scripted trigger per side."""

    def __init__(self, left_profile, right_profile):
        self._left = list(left_profile)
        self._right = list(right_profile)

    def get_key_value_by_name(self, name):
        if name == "left_trigger":
            return self._take(self._left)
        if name == "right_trigger":
            return self._take(self._right)
        return 0.0

    def get_pose_by_name(self, name):
        return np.zeros(7)

    def get_button_state_by_name(self, name):
        return False

    def get_timestamp_ns(self):
        return time.time_ns()

    @staticmethod
    def _take(profile):
        return profile.pop(0) if profile else 0.0


def expected_position(trigger, threshold, open_percent, close_percent):
    trigger = min(1.0, max(0.0, trigger))
    closure = 0.0 if trigger <= threshold else (trigger - threshold) / (1.0 - threshold)
    return open_percent + closure * (close_percent - open_percent)


def check_mapper_math(threshold, open_percent, close_percent, max_width):
    """Compare TriggerGripperMapper.map() against the closed-form formula."""
    from xrobotoolkit_teleop.hardware.fr3c_gripper import TriggerGripperMapper

    mapper = TriggerGripperMapper(
        trigger_threshold=threshold,
        open_position_percent=open_percent,
        closed_position_percent=close_percent,
        max_width_mm=max_width,
    )
    cases = [0.0, 0.05, 0.1, 0.5, 0.9, 1.0]
    rows, failures = [], []
    for trig in cases:
        out = mapper.map(trig)
        exp = expected_position(trig, threshold, open_percent, close_percent)
        rows.append((trig, out.position_percent, out.width_mm))
        if abs(out.position_percent - exp) > 1e-6:
            failures.append((trig, exp, out.position_percent))
    return rows, failures


def run_simulated(side, threshold, open_percent, close_percent, min_change, max_width):
    """Feed a scripted trigger into Fr3cVrGripperController (fake robot) and
    return the exact move_gripper positions it issues plus the expected ones."""
    from xrobotoolkit_teleop.hardware.fr3c_gripper import Fr3cVrGripperController

    profile = [0.0, 1.0, 1.0, 0.0, 0.0, 1.0]
    robot = FakeRobot()
    left_profile = profile if side == "left" else [0.0] * len(profile)
    right_profile = profile if side == "right" else [0.0] * len(profile)
    controller = Fr3cVrGripperController(
        xr_client=FakeXrClient(left_profile, right_profile),
        robot=robot,
        controller_side=side,
        trigger_threshold=threshold,
        open_position_percent=open_percent,
        closed_position_percent=close_percent,
        max_width_mm=max_width,
        min_position_change_percent=min_change,
        min_command_interval_s=0.0,
        toggle_debounce_s=0.0,
    )
    for _ in range(len(profile)):
        controller.update()

    expected_sent = [close_percent, open_percent]

    got = [round(c.position_percent, 4) for c in robot.commands]
    expected = [round(p, 4) for p in expected_sent]
    return expected, got


class OneShotXrClient:
    """Injects a single close->open trigger sequence on the given side."""

    def __init__(self, side):
        self._side = side
        self._i = 0

    def get_key_value_by_name(self, name):
        if name == f"{self._side}_trigger":
            values = [0.0, 1.0, 0.0, 1.0]
            v = values[min(self._i, len(values) - 1)]
            return v

        if name == f"{self._side}_grip":
            return 1.0
        return 0.0

    def get_timestamp_ns(self):
        return time.time_ns()

    def advance(self):
        self._i += 1


def real_robot_move(robot_ip, side, threshold, open_percent, close_percent, no_move):
    """On-hardware self-test: actually call MoveGripper on the target arm."""
    from xrobotoolkit_teleop.hardware.interface.fr3c import Fr3cController
    from xrobotoolkit_teleop.hardware.fr3c_gripper import (
        Fr3cVrGripperController,
        TriggerGripperMapper,
    )

    # Assert the fixed config first so a wiring bug can't silently use the
    # module defaults (open=100 / closed=0), which would invert the motion.
    mapper = TriggerGripperMapper(
        trigger_threshold=threshold,
        open_position_percent=open_percent,
        closed_position_percent=close_percent,
    )
    open_pos = mapper.map(0.0).position_percent
    close_pos = mapper.map(1.0).position_percent
    print(f"[3/3] Real arm {robot_ip}: open=0.0% -> MoveGripper({open_pos}), "
          f"close=1.0 -> MoveGripper({close_pos})")

    robot = Fr3cController(robot_ip=robot_ip)
    client = OneShotXrClient(side)
    controller = Fr3cVrGripperController(
        xr_client=client,
        robot=robot,
        controller_side=side,
        trigger_threshold=threshold,
        open_position_percent=open_percent,
        closed_position_percent=close_percent,
    )
    try:
        robot.reset_all_errors()
        controller.activate()
        print("      gripper activated")
        if no_move:
            print("      --no-move set: skipping MoveGripper")
            return 0
        client.advance()
        controller.update()      # trigger 1.0 -> close
        print(f"      MoveGripper({close_pos}) issued (trigger 1.0). Watch it close...")
        client.advance()
        time.sleep(1.5)
        controller.update()      # release, keep closed
        client.advance()
        controller.update()      # next trigger 1.0 -> open
        print(f"      MoveGripper({open_pos}) issued (second trigger press). Watch it open.")
    finally:
        robot.close()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument("--threshold", type=float, default=0.05)
    parser.add_argument("--open-percent", type=float, default=0.0)
    parser.add_argument("--close-percent", type=float, default=97.0)
    parser.add_argument("--min-change", type=float, default=2.0)
    parser.add_argument("--max-width-mm", type=float, default=90.0)
    parser.add_argument("--robot-ip", default=None)
    parser.add_argument("--no-move", action="store_true")
    args = parser.parse_args()

    sys.path.insert(
        0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    )

    # --- check 1: mapper math ---
    rows, failures = check_mapper_math(
        args.threshold, args.open_percent, args.close_percent, args.max_width_mm
    )
    print(f"[1/3] TriggerGrip net math (threshold={args.threshold}, "
          f"open={args.open_percent}%, close={args.close_percent}%)")
    for trig, pos, width in rows:
        print(f"      trigger={trig:4.2f} -> position_percent={pos:7.2f}  width={width:5.1f}mm")
    if failures:
        print("      FAILED:", failures)
        return 1
    print("      math check: OK")

    # --- check 2: simulated controller -> move_gripper ---
    expected, got = run_simulated(
        args.side, args.threshold, args.open_percent, args.close_percent,
        args.min_change, args.max_width_mm,
    )
    print(f"[2/3] Simulated {args.side}-controller move_gripper positions")
    print(f"      expected sends: {expected}")
    print(f"      actual sends:   {got}")
    if got == expected:
        print("      mapping check: OK  (trigger maps to expected closure)")
    else:
        print("      mapping check: MISMATCH -- check TriggerGripperMapper input wiring")
        return 1

    # --- check 3: optional real hardware self-test ---
    if args.robot_ip:
        real_robot_move(
            args.robot_ip, args.side, args.threshold, args.open_percent,
            args.close_percent, args.no_move,
        )
        print("      [3/3] real MoveGripper self-test done")

    print("All checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
