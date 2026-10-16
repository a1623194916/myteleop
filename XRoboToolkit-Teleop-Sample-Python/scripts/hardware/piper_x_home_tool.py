"""Drive the Piper-X arms back to the saved home pose without starting teleop.

    show  read the joints and print the distance to the saved home (read-only)
    go    drive the selected arms to the saved home pose

The target comes from the same configs/piper_x_home.json the teleop home button
reads, through the same bounded-correction code and the same slew limit, so
this tool lands on exactly the pose the button does. Only J1-J6 are commanded;
the effector (gripper) is left untouched. Motors stay enabled after arrival so
the arm holds the pose - run 'bash disable_piper_x.sh' to release them.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
TELEOP_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_HOME_CONFIG = PROJECT_ROOT / "configs" / "piper_x_home.json"
# 15 deg/s at 100 Hz, the rate the teleop home button already uses.
DEFAULT_SLEW_DEG = 0.15

# Any of these holding a CAN channel would fight this tool for the motors.
BUSY_MARKERS = (
    "teleop_piper_x_hardware.py",
    "piper_teleop_recorder.teleop_record",
    "piper_x_gripper_tool.py",
    "piper_x_mit_tune.py",
    "piper_x_home_tool.py",
)

sys.path.insert(0, str(TELEOP_ROOT))

import mujoco  # noqa: E402

from scripts.hardware.teleop_piper_x_hardware import select_arms  # noqa: E402
from xrobotoolkit_teleop.hardware.piper_x import (  # noqa: E402
    JointFollower, JointTrackingError, PiperXCanArm,
)
from xrobotoolkit_teleop.hardware.piper_x_home import (  # noqa: E402
    bounded_home_target, load_home_targets,
)
from xrobotoolkit_teleop.simulation.piper_x_model import JOINTS, build_scene  # noqa: E402


def can_link_state(channel):
    """Read a SocketCAN channel's error state.

    A powered-off arm stops acknowledging frames, so the channel falls out of
    ERROR-ACTIVE and the SDK then reports only 'firmware version unavailable'.
    Reporting the state turns that dead end into an obvious cabling/power hint.
    """
    try:
        result = subprocess.run(["ip", "-details", "link", "show", channel],
                                capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    match = re.search(r"\bcan state (\S+)", result.stdout)
    return match.group(1) if match else "unknown"


def can_link_report(channels):
    """One line per channel plus a hint when a channel is not healthy."""
    states = [(channel, can_link_state(channel)) for channel in channels.values()]
    report = "CAN link state: " + ", ".join(f"{channel}={state}" for channel, state in states)
    unhealthy = [channel for channel, state in states
                 if state not in ("ERROR-ACTIVE", "unknown")]
    if unhealthy:
        report += (f"\n{', '.join(unhealthy)} is not ERROR-ACTIVE. That normally means the arm "
                   f"is powered off or the CAN cable is loose - it is not a software fault.")
    return report


def require_can_free():
    """Refuse to share a CAN channel with another Piper process."""
    mine = os.getpid()
    try:
        listing = subprocess.run(["ps", "-eo", "pid=,args="], check=True,
                                 capture_output=True, text=True).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError(f"cannot list processes to check for a running teleop: {exc}")
    conflicts = []
    for line in listing.splitlines():
        pid, _, command = line.strip().partition(" ")
        if not pid.isdigit() or int(pid) == mine:
            continue
        if not re.match(r"^(?:\S*/)?python[\d.]*\s", command):
            continue
        if any(marker in command for marker in BUSY_MARKERS):
            conflicts.append(f"pid {pid}: {command[:120]}")
    if conflicts:
        raise RuntimeError("another Piper process is using the CAN bus; stop it first:\n  "
                           + "\n  ".join(conflicts))


def joint_limits():
    """The J1-J6 limits from the same MuJoCo scene teleop builds."""
    _, scene = build_scene()
    model = mujoco.MjModel.from_xml_path(str(scene))
    ids = [model.joint(name).id for name in JOINTS]
    limits = model.jnt_range[ids]
    return limits[:, 0].copy(), limits[:, 1].copy()


def home_states(targets, lower, upper):
    """Bounded target, correction and feedback margin per arm."""
    states = {}
    for side, raw in targets.items():
        target, correction, margin = bounded_home_target(side, raw, lower, upper)
        print(f"{side} saved home (deg): {np.rad2deg(np.asarray(raw)).round(3)}", flush=True)
        if correction > 1e-6:
            print(f"{side} WARNING: saved pose lies outside nominal limits; "
                  f"bounded target (deg): {np.rad2deg(target).round(3)}", flush=True)
        states[side] = {"target": target, "correction": correction, "margin": margin}
    return states


def describe(measured, state):
    delta = np.rad2deg(state["target"] - measured)
    return (f"  current (deg): {np.rad2deg(measured).round(3)}\n"
            f"  target  (deg): {np.rad2deg(state['target']).round(3)}\n"
            f"  delta   (deg): {delta.round(3)}\n"
            f"  max |delta| = {float(np.max(np.abs(delta))):.3f} deg")


def cmd_show(arms, states, args):
    for side, arm in arms.items():
        measured = np.asarray(arm.read_joints(), dtype=float)
        print(f"\n{side} @ {arm.channel}")
        print(describe(measured, states[side]), flush=True)
    print("\nRead-only: no motor was enabled and no command was sent.")
    return 0


def cmd_go(arms, states, args, lower, upper):
    period = 1.0 / args.rate_hz
    tolerance = np.deg2rad(args.tolerance_deg)
    slew_rad = np.deg2rad(args.slew_deg)

    pending, measured = {}, {}
    for side, state in states.items():
        state["follower"] = JointFollower(
            lower, upper, slew_rad,
            max_error_rad=np.deg2rad(args.tracking_error_deg),
            feedback_limit_margin_rad=state["margin"])
        measured[side] = np.asarray(arms[side].read_joints(), dtype=float)
        print(f"\n{side} @ {arms[side].channel}")
        print(describe(measured[side], state), flush=True)
        if float(np.max(np.abs(measured[side] - state["target"]))) <= tolerance:
            print(f"{side}: already at home; no command will be sent.", flush=True)
        else:
            pending[side] = state

    if not pending:
        print("\nAll selected arms are already at the saved home pose.")
        return 0

    speed = args.slew_deg * args.rate_hz
    worst_rad = max(float(np.max(np.abs(measured[side] - state["target"])))
                    for side, state in pending.items())
    eta = np.rad2deg(worst_rad) / max(speed, 1e-6)
    timeout = args.timeout if args.timeout > 0 else max(30.0, 3.0 * eta + 10.0)
    print(f"\nMoving J1-J6 to home at {speed:g} deg/s "
          f"(software slew {args.slew_deg:g} deg/cycle @ {args.rate_hz:g} Hz); "
          f"worst joint needs about {eta:.1f} s, timeout {timeout:.0f} s.", flush=True)
    print("The gripper is not commanded; motors stay enabled after arrival.", flush=True)

    if not args.yes:
        end = time.monotonic() + args.delay
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            print(f"starting in {math.ceil(remaining)} s; Ctrl+C cancels before any motion.",
                  flush=True)
            time.sleep(min(1.0, remaining))

    done = set()
    next_tick = time.monotonic()
    deadline = next_tick + timeout
    try:
        while len(done) < len(pending):
            for side in list(pending):
                if side in done:
                    continue
                state = states[side]
                measured[side] = np.asarray(arms[side].read_joints(), dtype=float)
                error = float(np.max(np.abs(measured[side] - state["target"])))
                if error <= tolerance:
                    done.add(side)
                    print(f"{side}: home reached "
                          f"(max joint error {np.rad2deg(error):.3f} deg).", flush=True)
                    continue
                command = state["follower"].next(measured[side], state["target"], True)
                arms[side].move_joints(command, motion_mode=args.motion_mode)
            if time.monotonic() > deadline:
                outstanding = ", ".join(side for side in pending if side not in done)
                raise RuntimeError(f"return home timed out after {timeout:.0f} s; "
                                   f"not at home: {outstanding}")
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except JointTrackingError as exc:
        # Stop sending rather than pushing a lagging arm; leave the motors enabled
        # so the arm holds whatever pose it reached.
        raise RuntimeError(f"{exc}; stopped sending commands, motors still enabled") from exc

    # Confirm the pose settled instead of reading a single crossing sample.
    time.sleep(0.5)
    for side in done:
        held = np.asarray(arms[side].read_joints(), dtype=float)
        error = float(np.max(np.abs(held - states[side]["target"])))
        print(f"{side}: settled at (deg) {np.rad2deg(held).round(3)} "
              f"(max joint error {np.rad2deg(error):.3f} deg).", flush=True)
        if error > tolerance:
            raise RuntimeError(f"{side} drifted out of tolerance after arriving")
    print("\nHome pose reached on every selected arm. Motors remain enabled; "
          "run 'bash disable_piper_x.sh' when you want them released.")
    return 0


def open_arms(channels, firmware, read_only, speed_percent):
    """Connect every selected arm, or none of them."""
    stack = ExitStack()
    arms = {}
    try:
        for side, channel in channels.items():
            arm = PiperXCanArm(channel=channel, firmware=firmware, read_only=read_only,
                               speed_percent=speed_percent)
            stack.callback(arm.close)
            arms[side] = arm
    except BaseException:
        stack.close()
        raise
    return stack, arms


def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", nargs="?", choices=("show", "go"), default="show",
                        help="show = read-only distance to home; go = move to home (default: show)")
    parser.add_argument("--mode", choices=("single", "dual"), default="dual")
    parser.add_argument("--controller-side", choices=("left", "right"), default="right",
                        help="single mode only; left maps to can0, right to can1")
    parser.add_argument("--channel", help="single mode CAN channel; overrides the side default")
    parser.add_argument("--left-channel", default="can0")
    parser.add_argument("--right-channel", default="can1")
    parser.add_argument("--firmware", default=None)
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG))
    parser.add_argument("--speed-percent", type=int, default=100,
                        help="driver speed ceiling; the software slew below is the real limit")
    parser.add_argument("--motion-mode", choices=("j", "js", "mit"), default="j",
                        help="j = planned motion, matching the teleop home button")
    parser.add_argument("--slew-deg", type=float, default=DEFAULT_SLEW_DEG,
                        help="max joint step per cycle (default 0.15 at 100 Hz = 15 deg/s)")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--tolerance-deg", type=float, default=0.5)
    parser.add_argument("--tracking-error-deg", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=0.0,
                        help="seconds; 0 derives it from the distance and the slew rate")
    parser.add_argument("--delay", type=float, default=3.0,
                        help="countdown before the first command; skip with --yes")
    parser.add_argument("--yes", action="store_true", help="skip the countdown")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the targets and distances without opening CAN")
    return parser


def validate(args):
    checks = (("slew-deg", args.slew_deg, True), ("rate-hz", args.rate_hz, True),
              ("tolerance-deg", args.tolerance_deg, True),
              ("tracking-error-deg", args.tracking_error_deg, True),
              ("delay", args.delay, True), ("timeout", args.timeout, False))
    for name, value, must_be_positive in checks:
        if not math.isfinite(value) or (value <= 0 if must_be_positive else value < 0):
            raise ValueError(f"--{name} is out of range")
    if not 1 <= args.speed_percent <= 100:
        raise ValueError("--speed-percent must be within 1..100")


def main(argv=None):
    args = build_parser().parse_args(argv)
    moving = False
    channels = {}
    try:
        validate(args)
        channels = select_arms(args.mode, args.controller_side, args.channel,
                               args.left_channel, args.right_channel)
        targets = load_home_targets(args.home_config, channels)
        lower, upper = joint_limits()
        states = home_states(targets, lower, upper)
        read_only = args.command == "show"

        if args.dry_run:
            print(f"\nDRY RUN: would {'read' if read_only else 'drive'} "
                  f"{', '.join(f'{side}={channel}' for side, channel in channels.items())}; "
                  f"no CAN channel opened.")
            return 0

        require_can_free()
        print(f"\nOpening {'(read-only) ' if read_only else ''}"
              f"{', '.join(f'{side}={channel}' for side, channel in channels.items())}. "
              f"For a single arm use --mode single --controller-side right --channel can1.",
              flush=True)
        stack, arms = open_arms(channels, args.firmware, read_only, args.speed_percent)
        try:
            if read_only:
                return cmd_show(arms, states, args)
            moving = True
            return cmd_go(arms, states, args, lower, upper)
        finally:
            stack.close()
    except KeyboardInterrupt:
        message = ("Stopped sending commands; motors stay enabled and hold the current pose. "
                   "Use 'bash disable_piper_x.sh' to release them." if moving
                   else "Cancelled before any command was sent.")
        print(f"\n{message}", flush=True)
        return 130
    except Exception as exc:
        detail = f"\n{can_link_report(channels)}" if channels else ""
        print(f"home tool failed: {exc}{detail}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
