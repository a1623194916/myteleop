"""Delayed, feedback-verified six-axis motor disable for single/dual Piper-X."""

import argparse
import math
import re
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path


def selected_channels(mode, channel, left_channel, right_channel):
    if mode == "single":
        channels = [channel or right_channel]
    else:
        if channel is not None:
            raise ValueError("--channel only applies to --mode single")
        channels = [left_channel, right_channel]
    if any(not name.strip() for name in channels) or len(set(channels)) != len(channels):
        raise ValueError("CAN channels must be nonempty and distinct")
    return channels


def require_teleop_stopped():
    result = subprocess.run(["ps", "-eo", "args="], check=True, capture_output=True, text=True)
    for command in result.stdout.splitlines():
        if re.match(r"^(?:\S*/)?python[\d.]*\s", command) and (
            "teleop_piper_x_hardware.py" in command or "piper_teleop_recorder.teleop_record" in command
        ):
            raise RuntimeError("Piper teleop is still running. Stop it with B / Ctrl+C first.")


def countdown(delay):
    deadline = time.monotonic() + delay
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        print(f"{math.ceil(remaining)} 秒后失能；请托住机械臂，Ctrl+C 取消。", flush=True)
        time.sleep(min(1.0, remaining))


def motor_states(driver, since=0.0):
    """Missing or old feedback cannot prove that a motor is disabled."""
    states = []
    now = time.time()
    for joint in range(1, 7):
        reading = driver.get_driver_states(joint)
        if (reading is None or not math.isfinite(reading.timestamp)
                or not -0.1 <= now - reading.timestamp <= 0.3 or reading.timestamp < since):
            states.append(None)
        else:
            states.append(bool(reading.msg.foc_status.driver_enable_status))
    return states


def disable_arms(arms, timeout=3.0):
    issued_at = time.time()
    deadline = time.monotonic() + timeout
    pending = dict(arms)
    errors = {}
    while pending and time.monotonic() < deadline:
        for channel, arm in list(pending.items()):
            try:
                # Individual joint IDs leave the independent gripper command unchanged.
                for joint in range(1, 7):
                    arm.driver.disable(joint_index=joint)
                states = motor_states(arm.driver, since=issued_at)
                if all(state is False for state in states):
                    print(f"{channel}: 六轴电机已失能，反馈确认 J1–J6 全部关闭。", flush=True)
                    del pending[channel]
            except Exception as exc:
                errors[channel] = str(exc)
        if pending:
            time.sleep(0.1)
    if pending:
        details = []
        for channel, arm in pending.items():
            try:
                states = motor_states(arm.driver, since=issued_at)
                detail = f"J1–J6 enabled={states} (None=反馈缺失/过期)"
            except Exception as exc:
                detail = str(exc)
            details.append(f"{channel}: {detail}; {errors.get(channel, '')}")
        raise RuntimeError("未能确认全部失能；部分电机可能已失能，请继续托住机械臂。 " + "; ".join(details))


def main(argv=None):
    disabling = False
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("single", "dual"), default="dual")
    parser.add_argument("--channel", help="single mode CAN channel; default can1")
    parser.add_argument("--left-channel", default="can0")
    parser.add_argument("--right-channel", default="can1")
    parser.add_argument("--firmware", default=None)
    parser.add_argument("--delay", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=3.0, help="disable confirmation timeout")
    parser.add_argument("--dry-run", action="store_true", help="show targets/countdown without connecting CAN")
    args = parser.parse_args(argv)
    try:
        channels = selected_channels(args.mode, args.channel, args.left_channel, args.right_channel)
        if not math.isfinite(args.delay) or args.delay <= 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError("delay and timeout must be finite and positive")
        require_teleop_stopped()
        print(f"目标 CAN: {', '.join(channels)}；仅失能六轴电机，夹爪保持原状态。", flush=True)
        if args.dry_run:
            countdown(args.delay)
            print("DRY RUN: 未连接 CAN，未发送失能指令。")
            return 0
        sys.path.insert(0, str(Path(__file__).resolve().parent / "XRoboToolkit-Teleop-Sample-Python"))
        from xrobotoolkit_teleop.hardware.piper_x import PiperXCanArm

        with ExitStack() as cleanup:
            arms = {}
            print("正在只读连接所选机械臂；全部连接完成后开始倒计时。", flush=True)
            for channel in channels:
                arm = PiperXCanArm(channel=channel, firmware=args.firmware, read_only=True)
                cleanup.callback(arm.close)
                arms[channel] = arm
            countdown(args.delay)
            require_teleop_stopped()  # Detect a teleop started during the countdown.
            disabling = True
            disable_arms(arms, args.timeout)
        return 0
    except KeyboardInterrupt:
        message = ("已取消；部分电机可能已关闭，请继续托住机械臂。" if disabling
                   else "已取消，未发送失能指令。")
        print(f"\n{message}", flush=True)
        return 130
    except Exception as exc:
        print(f"失能失败: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
