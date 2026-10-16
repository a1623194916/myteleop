"""Measure MIT servo tracking error for a small step, per (kp, kd) pair.

This MOVES THE ARM. The step is small (1.5 deg default) and every joint is
commanded every cycle, so the arm keeps holding the joints that are not being
tested. Use it to pick kp/kd from data instead of by feel.

Run:
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_mit_tune.py --channel can1
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from pyAgxArm import AgxArmFactory, create_agx_arm_config, PiperFW


def hold(arm, targets, kp, kd, cycles, rate_hz):
    """Command all six joints at `targets` for `cycles`, then read feedback."""
    period = 1.0 / rate_hz
    for _ in range(cycles):
        started = time.monotonic()
        for index, position in enumerate(targets, 1):
            arm.move_mit(index, p_des=float(position), v_des=0.0,
                         kp=kp, kd=kd, t_ff=0.0)
        time.sleep(max(0.0, period - (time.monotonic() - started)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="can1")
    parser.add_argument("--joint", type=int, default=4, choices=range(1, 7),
                        help="joint index to step (4/5/6 are the wrist)")
    parser.add_argument("--step-deg", type=float, default=1.5)
    parser.add_argument("--gains", nargs="*", default=["10:0.8", "25:1.5", "50:2.5"])
    parser.add_argument("--settle-s", type=float, default=1.5)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--yes", action="store_true", help="required: this moves the arm")
    args = parser.parse_args()
    if not args.yes:
        parser.error("this command moves the arm; re-run with --yes")

    cfg = create_agx_arm_config(robot="piper_x", firmeware_version=PiperFW.DEFAULT,
                                interface="socketcan", channel=args.channel,
                                bitrate=1000000)
    arm = AgxArmFactory.create_arm(cfg)
    arm.connect()
    try:
        deadline = time.monotonic() + 5.0
        while not arm.enable():
            if time.monotonic() >= deadline:
                raise RuntimeError("motors did not enable within 5 s")
            time.sleep(0.01)
        arm.set_speed_percent(100)

        baseline = np.asarray(arm.get_joint_angles().msg, dtype=float)
        print(f"holding at measured joints (deg): {np.rad2deg(baseline).round(2)}", flush=True)
        step = np.deg2rad(args.step_deg)
        index = args.joint - 1
        print(f"stepping joint {args.joint} by {args.step_deg:g} deg; "
              f"all six joints are commanded every cycle.\n", flush=True)
        print(f"{'kp:kd':>10} {'achieved':>10} {'error':>9} {'error % of step':>16}")

        for pair in args.gains:
            kp_text, kd_text = pair.split(":")
            kp, kd = float(kp_text), float(kd_text)
            target = baseline.copy()
            target[index] += step
            hold(arm, baseline, kp, kd, int(args.settle_s * args.rate_hz / 4), args.rate_hz)
            hold(arm, target, kp, kd, int(args.settle_s * args.rate_hz), args.rate_hz)
            samples = []
            for _ in range(8):
                hold(arm, target, kp, kd, int(0.02 * args.rate_hz), args.rate_hz)
                samples.append(np.asarray(arm.get_joint_angles().msg, dtype=float))
            achieved = np.mean(samples, axis=0)
            error_deg = np.rad2deg(abs(achieved[index] - target[index]))
            fraction = 100.0 * error_deg / abs(args.step_deg)
            print(f"{pair:>10} {np.rad2deg(achieved[index]):9.3f}° {error_deg:8.3f}° "
                  f"{fraction:15.0f}%", flush=True)

        print("\nrestoring the measured pose...", flush=True)
        hold(arm, baseline, 10.0, 0.8, int(1.0 * args.rate_hz), args.rate_hz)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
