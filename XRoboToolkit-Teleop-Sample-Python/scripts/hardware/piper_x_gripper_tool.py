"""Piper-X AGX gripper diagnostics and zero-point calibration.

The teleop process only ever *reads* the gripper width, and its startup check
("does the width equal the full stroke?") also passes for a gripper whose
position feedback is frozen, so a dead effector silently makes TRIGGER look
unmapped. This tool talks to the effector directly on the CAN channel.

The arm joints are NOT enabled by this tool (no joint motion at all), only the
end effector is commanded.

Examples
--------
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py read
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py release --seconds 60
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py watch --seconds 20
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py raw --seconds 20
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py sweep
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py angle
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py calibrate
    PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py mode --yes

`release` de-energizes the effector: an enabled FOC holds the jaws stiffly, so
this must run before any manual positioning. `watch` needs a human: back-drive
the jaws by hand while it runs. If the printed width does not follow your hand,
the position sensor / zero point is invalid, not the teleop mapping. `raw`
prints the per-byte variability of the feedback frame, which is the evidence to
capture if the effector has to go back to the vendor. `calibrate` releases the
effector first, then needs the jaws held at the closed reference position while
it runs. `mode` retries the effector in each arm motion mode, which is the last
software hypothesis (it writes the arm's motion-mode state, hence `--yes`).
"""

from __future__ import annotations

import argparse
import time

from pyAgxArm import AgxArmFactory, create_agx_arm_config, PiperFW


def open_arm(channel: str, enable: bool = False, speed_percent: int = 100):
    """Connect one Piper-X arm. Motors stay disabled unless `enable` is set."""
    cfg = create_agx_arm_config(
        robot="piper_x",
        firmeware_version=PiperFW.DEFAULT,
        interface="socketcan",
        channel=channel,
        bitrate=1000000,
    )
    arm = AgxArmFactory.create_arm(cfg)
    gripper = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
    arm.connect()
    if enable:
        deadline = time.monotonic() + 5.0
        while not arm.enable():
            if time.monotonic() >= deadline:
                arm.disconnect()
                raise RuntimeError("Piper-X motors did not enable within 5 s")
            time.sleep(0.01)
        arm.set_speed_percent(speed_percent)
    return arm, gripper


def status_line(gripper):
    st = gripper.get_gripper_status()
    if st is None:
        return "  <no gripper feedback frame>"
    m, f = st.msg, st.msg.foc_status
    age_ms = (time.time() - st.timestamp) * 1000
    unit = "mm" if m.mode == "width" else "deg"
    factor = 1000.0 if m.mode == "width" else 1.0
    return (f"  mode={m.mode:5s} value={m.value * factor:9.2f} {unit:3s} "
            f"force={m.force:7.3f} N status=0x{m.status_code:02x} age={age_ms:5.1f}ms | "
            f"enabled={f.driver_enable_status} homed={f.homing_status} "
            f"vlow={f.voltage_too_low} motor_hot={f.motor_overheating} "
            f"overcurrent={f.driver_overcurrent} driver_hot={f.driver_overheating} "
            f"sensor={f.sensor_status} driver_error={f.driver_error_status}")


def cmd_read(arm, gripper, args):
    print("firmware:", arm.get_firmware())
    print("channel :", arm.get_channel())
    print("\ngripper feedback (5 samples):")
    for _ in range(5):
        print(status_line(gripper), flush=True)
        time.sleep(0.2)
    print("\nteaching-pendant params:")
    param = gripper.get_gripper_teaching_pendant_param(timeout=1.5)
    if param is None:
        print("  unavailable")
    else:
        for key, value in vars(param.msg).items():
            print(f"  {key} = {value}")
    print("\nctrl-state echo (0x159 as received back):")
    ctrl = gripper.get_gripper_ctrl_states()
    print("  " + ("none" if ctrl is None else str(vars(ctrl.msg))))
    print("\nNOTE on the status bits of this firmware: `enabled` (bit6) toggles between frames and")
    print("`homed` (bit7) stays 0 even on a fully working, freshly zeroed gripper. Neither is a")
    print("health signal - judge the effector by whether `value` follows the commanded width.")


def cmd_watch(arm, gripper, args):
    print(f"Watching the gripper for {args.seconds:g} s. "
          "Back-drive the jaws by hand and check that the width follows.")
    print("Joint motors stay disabled. Ctrl+C stops early.")
    print(f"  start: {status_line(gripper)}", flush=True)
    deadline = time.monotonic() + args.seconds
    samples, t0 = [], time.monotonic()
    try:
        while time.monotonic() < deadline:
            st = gripper.get_gripper_status()
            if st is not None and st.msg.mode == "width":
                samples.append(st.msg.value)
                print(f"  t={time.monotonic() - t0:5.1f}s width={st.msg.value * 1000:9.2f} mm "
                      f"force={st.msg.force:7.3f} N status=0x{st.msg.status_code:02x}", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    if samples:
        lo, hi = min(samples) * 1000, max(samples) * 1000
        print(f"\nrange over {len(samples)} samples: {lo:.2f} .. {hi:.2f} mm (span {hi - lo:.2f} mm)")
        if hi - lo < 0.5:
            print("VERDICT: the width never changed. Either the jaws were not moved, or the "
                  "gripper position sensor / zero point is invalid. Re-run and move the jaws "
                  "by hand; if it still never changes, calibrate (or the effector is faulty).")
        else:
            print("VERDICT: the position sensor tracks the jaws, so the gripper hardware is "
                  "alive; the problem is the motion command path, not the teleop mapping.")


def cmd_sweep(arm, gripper, args):
    targets = args.widths or [0.10, 0.07, 0.04, 0.00, 0.04, 0.07, 0.10]
    print(f"Sweeping widths: {[f'{w * 1000:.0f}mm' for w in targets]}")
    print(f"  start: {status_line(gripper)}", flush=True)
    observed = []
    for target in targets:
        print(f"\n>>> width={target * 1000:.1f} mm force={args.force:g} N", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.hold:
            gripper.move_gripper_m(value=target, force=args.force)
            print(f"    t={time.monotonic() - t0:4.2f}s {status_line(gripper)}", flush=True)
            time.sleep(0.2)
        st = gripper.get_gripper_status()
        if st is not None:
            observed.append((target * 1000, st.msg.value * 1000))
    _sweep_verdict(observed)


def _sweep_verdict(observed):
    """Name the failure mode instead of leaving 'nothing moved' unexplained."""
    if not observed:
        print("\nVERDICT: no gripper feedback at all - check CAN, effector power and connector.")
        return
    errors = [abs(got - want) for want, got in observed]
    if max(errors) <= 5.0:
        print(f"\nVERDICT: the jaws track every target (max error {max(errors):.1f} mm). Healthy.")
        return
    spread = max(g for _, g in observed) - min(g for _, g in observed)
    print(f"\nVERDICT: width barely moved (span {spread:.1f} mm, max error {max(errors):.1f} mm).")
    if spread < 0.5:
        print("  The jaws did not move at all. Causes, in the order we have actually hit them:")
        print("  1. Zero point sits at the wrong end. `value` is measured from the last set_zero, so")
        print("     if the zero was latched with the jaws OPEN, every positive width is already at the")
        print("     stop and negative widths are clamped away. Fix: `release`, push the jaws CLOSED,")
        print("     then `calibrate`.")
        print("  2. Effector still disabled from an earlier `release`/disable. Send a motion frame and")
        print("     confirm `value` reacts at all; note that the `enabled` bit toggles on this firmware")
        print("     and is NOT a usable health signal.")
        print("  3. Loose tool-flange connector: the frame stays byte-identical and force never varies.")
        print("     Check with `raw`; reseat the connector.")
    else:
        print("  Partial motion only - the commanded range is being clipped, so the zero point or")
        print("  max_range_config does not match the real stroke.")


def cmd_angle(arm, gripper, args):
    """Angle mode is independent of the stroke/zero calibration."""
    print("Angle-mode probe (status_code 0x05). Joint motors disabled.")
    for deg in args.degs or [0.0, 10.0, -10.0, 0.0]:
        print(f"\n>>> angle={deg:g} deg force={args.force:g} N", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.hold:
            gripper.move_gripper_deg(value=deg, force=args.force)
            print(f"    t={time.monotonic() - t0:4.2f}s {status_line(gripper)}", flush=True)
            time.sleep(0.2)


def cmd_raw(arm, gripper, args):
    """Per-byte variability of the gripper feedback frame (RMA evidence)."""
    import can
    from collections import defaultdict

    wanted = {int(i, 16) for i in args.ids}
    frames = defaultdict(list)
    bus = can.interface.Bus(interface="socketcan", channel=args.channel,
                            bitrate=1000000, receive_own_messages=False)
    print(f"Dumping {[f'0x{i:03X}' for i in sorted(wanted)]} for {args.seconds:g} s.")
    print("Move the jaws by hand during the dump; a live sensor must show +/-1 LSB jitter.")
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.5)
            if msg is None or msg.is_error_frame:
                continue
            if msg.arbitration_id in wanted:
                frames[msg.arbitration_id].append(bytes(msg.data))
    finally:
        bus.shutdown()

    for aid, data in sorted(frames.items()):
        print(f"\n0x{aid:03X}: {len(data)} frames, {len(set(data))} distinct payload(s)")
        for index in range(8):
            values = sorted({d[index] for d in data})
            shown = ", ".join(f"0x{v:02X}" for v in values[:6]) + (" ..." if len(values) > 6 else "")
            print(f"  byte{index}: {'VARIES' if len(values) > 1 else 'FIXED '} "
                  f"({len(values):>3} values) {shown}")
    if frames:
        print("\nIf every byte is FIXED while you were moving the jaws, the width field is a "
              "frozen register rather than a measurement: the effector's position feedback is "
              "not live. Capture this table for the vendor.")


def cmd_mode(arm, gripper, args):
    """Retry the gripper in each arm motion mode (last software hypothesis)."""
    modes = args.modes or ["p", "j", "mit"]
    print("For each arm motion mode: set it, then retry a small gripper close.")
    print("Only the end effector is commanded; no joint target is ever sent.")
    for mode in modes:
        try:
            arm.set_motion_mode(mode)
            print(f"\n>>> motion_mode={mode!r}", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"\n>>> motion_mode={mode!r} raised {type(exc).__name__}: {exc}", flush=True)
            continue
        time.sleep(0.3)
        print("   " + status_line(gripper), flush=True)
        for target in (0.05, 0.02, 0.05):
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                gripper.move_gripper_m(value=target, force=args.force)
                time.sleep(0.2)
            st = gripper.get_gripper_status()
            width = "n/a" if st is None else f"{st.msg.value * 1000:.2f} mm"
            print(f"   cmd {target * 1000:5.1f} mm -> read {width}", flush=True)


def cmd_release(arm, gripper, args):
    """De-energize the effector so the jaws can be back-driven by hand.

    An enabled FOC holds the jaws stiffly enough that they cannot be pushed by
    hand, so this must run before any manual positioning or zeroing.
    """
    print("Releasing the gripper (FOC disabled) so the jaws can be moved by hand.")
    print(f"Keeping it released for {args.seconds:g} s and logging the width every 0.2 s.")
    print("Push the jaws open and closed a few times during this window.")
    deadline = time.monotonic() + args.seconds
    t0 = time.monotonic()
    samples = []
    while time.monotonic() < deadline:
        gripper.disable_gripper()
        st = gripper.get_gripper_status()
        if st is not None:
            m = st.msg
            samples.append(m.value)
            print(f"  t={time.monotonic() - t0:6.1f}s mode={m.mode} "
                  f"width={m.value * 1000:9.2f} mm force={m.force:7.3f} N "
                  f"status=0x{m.status_code:02x} en={m.foc_status.driver_enable_status}",
                  flush=True)
        time.sleep(0.2)
    if samples:
        lo, hi = min(samples) * 1000, max(samples) * 1000
        print(f"\nrange over {len(samples)} samples: {lo:.2f} .. {hi:.2f} mm (span {hi - lo:.2f} mm)")
        if hi - lo < 0.5:
            print("VERDICT: the width did not follow the jaws. If you did push them, the "
                  "position feedback is not live (encoder / cable / effector fault).")
        else:
            print("VERDICT: the position feedback follows the jaws, so the encoder is live. "
                  "The remaining fault is on the motor drive side.")


def cmd_calibrate(arm, gripper, args):
    """Vendor zero-point sequence: mode 0x00 first, then 0xAE to latch the zero."""
    print("Piper AGX gripper zero-point calibration.")
    print("Reference: the gripper's own UI sends Ctrl(value=0, force=1000, status=0x00),")
    print("waits 1.5 s, then the same frame with set_zero=0xAE. The zero point is")
    print("whichever jaw position the gripper is in when the second frame lands.")

    # Disable FIRST. An enabled FOC holds the jaws stiffly, so asking for the
    # reference position before this point is unsatisfiable by hand.
    print("\nstep 0/3: release the effector so the jaws can be moved by hand...")
    for _ in range(5):
        gripper.disable_gripper()
        time.sleep(0.1)
    print("   " + status_line(gripper))

    if not args.yes:
        print("\nPush the jaws to the closed reference position by hand and keep holding them.")
        input("Press Enter while still holding them closed, Ctrl+C to abort... ")

    print("\nstep 1/3: hold mode 0x00 (disable/width) for 1.5 s so the driver settles...")
    # The vendor UI keeps the frame at status_code 0x00 for the whole settle window.
    # Sending 0x01 (enable/width) here re-arms the position loop and the following
    # 0xAE then no-ops, which is why the earlier attempt ACKed but never homed.
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        gripper.disable_gripper()
        time.sleep(0.2)
    print("   " + status_line(gripper))

    print("step 2/3: latch the zero point (set_zero=0xAE)...")
    ok = gripper.calibrate_gripper(timeout=2.0)
    print(f"   calibrate_gripper() -> {ok}")
    time.sleep(0.5)
    print("   " + status_line(gripper))

    print(f"\nstep 3/3: restoring the configured stroke (max_range_config={args.max_range:g} m)...")
    print("   set_gripper_teaching_pendant_param ->",
          gripper.set_gripper_teaching_pendant_param(max_range_config=args.max_range))

    st = gripper.get_gripper_status()
    if st is not None and st.msg.status_code & 0x80:
        print("\nRESULT: homing_status is now set. Probing 3 mm / 6 mm to confirm the jaws move.")
        for probe in (0.003, 0.006):
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.5:
                gripper.move_gripper_m(value=probe, force=args.force)
                time.sleep(0.2)
            st = gripper.get_gripper_status()
            width = "n/a" if st is None else f"{st.msg.value * 1000:.2f} mm"
            print(f"   cmd {probe * 1000:4.1f} mm -> read {width}", flush=True)
    else:
        print("\nRESULT: homing_status is still clear. Re-run with the jaws fully closed, and "
              "check that the gripper is powered and no fault bit is set.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default="can1", help="SocketCAN channel (default can1)")
    parser.add_argument("--force", type=float, default=2.0, help="gripper force in N (default 2)")
    parser.add_argument("--enable-arm", action="store_true",
                        help="enable the six joint motors too (default: joints stay disabled)")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("read", help="print gripper status and teaching params")

    watch = sub.add_parser("watch", help="print the width continuously (move the jaws by hand)")
    watch.add_argument("--seconds", type=float, default=20.0)

    sweep = sub.add_parser("sweep", help="command a width sweep and watch the reading")
    sweep.add_argument("--widths", type=float, nargs="*", default=None, help="widths in metres")
    sweep.add_argument("--hold", type=float, default=1.5, help="seconds per target")

    angle = sub.add_parser("angle", help="probe angle mode (independent of stroke calibration)")
    angle.add_argument("--degs", type=float, nargs="*", default=None)
    angle.add_argument("--hold", type=float, default=1.0)

    cal = sub.add_parser("calibrate", help="run the vendor zero-point sequence")
    cal.add_argument("--max-range", type=float, default=0.10, choices=(0.07, 0.1))
    cal.add_argument("--yes", action="store_true", help="skip the interactive confirmation")

    raw = sub.add_parser("raw", help="per-byte variability of the feedback frame (RMA evidence)")
    raw.add_argument("--seconds", type=float, default=20.0)
    raw.add_argument("--ids", nargs="*", default=["2A8"])

    rel = sub.add_parser("release", help="de-energize the jaws so they can be moved by hand")
    rel.add_argument("--seconds", type=float, default=60.0)

    mode = sub.add_parser("mode", help="retry the gripper in each arm motion mode")
    mode.add_argument("--modes", nargs="*", default=None, choices=("p", "j", "l", "c", "mit", "js", "cpv"))
    mode.add_argument("--yes", action="store_true",
                      help="required: this writes the arm's motion-mode state")

    args = parser.parse_args()
    if args.command == "mode" and not args.yes:
        parser.error("'mode' changes the arm's motion-mode state; re-run with --yes")
    arm, gripper = open_arm(args.channel, enable=args.enable_arm)
    try:
        print(f"connected to {args.channel}\n  start: {status_line(gripper)}", flush=True)
        {"read": cmd_read, "watch": cmd_watch, "sweep": cmd_sweep, "raw": cmd_raw,
         "release": cmd_release, "mode": cmd_mode, "angle": cmd_angle,
         "calibrate": cmd_calibrate}[args.command](arm, gripper, args)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
