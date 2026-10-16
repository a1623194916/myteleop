"""PICO single/dual Piper-X teleoperation over independent SocketCAN channels."""

import sys
import time
from pathlib import Path
from typing import Literal

import numpy as np
import tyro

from xrobotoolkit_teleop.hardware.piper_x import JointFollower, JointTrackingError, PiperXCanArm
from xrobotoolkit_teleop.hardware.piper_x_home import JointHomeReturn, load_home_targets
from xrobotoolkit_teleop.simulation.piper_x_model import JOINTS, TCP_LINK, TCP_OFFSET_M, build_scene


def select_arms(mode, controller_side, channel, left_channel, right_channel):
    """Resolve hand/channel mapping before opening XR, cameras or CAN."""
    if mode not in {"single", "dual"} or controller_side not in {"left", "right"}:
        raise ValueError("Use mode single/dual and controller-side left/right")
    if mode == "dual":
        if channel is not None:
            raise ValueError("--channel is for single mode; dual uses --left-channel and --right-channel")
        if left_channel == right_channel:
            raise ValueError("Dual Piper-X requires two different CAN channels")
        selected = {"left": left_channel, "right": right_channel}
    else:
        selected = {controller_side: channel or (left_channel if controller_side == "left" else right_channel)}
    if any(not value.strip() for value in selected.values()):
        raise ValueError("CAN channel names must not be empty")
    return selected


def validate_controller(xr, side):
    pose = np.asarray(xr.get_pose_by_name(f"{side}_controller"), dtype=float)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:]) < 0.5:
        raise RuntimeError(f"{side} controller pose unavailable; stopping teleoperation")
    for key in ("grip", "trigger"):
        if not np.isfinite(xr.get_key_value_by_name(f"{side}_{key}")):
            raise RuntimeError(f"{side} controller {key} is non-finite")


def wait_for_pico(xr, sides, timeout=0.0, motors_enabled=False, check_feedback=None):
    """Wait for a new packet with all selected hands valid, before opening CAN."""
    started = time.monotonic()
    timestamp = xr.get_timestamp_ns()
    next_status = started + 10.0
    motor_note = ("Motors enabled; no joint commands while waiting." if motors_enabled
                  else "Motors are not enabled by this process.")
    print(f"Waiting for PICO ({', '.join(sides)}); connect headset to PC Service at any time. "
          f"{motor_note} Ctrl+C cancels.", flush=True)
    while True:
        now = time.monotonic()
        if check_feedback is not None:
            check_feedback()
        current = xr.get_timestamp_ns()
        if current > 0 and current != timestamp:
            timestamp = current
            try:
                for side in sides:
                    validate_controller(xr, side)
            except RuntimeError:
                pass  # Packets can arrive before all selected controllers are tracked.
            else:
                print("PICO ready: live packets and selected controller poses received.", flush=True)
                return
        if timeout > 0 and now - started >= timeout:
            raise RuntimeError(f"No valid live PICO controllers after {timeout:g} seconds")
        if now >= next_status:
            print("Still waiting for PICO connection / selected controller tracking...", flush=True)
            next_status = now + 10.0
        time.sleep(0.02)


def hold_for_rearm(runtimes, homes, recording=None):
    for side, runtime in runtimes.items():
        runtime.release_clutch()
        runtime.rearm_required = True
        runtime.recovering = False
        homes[side].returning = False
        homes[side].blocked_until_release = True
        homes[side].last_pressed = True
        homes[side].follower.last_command = None
    if recording is not None:
        recording.pause()


def pause_for_pico(xr, runtimes, homes, recording=None):
    """No motion sends while Send is off; require fresh anchors on resume."""
    print("PICO Send/tracking paused: holding commands; waiting for live data. Ctrl+C exits.", flush=True)
    hold_for_rearm(runtimes, homes, recording)
    def check_feedback():
        for runtime in runtimes.values():
            runtime.arm.read_joints()
    wait_for_pico(xr, runtimes, motors_enabled=True, check_feedback=check_feedback)
    print("PICO resumed: release GRIP, then GRIP again to take over.", flush=True)


class TeleopArm:
    """One arm's IK, clutch and gripper state; XR is shared across both arms."""

    def __init__(self, side, channel, xr, urdf, scene, scale_factor, max_joint_step_deg=1.2,
                 rate_hz=50.0, gripper_force=5.0, no_pose_filter=False,
                 position_deadband=0.001, rotation_deadband_deg=0.3,
                 max_linear_speed=0.5, max_angular_speed=1.5,
                 oneeuro_min_cutoff=3.0, oneeuro_beta=40.0, unlimited_motion=False,
                 oneeuro_derivative_cutoff=10.0, oneeuro_rotation_min_cutoff=3.0,
                 oneeuro_rotation_beta=3.0, servo_slew_deg=None,
                 tracking_error_deg=20.0, tracking_timeout=0.8,
                 gripper_slew_mm_s=250.0):
        from xrobotoolkit_teleop.hardware.piper_x_controller import PiperTeleopController

        self.side, self.channel, self.xr = side, channel, xr
        self.controller = PiperTeleopController(
            xml_path=str(scene), robot_urdf_path=str(urdf),
            manipulator_config={f"{side}_hand": {
                "link_name": TCP_LINK, "pose_source": f"{side}_controller",
                "control_trigger": f"{side}_grip", "vis_target": "target",
            }},
            scale_factor=scale_factor, xr_client=xr, dt=1.0 / rate_hz,
            pose_filter=not no_pose_filter, position_deadband=position_deadband,
            rotation_deadband_deg=rotation_deadband_deg,
            max_linear_speed=max_linear_speed, max_angular_speed=max_angular_speed,
            oneeuro_min_cutoff=oneeuro_min_cutoff, oneeuro_beta=oneeuro_beta,
            unlimited_motion=unlimited_motion,
            oneeuro_derivative_cutoff=oneeuro_derivative_cutoff,
            oneeuro_rotation_min_cutoff=oneeuro_rotation_min_cutoff,
            oneeuro_rotation_beta=oneeuro_rotation_beta,
        )
        self.model = self.controller.mj_model
        self.joint_ids = [self.model.joint(name).id for name in JOINTS]
        limits = self.model.jnt_range[self.joint_ids]
        # The MIT servo is soft (kp=10). An unbounded per-cycle jump to the raw IK
        # solution makes feedback lag by tens of degrees and immediately trips the
        # tracking monitor, which used to stop the arm and demand a re-clutch.
        # Always slew-limit the joint command; home-return keeps its own cap.
        slew_deg = max_joint_step_deg if servo_slew_deg is None else servo_slew_deg
        self.follower = JointFollower(limits[:, 0], limits[:, 1],
                                      np.deg2rad(slew_deg) if slew_deg > 0 else np.inf,
                                      max_error_rad=np.deg2rad(tracking_error_deg),
                                      tracking_timeout=tracking_timeout)
        self.rearm_required = False
        self.recovering = False
        self.arm = None
        self.last_command = None
        self.last_gripper = None
        self.last_gripper_time = 0.0
        self.gripper_command_m = np.nan
        self.gripper_force = gripper_force
        self.gripper_slew_m_per_s = max(0.0, float(gripper_slew_mm_s)) / 1000.0
        self.gripper_command_norm = None
        self.gripper_slew_time = 0.0
        self.gripper_armed = False  # Require a released trigger after initial opening.
        self.last_trigger = 0.0
        self.last_trigger_at_send = 0.0
        self.follow_metrics = {}

    def solve(self, measured):
        started = time.monotonic()
        previous = self.last_command
        active = self.xr.get_key_value_by_name(f"{self.side}_grip") > 0.9
        target = self.controller.solve_from_feedback(measured, active)
        command = self.follower.next(measured, target, active)
        if active:
            self.controller.pose_ik.previous = command.copy()
            # Compare feedback with the PREVIOUS sent command: the current
            # command cannot have reached hardware when this sample was taken.
            self.follow_metrics = {
                "input_mm": self.controller.input_position_gap_m * 1000,
                "limit_deg": float(np.rad2deg(np.max(np.abs(target - command)))),
                "drive_deg": float(np.rad2deg(np.max(np.abs(measured - previous)))) if previous is not None else 0.0,
                "track_deg": float(np.rad2deg(self.follower.tracking_error(measured))),
                "ik_ms": (time.monotonic() - started) * 1000,
                "ik_residual_mm": self.controller.last_ik_result.position_error_m * 1000,
                "ik_residual_deg": np.rad2deg(self.controller.last_ik_result.rotation_error_rad),
                "ik_reached": self.controller.last_ik_result.reached,
            }
        return command, active

    def release_clutch(self):
        name = f"{self.side}_hand"
        self.controller.ref_ee_xyz[name] = None
        self.controller.ref_controller_xyz[name] = None
        self.follower.last_command = None
        self.follower.error_started = None
        self.controller.pose_ik.previous = None

    def send(self, command, tick, update_gripper=True, home_return=False):
        if command is not None:
            if home_return:
                # Planned joint mode is used for home, including the small
                # boundary correction in the saved physical pose.
                self.arm.move_joints(command, motion_mode="j")
            else:
                self.arm.move_joints(command)
            self.last_command = command.copy()
        if not update_gripper:
            return
        trigger = float(np.clip(self.xr.get_key_value_by_name(f"{self.side}_trigger"), 0, 1))
        self.last_trigger = trigger
        if trigger <= 0.05:
            self.gripper_armed = True
        if not self.gripper_armed or trigger <= 0.05:
            opening = 1.0
        elif trigger >= 0.95:
            opening = 0.0
        else:
            opening = 1.0 - (trigger - 0.05) / 0.9
        # Like UR, trigger is independent of the arm clutch. Periodic refresh also
        # recovers a lost CAN target, rather than sending each width only once.
        # Slew-limit the commanded stroke. The gripper takes a position and a
        # force, with no speed field, so a step command lets the jaws travel to
        # the end stop at full speed - the release slam. Ramp the position
        # instead; the effector follows the intermediate targets.
        span = float(self.arm.gripper_max_width) or 1.0
        if self.gripper_command_norm is None:
            self.gripper_command_norm = opening
            self.gripper_slew_time = tick
        max_step = self.gripper_slew_m_per_s * max(tick - self.gripper_slew_time, 1e-3) / span
        delta = float(np.clip(opening - self.gripper_command_norm, -max_step, max_step))
        self.gripper_command_norm = float(np.clip(self.gripper_command_norm + delta, 0.0, 1.0))
        self.gripper_slew_time = tick
        commanded = self.gripper_command_norm
        due = tick - self.last_gripper_time
        if due >= 0.02 and (self.last_gripper is None or abs(commanded - self.last_gripper) > 0.004
                            or due >= 0.5):
            self.arm.set_gripper(commanded, force=self.gripper_force)
            self.gripper_command_m = commanded * span
            self.last_gripper, self.last_gripper_time = commanded, tick
            # Remember which trigger sample produced this width; the live trigger
            # keeps moving, so reporting it next to a rate-limited width looks
            # like a broken mapping.
            self.last_trigger_at_send = trigger

    def hold_or_recover(self, measured):
        """Freeze the arm during a tracking hold and resume once it catches up.

        Nothing is sent while held, so the MIT servo keeps holding its last
        target. Re-anchoring on resume avoids replaying the stale target that
        caused the lag in the first place (no button release needed).
        """
        if not self.recovering:
            return False
        if self.follower.error_within_tolerance(measured):
            self.follower.reanchor(measured)
            self.controller.pose_ik.previous = None
            self.recovering = False
            print(f"[{self.side}] tracking recovered "
                  f"(error {np.rad2deg(self.follower.tracking_error(measured)):.1f} deg); resuming.",
                  flush=True)
            return False
        return True

    def recording_state(self, measured, active, dual):
        from piper_teleop_recorder.teleop_record import flange_rpy_to_eef_rpy

        fp = self.arm.driver.get_flange_pose()
        gs = self.arm.gripper.get_gripper_status()
        flange = (np.asarray(fp.msg, dtype=np.float32) if fp is not None
                  and time.time() - fp.timestamp <= 0.3 else np.full(6, np.nan))
        gripper = (float(gs.msg.value) if gs is not None
                   and time.time() - gs.timestamp <= 0.3 else np.nan)
        prefix = f"{self.side}_" if dual else ""
        state = {
            prefix + "joint_rad": measured,
            prefix + "action_joint_rad": self.last_command,
            prefix + "tcp_flange": flange,
            prefix + "tcp_eef": flange_rpy_to_eef_rpy(flange, TCP_OFFSET_M),
            prefix + "gripper_m": gripper,
            prefix + "action_gripper_m": gripper if not np.isfinite(self.gripper_command_m) else self.gripper_command_m,
            prefix + "teleop_active": active,
            f"{self.side}_trigger": self.xr.get_key_value_by_name(f"{self.side}_trigger"),
            f"{self.side}_grip": self.xr.get_key_value_by_name(f"{self.side}_grip"),
        }
        return state

    def report_gripper(self):
        status = self.arm.gripper.get_gripper_status()
        target = self.gripper_command_m * 1000
        if status is None or time.time() - status.timestamp > 0.3:
            print(f"[{self.side}] gripper target={target:.1f} mm; WARNING: no fresh gripper feedback", flush=True)
            return
        msg = status.msg
        actual = f"{msg.value * 1000:.1f} mm" if msg.mode == "width" else f"{msg.value:.1f} deg"
        print(f"[{self.side}] trigger={self.last_trigger:.2f} (width sent at trigger="
              f"{self.last_trigger_at_send:.2f}); gripper target={target:.1f} mm, actual={actual}, "
              f"enabled={msg.foc_status.driver_enable_status}, homed={msg.foc_status.homing_status}, "
              f"status=0x{msg.status_code:02x}", flush=True)


def main(
    channel: str | None = None,
    enable_hardware: bool = False,
    firmware: str | None = None,
    rate_hz: float = 100.0,
    max_joint_step_deg: float = 1.2,
    scale_factor: float = 1.0,
    speed_percent: int = 100,
    gripper_max_width: float = 0.10,
    gripper_force: float = 5.0,
    motion_mode: Literal["j", "js", "mit"] = "js",
    mit_kp: float = 10.0,
    mit_kd: float = 0.8,
    no_pose_filter: bool = False,
    position_deadband: float = 0.001,
    rotation_deadband_deg: float = 0.3,
    max_linear_speed: float = 0.5,
    max_angular_speed: float = 1.5,
    oneeuro_min_cutoff: float = 3.0,
    oneeuro_beta: float = 40.0,
    oneeuro_derivative_cutoff: float = 10.0,
    oneeuro_rotation_min_cutoff: float = 3.0,
    oneeuro_rotation_beta: float = 3.0,
    read_only: bool = False,
    record_config: str | None = None,
    mode: Literal["single", "dual"] = "single",
    controller_side: Literal["left", "right"] = "right",
    left_channel: str = "can0",
    right_channel: str = "can1",
    left_firmware: str | None = None,
    right_firmware: str | None = None,
    pico_wait_timeout: float = 0.0,
    home_config: str = str(Path(__file__).resolve().parents[3] / "configs/piper_x_home.json"),
    home_speed_deg_s: float = 15.0,
    home_timeout: float = 30.0,
    unlimited_motion: bool = True,
    servo_slew_deg: float = 2.0,
    tracking_error_deg: float = 20.0,
    tracking_timeout: float = 0.8,
    strict_rearm: bool = False,
    skip_gripper_check: bool = False,
    allow_gripper_fault: bool = False,
    gripper_slew_mm_s: float = 250.0,
) -> None:
    """Independent GRIP/TRIGGER; A records; B returns right, Y returns left; Ctrl+C exits."""
    selected = select_arms(mode, controller_side, channel, left_channel, right_channel)
    profiles = {side: (left_firmware if side == "left" else right_firmware) or firmware
                for side in selected}
    print(f"Piper-X mode={mode}, hand/CAN mapping={selected}")
    if read_only:
        from contextlib import ExitStack
        with ExitStack() as cleanup:
            arms = {}
            for side, can_channel in selected.items():
                arm = PiperXCanArm(channel=can_channel, firmware=profiles[side], read_only=True)
                cleanup.callback(arm.close)
                arms[side] = arm
            for _ in range(10):
                for side, arm in arms.items():
                    print(f"{side} ({selected[side]}) measured joints (deg):", np.rad2deg(arm.read_joints()).round(2))
                time.sleep(0.1)
        return
    if not enable_hardware:
        print("Preflight only: motors NOT enabled. Check CAN at 1 Mbit/s, e-stop and clear workspace.")
        print("Run with --enable-hardware only after the MuJoCo test and read-only feedback check.")
        return
    if not 0 < rate_hz <= 100 or not 0 < max_joint_step_deg <= 2:
        raise ValueError("Use 0 < rate_hz <= 100 and 0 < max_joint_step_deg <= 2")
    if not 0 < scale_factor <= 1 or not 1 <= speed_percent <= 100 or not 0 < gripper_max_width <= 0.1:
        raise ValueError("Invalid motion scale, speed percentage or gripper width")
    if not 0 < gripper_force <= 10 or position_deadband < 0 or rotation_deadband_deg < 0:
        raise ValueError("Invalid gripper force (0..10 N) or pose deadband")
    if not np.isfinite(mit_kp) or not np.isfinite(mit_kd) or not 0 < mit_kp <= 500 or not 0 < mit_kd <= 5:
        raise ValueError("MIT requires 0 < kp <= 500 and 0 < kd <= 5")
    if max_linear_speed <= 0 or max_angular_speed <= 0:
        raise ValueError("Cartesian speed limits must be positive")
    if oneeuro_min_cutoff <= 0 or oneeuro_beta < 0:
        raise ValueError("Invalid One-Euro filter parameters")
    if oneeuro_derivative_cutoff <= 0 or oneeuro_rotation_min_cutoff <= 0 or oneeuro_rotation_beta < 0:
        raise ValueError("Invalid One-Euro derivative/rotation parameters")
    if not 0 < home_speed_deg_s <= 30 or not 0 < home_timeout <= 120:
        raise ValueError("Use 0 < home-speed-deg-s <= 30 and 0 < home-timeout <= 120")
    if not 0 < servo_slew_deg <= 15:
        raise ValueError("Use 0 < servo-slew-deg <= 15 (per cycle joint command slew)")
    if not 5 <= tracking_error_deg <= 60 or not 0.1 <= tracking_timeout <= 5:
        raise ValueError("Use 5 <= tracking-error-deg <= 60 and 0.1 <= tracking-timeout <= 5")
    if not 0 < gripper_slew_mm_s <= 1000:
        raise ValueError("Use 0 < gripper-slew-mm-s <= 1000; 0 would disable the stroke ramp")
    home_targets = load_home_targets(home_config, selected)

    from xrobotoolkit_teleop.common.xr_client import XrClient

    urdf, scene = build_scene()
    xr = XrClient()
    runtimes = {}
    recording = None
    try:
        for side, can_channel in selected.items():
            runtimes[side] = TeleopArm(
                side, can_channel, xr, urdf, scene, scale_factor, max_joint_step_deg,
                rate_hz=rate_hz, gripper_force=gripper_force, no_pose_filter=no_pose_filter,
                position_deadband=position_deadband, rotation_deadband_deg=rotation_deadband_deg,
                max_linear_speed=max_linear_speed, max_angular_speed=max_angular_speed,
                oneeuro_min_cutoff=oneeuro_min_cutoff, oneeuro_beta=oneeuro_beta,
                unlimited_motion=unlimited_motion,
                oneeuro_derivative_cutoff=oneeuro_derivative_cutoff,
                oneeuro_rotation_min_cutoff=oneeuro_rotation_min_cutoff,
                oneeuro_rotation_beta=oneeuro_rotation_beta,
                servo_slew_deg=(max_joint_step_deg if not unlimited_motion else servo_slew_deg),
                tracking_error_deg=tracking_error_deg,
                tracking_timeout=tracking_timeout,
                gripper_slew_mm_s=gripper_slew_mm_s,
            )
        homes = {}
        for side, runtime in runtimes.items():
            homes[side] = JointHomeReturn(
                side, home_targets[side], runtime.follower.lower, runtime.follower.upper,
                min(np.deg2rad(home_speed_deg_s) / rate_hz, np.deg2rad(max_joint_step_deg)),
                timeout=home_timeout,
            )
            runtime.follower.feedback_limit_margin_rad = homes[side].feedback_margin
        if record_config:
            sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
            from piper_teleop_recorder.session import PiperRecordingSession

            recording = PiperRecordingSession(record_config, xr, TCP_OFFSET_M,
                                              arm_channels=selected, motion_settings={
                                                  "gripper_force_n": gripper_force,
                                                  "pose_filter": not no_pose_filter,
                                                  "position_deadband_m": position_deadband,
                                                  "rotation_deadband_deg": rotation_deadband_deg,
                                                  "max_linear_speed_m_s": max_linear_speed,
                                                  "max_angular_speed_rad_s": max_angular_speed,
                                                  "motion_mode": motion_mode,
                                                  "ik_backend": "continuous_bounded_pose",
                                                  "mit_kp": mit_kp,
                                                  "mit_kd": mit_kd,
                                                  "unlimited_motion": unlimited_motion,
                                                  "oneeuro_min_cutoff": oneeuro_min_cutoff,
                                                  "oneeuro_beta": oneeuro_beta,
                                                  "oneeuro_derivative_cutoff": oneeuro_derivative_cutoff,
                                                  "oneeuro_rotation_min_cutoff": oneeuro_rotation_min_cutoff,
                                                  "oneeuro_rotation_beta": oneeuro_rotation_beta,
                                                  "home_config": str(Path(home_config).resolve()),
                                                  "home_joint_rad": {side: home.target.tolist() for side, home in homes.items()},
                                                  "home_speed_deg_s": home_speed_deg_s,
                                                  "home_motion_mode": "j",
                                                  "servo_slew_deg": max_joint_step_deg if not unlimited_motion else servo_slew_deg,
                                                  "tracking_error_deg": tracking_error_deg,
                                                  "tracking_timeout_s": tracking_timeout,
                                                  "gripper_slew_mm_s": gripper_slew_mm_s,
                                              })
            recording.cfg.control.rate_hz = rate_hz
            recording.cfg.control.max_joint_step_deg = max_joint_step_deg
            recording.cfg.control.scale_factor = scale_factor
            recording.cfg.control.no_pose_filter = no_pose_filter
            recording.cfg.control.position_deadband = position_deadband
            recording.cfg.control.rotation_deadband_deg = rotation_deadband_deg
            recording.cfg.control.max_linear_speed = max_linear_speed
            recording.cfg.control.max_angular_speed = max_angular_speed
            recording.cfg.control.oneeuro_min_cutoff = oneeuro_min_cutoff
            recording.cfg.control.oneeuro_beta = oneeuro_beta
            recording.cfg.control.oneeuro_derivative_cutoff = oneeuro_derivative_cutoff
            recording.cfg.control.oneeuro_rotation_min_cutoff = oneeuro_rotation_min_cutoff
            recording.cfg.control.oneeuro_rotation_beta = oneeuro_rotation_beta
            recording.cfg.can.motion_mode = motion_mode
            recording.cfg.can.mit_kp = mit_kp
            recording.cfg.can.mit_kd = mit_kd
            recording.cfg.can.channel = next(iter(selected.values()))
            recording.cfg.can.firmware = firmware
            recording.cfg.can.speed_percent = speed_percent
            recording.cfg.can.gripper_max_width = gripper_max_width
            recording.cfg.can.gripper_force = gripper_force
            recording.cfg.arm_firmware = profiles
            recording.prepare()

        wait_for_pico(xr, selected, pico_wait_timeout)
        # Check both arms' read-only feedback before enabling either one.
        for side, can_channel in selected.items():
            probe = PiperXCanArm(channel=can_channel, firmware=profiles[side], read_only=True)
            try:
                probe.read_joints()
            finally:
                probe.close()
        for side, runtime in runtimes.items():
            runtime.arm = PiperXCanArm(
                channel=runtime.channel, firmware=profiles[side], speed_percent=speed_percent,
                gripper_max_width=gripper_max_width,
                motion_mode=motion_mode,
                mit_kp=mit_kp, mit_kd=mit_kd,
            )
            runtime.last_command = runtime.arm.read_joints()

        for runtime in runtimes.values():
            try:
                runtime.arm.open_gripper(force=gripper_force)
            except RuntimeError as exc:
                # A non-responding effector must not block arm teleoperation:
                # the arm fix has to be verifiable while the gripper is broken.
                if not allow_gripper_fault:
                    raise
                print(f"[{runtime.side}] WARNING: {exc} -- continuing with "
                      f"--allow-gripper-fault; TRIGGER will have no effect.", flush=True)
            runtime.last_gripper = 1.0
            runtime.gripper_command_norm = 1.0
            runtime.gripper_slew_time = time.monotonic()
            runtime.gripper_command_m = runtime.arm.gripper_max_width
            runtime.last_gripper_time = time.monotonic()
        # `open_gripper` only proves the width READS as the full stroke, which is
        # also true for an effector whose feedback is frozen (not homed). Probe a
        # small close so a dead gripper cannot silently make TRIGGER look unmapped.
        if not skip_gripper_check:
            for runtime in runtimes.values():
                moved, before, after = runtime.arm.verify_gripper_stroke()
                if moved:
                    print(f"[{runtime.side}] gripper stroke check OK: "
                          f"{before * 1000:.1f} -> {after * 1000:.1f} mm", flush=True)
                else:
                    shown = "no feedback" if after is None else f"{after * 1000:.1f} mm"
                    print(f"[{runtime.side}] GRIPPER NOT RESPONDING: commanded a small close but the "
                          f"width stayed at {before * 1000:.1f} -> {shown}. TRIGGER will look "
                          f"unmapped until this is fixed. Run "
                          f"'PYTHONPATH=. .venv/bin/python scripts/hardware/piper_x_gripper_tool.py "
                          f"--channel {runtime.channel} read' for the gripper state.",
                          flush=True)
        # Opening may take seconds. Recheck live XR after it before any joint command.
        wait_for_pico(xr, selected, pico_wait_timeout, motors_enabled=True)

        timestamp, packet_time = xr.get_timestamp_ns(), time.monotonic()
        print("Piper-X enabled: GRIP takes over, TRIGGER controls gripper; B returns right, Y returns left; Ctrl+C exits.")
        joint_slew = max_joint_step_deg if not unlimited_motion else servo_slew_deg
        joint_cap = (f'{rate_hz * joint_slew:g} deg/s ({joint_slew:g} deg/cycle)'
                     if joint_slew > 0 else 'off')
        cartesian_cap = 'off' if unlimited_motion else f'{max_linear_speed:g} m/s'
        print(f"Motion: {rate_hz:g} Hz, scale={scale_factor:g}, joint slew cap={joint_cap}, "
              f"driver speed={speed_percent}%; One-Euro={'off' if no_pose_filter else 'on'}, "
              f"Cartesian cap={cartesian_cap}; mode={motion_mode}; "
              f"gripper={gripper_force:g} N, stroke ramp={gripper_slew_mm_s:g} mm/s "
              f"(full stroke {gripper_max_width * 1000 / gripper_slew_mm_s:.2f} s); "
              f"release TRIGGER once to arm closing.")
        print(f"One-Euro: position cutoff={oneeuro_min_cutoff:g}, beta={oneeuro_beta:g}; "
              f"rotation cutoff={oneeuro_rotation_min_cutoff:g}, beta={oneeuro_rotation_beta:g}; "
              f"derivative cutoff={oneeuro_derivative_cutoff:g}; tracking grace={tracking_timeout:g} s.")
        if motion_mode == "mit":
            print("NOTE: motion mode 'mit' measured ~11 deg of joint tracking lag on "
                  "firmware S-V1.9-0 (a 20 deg, 10 deg/s ramp); 'js' measured ~1.2 deg and "
                  "'j' ~0.4 deg. Expect soft, trailing motion until the mode changes.")
        print(f"Tracking guard: fault above {tracking_error_deg:g} deg for {tracking_timeout:g} s; "
              + ("hold and auto-resume (release GRIP to re-clutch manually)."
                 if not strict_rearm else "stop and require a fresh GRIP re-clutch."))
        print("IK: continuous bounded pose solve, previous-command seed, FK residual checks (no Placo step).")
        if motion_mode == "mit":
            print(f"Servo: SDK move_mit on J1-J6; kp={mit_kp:g}, kd={mit_kd:g}, velocity/torque feedforward=0.")
        period = 1.0 / rate_hz
        next_gripper_report = time.monotonic() + 2.0
        report_started, report_cycles = time.monotonic(), 0
        max_packet_gap = 0.0
        while True:
            tick = time.monotonic()
            current_timestamp = xr.get_timestamp_ns()
            max_packet_gap = max(max_packet_gap, tick - packet_time)
            if current_timestamp != timestamp and current_timestamp > 0:
                timestamp, packet_time = current_timestamp, tick
            if tick - packet_time > 0.5:
                pause_for_pico(xr, runtimes, homes, recording)
                timestamp, packet_time = xr.get_timestamp_ns(), time.monotonic()
                report_started, report_cycles = packet_time, 0
                next_gripper_report, max_packet_gap = packet_time + 2.0, 0.0
                continue
            # Verify every selected arm/hand before sending either arm's target.
            try:
                for side in selected:
                    validate_controller(xr, side)
            except RuntimeError:
                pause_for_pico(xr, runtimes, homes, recording)
                timestamp, packet_time = xr.get_timestamp_ns(), time.monotonic()
                report_started, report_cycles = packet_time, 0
                next_gripper_report, max_packet_gap = packet_time + 2.0, 0.0
                continue
            measured = {side: runtime.arm.read_joints() for side, runtime in runtimes.items()}
            solutions, home_handled = {}, {}
            try:
                for side, runtime in runtimes.items():
                    if getattr(runtime, "rearm_required", False):
                        # GRIP is the clutch; TRIGGER only drives the gripper, so
                        # holding it must not block re-arming the arm.
                        if xr.get_key_value_by_name(f"{side}_grip") > 0.9:
                            home_handled[side] = False
                            solutions[side] = (None, False)
                            continue
                        runtime.rearm_required = False
                    hold_or_recover = getattr(runtime, "hold_or_recover", None)
                    if hold_or_recover is not None and hold_or_recover(measured[side]):
                        # Wait for the arm to catch up; keep the gripper live.
                        home_handled[side] = False
                        solutions[side] = (None, False)
                        continue
                    command, handled = homes[side].step(
                        measured[side], xr.get_button_state_by_name("B" if side == "right" else "Y"),
                        xr.get_key_value_by_name(f"{side}_grip") > 0.9, tick,
                    )
                    home_handled[side] = handled
                    if handled:
                        runtime.release_clutch()
                        solutions[side] = (command, False)
                    else:
                        try:
                            solutions[side] = runtime.solve(measured[side])
                        except JointTrackingError as exc:
                            if strict_rearm:
                                raise
                            runtime.recovering = True
                            print(f"[tracking hold] {exc}; holding the last target and "
                                  f"auto-resuming when the arm catches up "
                                  f"(no button release needed).", flush=True)
                            solutions[side] = (None, False)
            except JointTrackingError as exc:
                print(f"[tracking hold] {exc}; release GRIP, then re-clutch to retry.", flush=True)
                hold_for_rearm(runtimes, homes, recording)
                continue
            for side, runtime in runtimes.items():
                runtime.send(solutions[side][0], tick, update_gripper=not home_handled[side],
                             home_return=home_handled[side])
            report_cycles += 1
            if tick >= next_gripper_report:
                actual_hz = report_cycles / max(tick - report_started, 1e-6)
                print(f"[loop] actual={actual_hz:.1f} Hz / requested={rate_hz:g} Hz; "
                      f"PICO max update gap={max_packet_gap * 1000:.0f} ms (not end-to-end latency)", flush=True)
                max_packet_gap = 0.0
                for side, runtime in runtimes.items():
                    runtime.report_gripper()
                    if solutions[side][1]:
                        target = runtime.controller.effector_task[f"{side}_hand"].T_world_frame[:3, 3]
                        actual = runtime.controller.mj_data.body(TCP_LINK).xpos
                        error_mm = np.linalg.norm(target - actual) * 1000
                        m = runtime.follow_metrics
                        print(f"[{side}] follow: TCP={error_mm:.1f} mm; "
                              f"input(filter/rate)={m['input_mm']:.1f} mm; "
                              f"joint-limit={m['limit_deg']:.2f} deg; "
                              f"drive(previous target)={m['drive_deg']:.2f} deg; "
                              f"tracking={m['track_deg']:.2f} deg; IK={m['ik_ms']:.1f} ms; "
                              f"IK residual={m['ik_residual_mm']:.1f} mm/{m['ik_residual_deg']:.1f} deg; "
                              f"target={'reached' if m['ik_reached'] else 'not reached (bounds/pose/iteration budget)'}", flush=True)
                report_started, report_cycles = tick, 0
                next_gripper_report = tick + 2.0
            if recording:
                state = {}
                for side, runtime in runtimes.items():
                    state.update(runtime.recording_state(measured[side], solutions[side][1], mode == "dual"))
                    state[(side + "_" if mode == "dual" else "") + "returning_home"] = homes[side].returning
                recording.update(state)
                if recording.controller._thread is None:
                    recording.start()
            time.sleep(max(0, period - (time.monotonic() - tick)))
    finally:
        try:
            if recording is not None:
                recording.close()
        finally:
            try:
                for runtime in runtimes.values():
                    if runtime.arm is not None:
                        try:
                            runtime.arm.close()
                        except Exception as exc:
                            print(f"{runtime.side} CAN disconnect failed: {exc}")
            finally:
                xr.close()


if __name__ == "__main__":
    tyro.cli(main)
