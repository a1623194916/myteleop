"""PICO single/dual Piper-X teleoperation over independent SocketCAN channels."""

import sys
import time
from pathlib import Path
from typing import Literal

import mujoco
import numpy as np
import tyro

from xrobotoolkit_teleop.hardware.piper_x import JointFollower, PiperXCanArm
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


def wait_for_pico(xr, sides, timeout=0.0):
    """Wait for a new packet with all selected hands valid, before opening CAN."""
    started = time.monotonic()
    timestamp = xr.get_timestamp_ns()
    next_status = started + 10.0
    print(f"Waiting for PICO ({', '.join(sides)}); connect headset to PC Service at any time. "
          "Motors are not enabled by this process; Ctrl+C cancels.", flush=True)
    while True:
        now = time.monotonic()
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


class TeleopArm:
    """One arm's IK, clutch and gripper state; XR is shared across both arms."""

    def __init__(self, side, channel, xr, urdf, scene, scale_factor, max_joint_step_deg,
                 rate_hz=50.0, gripper_force=5.0, no_pose_filter=False,
                 position_deadband=0.002, rotation_deadband_deg=0.5,
                 max_linear_speed=0.25, max_angular_speed=1.5):
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
        )
        self.model = self.controller.mj_model
        self.joint_ids = [self.model.joint(name).id for name in JOINTS]
        limits = self.model.jnt_range[self.joint_ids]
        self.follower = JointFollower(limits[:, 0], limits[:, 1], np.deg2rad(max_joint_step_deg))
        self.arm = None
        self.last_command = None
        self.last_gripper = None
        self.last_gripper_time = 0.0
        self.gripper_command_m = np.nan
        self.gripper_force = gripper_force

    def solve(self, measured):
        for index, joint_id in enumerate(self.joint_ids):
            self.controller.mj_data.qpos[self.model.jnt_qposadr[joint_id]] = measured[index]
        mujoco.mj_forward(self.model, self.controller.mj_data)
        self.controller._update_ik()
        self.controller._send_command()
        target = self.controller.mj_data.ctrl[:6].copy()
        active = self.xr.get_key_value_by_name(f"{self.side}_grip") > 0.9
        return self.follower.next(measured, target, active), active

    def send(self, command, tick):
        if command is not None:
            self.arm.move_joints(command)
            self.last_command = command.copy()
        opening = 1.0 - float(np.clip(self.xr.get_key_value_by_name(f"{self.side}_trigger"), 0, 1))
        # Like UR, trigger is independent of the arm clutch. Periodic refresh also
        # recovers a lost CAN target, rather than sending each width only once.
        due = tick - self.last_gripper_time
        if due >= 0.1 and (self.last_gripper is None or abs(opening - self.last_gripper) > 0.03 or due >= 0.5):
            self.arm.set_gripper(opening, force=self.gripper_force)
            self.gripper_command_m = opening * self.arm.gripper_max_width
            self.last_gripper, self.last_gripper_time = opening, tick

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
        print(f"[{self.side}] trigger={self.xr.get_key_value_by_name(self.side + '_trigger'):.2f}; "
              f"gripper target={target:.1f} mm, actual={actual}, "
              f"enabled={msg.foc_status.driver_enable_status}, status=0x{msg.status_code:02x}", flush=True)


def main(
    channel: str | None = None,
    enable_hardware: bool = False,
    firmware: str | None = None,
    rate_hz: float = 50.0,
    max_joint_step_deg: float = 0.6,
    scale_factor: float = 1.0,
    speed_percent: int = 30,
    gripper_max_width: float = 0.07,
    gripper_force: float = 5.0,
    no_pose_filter: bool = False,
    position_deadband: float = 0.002,
    rotation_deadband_deg: float = 0.5,
    max_linear_speed: float = 0.25,
    max_angular_speed: float = 1.5,
    read_only: bool = False,
    record_config: str | None = None,
    mode: Literal["single", "dual"] = "single",
    controller_side: Literal["left", "right"] = "right",
    left_channel: str = "can0",
    right_channel: str = "can1",
    left_firmware: str | None = None,
    right_firmware: str | None = None,
    pico_wait_timeout: float = 0.0,
) -> None:
    """Independent GRIP/TRIGGER per arm; A records both; B stops the whole session."""
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
    if not 0 < rate_hz <= 50 or not 0 < max_joint_step_deg <= 1:
        raise ValueError("Use 0 < rate_hz <= 50 and 0 < max_joint_step_deg <= 1")
    if not 0 < scale_factor <= 1 or not 1 <= speed_percent <= 100 or not 0 < gripper_max_width <= 0.1:
        raise ValueError("Invalid motion scale, speed percentage or gripper width")
    if not 0 < gripper_force <= 10 or position_deadband < 0 or rotation_deadband_deg < 0:
        raise ValueError("Invalid gripper force (0..10 N) or pose deadband")
    if max_linear_speed <= 0 or max_angular_speed <= 0:
        raise ValueError("Cartesian speed limits must be positive")

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
            )
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
                                              })
            recording.cfg.control.rate_hz = rate_hz
            recording.cfg.control.max_joint_step_deg = max_joint_step_deg
            recording.cfg.control.scale_factor = scale_factor
            recording.cfg.control.no_pose_filter = no_pose_filter
            recording.cfg.control.position_deadband = position_deadband
            recording.cfg.control.rotation_deadband_deg = rotation_deadband_deg
            recording.cfg.control.max_linear_speed = max_linear_speed
            recording.cfg.control.max_angular_speed = max_angular_speed
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
            )
            runtime.last_command = runtime.arm.read_joints()

        timestamp, packet_time = xr.get_timestamp_ns(), time.monotonic()
        print("Piper-X enabled. Each hand's GRIP engages its arm; TRIGGER controls its gripper; B stops all.")
        print(f"Motion: {rate_hz:g} Hz, scale={scale_factor:g}, joint cap={rate_hz * max_joint_step_deg:g} deg/s, "
              f"driver speed={speed_percent}%; One-Euro={'off' if no_pose_filter else 'on'}, "
              f"Cartesian cap={max_linear_speed:g} m/s; gripper={gripper_force:g} N (independent of GRIP).")
        period = 1.0 / rate_hz
        next_gripper_report = time.monotonic() + 2.0
        while True:
            tick = time.monotonic()
            current_timestamp = xr.get_timestamp_ns()
            if current_timestamp != timestamp and current_timestamp > 0:
                timestamp, packet_time = current_timestamp, tick
            if tick - packet_time > 0.5:
                raise RuntimeError("PICO packets stopped updating (> 500 ms); stopping all arms")
            if xr.get_button_state_by_name("B"):
                print("B pressed: stopping all arms")
                break
            # Verify every selected arm/hand before sending either arm's target.
            for side in selected:
                validate_controller(xr, side)
            measured = {side: runtime.arm.read_joints() for side, runtime in runtimes.items()}
            solutions = {side: runtime.solve(measured[side]) for side, runtime in runtimes.items()}
            for side, runtime in runtimes.items():
                runtime.send(solutions[side][0], tick)
            if tick >= next_gripper_report:
                for runtime in runtimes.values():
                    runtime.report_gripper()
                next_gripper_report = tick + 2.0
            if recording:
                state = {}
                for side, runtime in runtimes.items():
                    state.update(runtime.recording_state(measured[side], solutions[side][1], mode == "dual"))
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
