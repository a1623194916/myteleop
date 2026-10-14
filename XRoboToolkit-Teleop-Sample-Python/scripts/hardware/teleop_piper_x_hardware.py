"""PICO teleoperation of a real Piper-X over SocketCAN (opt-in motor enable)."""

import time
import sys
from pathlib import Path

import mujoco
import numpy as np
import tyro

from xrobotoolkit_teleop.hardware.piper_x import JointFollower, PiperXCanArm
from xrobotoolkit_teleop.simulation.piper_x_model import JOINTS, TCP_LINK, build_scene


def main(
    channel: str = "can0",
    enable_hardware: bool = False,
    firmware: str | None = None,
    rate_hz: float = 20.0,
    max_joint_step_deg: float = 0.35,
    scale_factor: float = 0.5,
    speed_percent: int = 10,
    gripper_max_width: float = 0.07,
    read_only: bool = False,
    record_config: str | None = None,
) -> None:
    """Hold right GRIP to follow the controller; press B to stop the session."""
    if read_only:
        arm = PiperXCanArm(channel=channel, firmware=firmware, read_only=True)
        try:
            for _ in range(10):
                print("Piper-X measured joints (deg):", np.rad2deg(arm.read_joints()).round(2))
                time.sleep(0.1)
        finally:
            arm.close()
        return
    if not enable_hardware:
        print("Preflight only: motors NOT enabled. Check CAN at 1 Mbit/s, e-stop and clear workspace.")
        print("Run with --enable-hardware only after the MuJoCo test and read-only feedback check.")
        return
    if not 0 < rate_hz <= 50 or not 0 < max_joint_step_deg <= 1:
        raise ValueError("Use 0 < rate_hz <= 50 and 0 < max_joint_step_deg <= 1")
    if not 0 < scale_factor <= 1 or not 1 <= speed_percent <= 100 or not 0 < gripper_max_width <= 0.1:
        raise ValueError("Invalid motion scale, speed percentage or gripper width")

    from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController
    from xrobotoolkit_teleop.simulation.piper_x_model import TCP_OFFSET_M

    urdf, scene = build_scene()
    # XR/IK setup happens before CAN is opened. A failed model or headset
    # connection must not leave a powered arm in a half-started session.
    controller = MujocoTeleopController(
        xml_path=str(scene), robot_urdf_path=str(urdf),
        manipulator_config={"right_hand": {
            "link_name": TCP_LINK, "pose_source": "right_controller",
            "control_trigger": "right_grip", "vis_target": "target",
        }},
        scale_factor=scale_factor,
    )
    model = controller.mj_model
    joint_ids = [model.joint(name).id for name in JOINTS]
    limits = model.jnt_range[joint_ids]
    follower = JointFollower(limits[:, 0], limits[:, 1], np.deg2rad(max_joint_step_deg))
    xr = controller.xr_client
    arm = None
    recording = None
    last_gripper = None
    last_gripper_time = 0.0
    period = 1.0 / rate_hz
    try:
        if record_config:
            sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
            from piper_teleop_recorder.session import PiperRecordingSession
            from piper_teleop_recorder.teleop_record import flange_rpy_to_eef_rpy
            recording = PiperRecordingSession(record_config, xr, TCP_OFFSET_M)
            recording.cfg.control.rate_hz = rate_hz
            recording.cfg.control.max_joint_step_deg = max_joint_step_deg
            recording.cfg.control.scale_factor = scale_factor
            recording.cfg.can.channel = channel
            recording.cfg.can.firmware = firmware
            recording.cfg.can.speed_percent = speed_percent
            recording.cfg.can.gripper_max_width = gripper_max_width
            recording.prepare()  # Camera ready before enabling robot motors.
        # Require live headset packets before enabling any motor. The headset
        # clock has its own origin, so freshness is measured by packet changes.
        timestamp = xr.get_timestamp_ns()
        deadline = time.monotonic() + 5
        while xr.get_timestamp_ns() == timestamp or xr.get_timestamp_ns() <= 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("No live PICO packets; connect headset to PC Service first")
            time.sleep(0.02)
        timestamp = xr.get_timestamp_ns()
        packet_time = time.monotonic()
        pose = np.asarray(xr.get_pose_by_name("right_controller"), dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:]) < 0.5:
            raise RuntimeError("Right controller pose is unavailable; motors will not enable")
        arm = PiperXCanArm(
            channel=channel, firmware=firmware, speed_percent=speed_percent,
            gripper_max_width=gripper_max_width,
        )
        last_command = arm.read_joints()
        gripper_command_m = np.nan
        print("Piper-X enabled. Hold right GRIP to move, right TRIGGER closes gripper; B stops.")
        while True:
            tick = time.monotonic()
            current_timestamp = xr.get_timestamp_ns()
            if current_timestamp != timestamp and current_timestamp > 0:
                timestamp, packet_time = current_timestamp, tick
            if tick - packet_time > 0.5:
                raise RuntimeError("PICO packets stopped updating (> 500 ms); stopping teleoperation")
            pose = np.asarray(xr.get_pose_by_name("right_controller"), dtype=float)
            if pose.shape != (7,) or not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:]) < 0.5:
                raise RuntimeError("Right controller pose lost; stopping teleoperation")
            if xr.get_button_state_by_name("B"):
                print("B pressed: stopping teleoperation")
                break
            measured = arm.read_joints()
            for index, joint_id in enumerate(joint_ids):
                controller.mj_data.qpos[model.jnt_qposadr[joint_id]] = measured[index]
            mujoco.mj_forward(controller.mj_model, controller.mj_data)
            controller._update_ik()
            controller._send_command()
            target = controller.mj_data.ctrl[:6].copy()
            active = xr.get_key_value_by_name("right_grip") > 0.9
            command = follower.next(measured, target, active)
            if command is not None:
                arm.move_joints(command)
                last_command = command.copy()
                opening = 1.0 - float(np.clip(xr.get_key_value_by_name("right_trigger"), 0, 1))
                if (last_gripper is None or abs(opening - last_gripper) > 0.03) and tick - last_gripper_time >= 0.1:
                    arm.set_gripper(opening)
                    gripper_command_m = opening * gripper_max_width
                    last_gripper = opening
                    last_gripper_time = tick
            if recording:
                fp = arm.driver.get_flange_pose()
                gs = arm.gripper.get_gripper_status()
                flange = (np.asarray(fp.msg, dtype=np.float32) if fp is not None
                          and time.time() - fp.timestamp <= 0.3 else np.full(6, np.nan))
                gripper = (float(gs.msg.value) if gs is not None
                           and time.time() - gs.timestamp <= 0.3 else np.nan)
                recording.update({
                    "joint_rad": measured,
                    "action_joint_rad": last_command,
                    "tcp_flange": flange,
                    "tcp_eef": flange_rpy_to_eef_rpy(flange, TCP_OFFSET_M),
                    "gripper_m": gripper,
                    "action_gripper_m": gripper if not np.isfinite(gripper_command_m) else gripper_command_m,
                    "right_trigger": xr.get_key_value_by_name("right_trigger"),
                    "right_grip": xr.get_key_value_by_name("right_grip"),
                    "teleop_active": active,
                })
                if recording.controller._thread is None:
                    recording.start()
            time.sleep(max(0, period - (time.monotonic() - tick)))
    finally:
        try:
            if recording is not None:
                recording.close()
        finally:
            try:
                if arm is not None:
                    arm.close()
            finally:
                xr.close()


if __name__ == "__main__":
    tyro.cli(main)
