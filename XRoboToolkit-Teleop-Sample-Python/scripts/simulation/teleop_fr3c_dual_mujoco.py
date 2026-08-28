"""Teleoperate dual FR3C arms in MuJoCo with PICO controllers.

Mirrors the official UR5e dual-arm teleop sample (teleop_dual_ur5e_mujoco.py),
adapted for the FR3C dual-arm model in fr3c_assets/.  Right controller drives
the right arm, left controller drives the left arm; hold GRIP to take over.

Input sources:
  pico  -- XRoboToolkit SDK (requires the PC service running)
  fake  -- scripted controller motion (no headset needed, for testing)
"""
import math
import sys
import time
import types
from pathlib import Path

import mujoco
import numpy as np
import tyro

PICO_ROOT = Path(__file__).resolve().parents[3]
ASSET_DIR = PICO_ROOT / "fr3c_assets"

SIDES = ("left", "right")


def _install_fake_xrt():
    """Inject an in-memory xrobotoolkit_sdk so the official XrClient can run
    without a real headset.  This does not modify any framework file."""
    mod = types.ModuleType("xrobotoolkit_sdk")
    _t0 = time.monotonic()

    def _t():
        return time.monotonic() - _t0

    mod.init = lambda: print("Fake XRoboToolkit SDK initialized.")
    mod.close = lambda: None
    mod.get_left_controller_pose = lambda: np.array([
        0.05 * math.sin(_t()),
        0.0,
        0.02 * math.sin(_t() * 0.5),
        0.0, 0.0, 0.0, 1.0,
    ])
    mod.get_right_controller_pose = lambda: np.array([
        0.05 * math.sin(_t() + 1.0),
        0.0,
        0.02 * math.sin(_t() * 0.5 + 1.0),
        0.0, 0.0, 0.0, 1.0,
    ])
    mod.get_headset_pose = lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    mod.get_left_trigger = lambda: 0.0
    mod.get_right_trigger = lambda: 0.0
    mod.get_left_grip = lambda: 1.0
    mod.get_right_grip = lambda: 1.0
    mod.get_A_button = lambda: False
    mod.get_B_button = lambda: False
    mod.get_X_button = lambda: False
    mod.get_Y_button = lambda: False
    mod.get_left_menu_button = lambda: False
    mod.get_right_menu_button = lambda: False
    mod.get_left_axis_click = lambda: False
    mod.get_right_axis_click = lambda: False
    mod.get_left_axis = lambda: [0.0, 0.0]
    mod.get_right_axis = lambda: [0.0, 0.0]
    mod.get_time_stamp_ns = lambda: 0
    mod.get_left_hand_is_active = lambda: False
    mod.get_right_hand_is_active = lambda: False
    mod.get_left_hand_tracking_state = lambda: np.zeros((27, 7))
    mod.get_right_hand_tracking_state = lambda: np.zeros((27, 7))
    mod.num_motion_data_available = lambda: 0
    mod.get_motion_tracker_pose = lambda: []
    mod.get_motion_tracker_velocity = lambda: []
    mod.get_motion_tracker_acceleration = lambda: []
    mod.get_motion_tracker_serial_numbers = lambda: []
    mod.is_body_data_available = lambda: False
    mod.get_body_joints_pose = lambda: np.zeros((24, 7))
    mod.get_body_joints_velocity = lambda: np.zeros((24, 6))
    mod.get_body_joints_acceleration = lambda: np.zeros((24, 6))

    sys.modules["xrobotoolkit_sdk"] = mod


def main(
    xml_path: str = str(ASSET_DIR / "scene_fr3c_dual.xml"),
    robot_urdf_path: str = str(ASSET_DIR / "fr3c_teleop_dual.urdf"),
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    input_source: str = "pico",
    headless_duration: float = 0.0,
    robot_left_ip: str = "",
    robot_right_ip: str = "",
):
    """
    Run dual FR3C teleoperation in MuJoCo.

    Args:
        input_source: "pico" for real headset, "fake" for scripted test motion.
        headless_duration: If > 0, run without viewer for this many seconds.
        robot_left_ip: If set (with robot_right_ip), read the real arm's
            current joints via the Fairino SDK and start the sim from that pose.
        robot_right_ip: See robot_left_ip.
    """
    if input_source == "fake":
        _install_fake_xrt()

    from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
        MujocoTeleopController,
    )

    qpos_init = None
    if robot_left_ip or robot_right_ip:
        from xrobotoolkit_teleop.hardware.interface.fr3c import Fr3cController

        if not (robot_left_ip and robot_right_ip):
            raise SystemExit("Both --robot-left-ip and --robot-right-ip are required.")
        qpos_init = np.zeros(12)
        for i, (side, ip) in enumerate((("left", robot_left_ip), ("right", robot_right_ip))):
            arm = Fr3cController(robot_ip=ip)
            q = arm.get_current_joint_positions()
            arm.close()
            qpos_init[i * 6 : (i + 1) * 6] = q
            print(f"{side} arm ({ip}) current joints (deg): {np.round(np.rad2deg(q), 3).tolist()}")

    config = {
        "right_hand": {
            "link_name": "right_wrist3_Link",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
        },
        "left_hand": {
            "link_name": "left_wrist3_Link",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "left_target",
        },
    }

    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        mj_qpos_init=qpos_init,
    )

    # Joint regularization toward the start pose (same pattern as the UR5e sample)
    joints_task = controller.solver.add_joints_task()
    start_q = qpos_init if qpos_init is not None else controller.mj_model.key("home").qpos
    joint_targets = {name: 0.0 for name in controller.placo_robot.joint_names()}
    for i, side in enumerate(SIDES):
        for j in range(6):
            joint_targets[f"{side}_j{j + 1}"] = float(start_q[i * 6 + j])
    joints_task.set_joints(joint_targets)
    joints_task.configure("joints_regularization", "soft", 1e-4)

    if headless_duration > 0:
        _run_headless(controller, headless_duration)
    else:
        controller.run()


def _run_headless(controller, duration):
    """Run the control loop without a MuJoCo viewer (for automated testing)."""
    t0 = time.monotonic()
    step = 0
    while time.monotonic() - t0 < duration:
        controller._update_robot_state()
        controller._update_ik()
        controller._update_gripper_target()
        controller._update_mocap_target()
        controller._send_command()
        mujoco.mj_step(controller.mj_model, controller.mj_data)
        step += 1
    print(f"Headless run completed: {duration:.1f}s, {step} steps")


if __name__ == "__main__":
    tyro.cli(main)
