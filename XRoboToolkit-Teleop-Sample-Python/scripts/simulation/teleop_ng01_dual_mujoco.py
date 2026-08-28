"""Teleoperate the dual NG01 arms in MuJoCo with PICO controllers.

Mirrors the official UR5e dual-arm teleop sample (teleop_dual_ur5e_mujoco.py),
adapted for the NG01 wheeled humanoid model in ../NG01_v4.  The left
controller drives the left arm, the right controller drives the right arm;
hold GRIP to take over an arm, move the index trigger to close/open that
arm's gripper.  The torso lift stays at its home height through the
regularized joint task.

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
NG01_ROOT = PICO_ROOT / "NG01_v4"

VIEWER_CAMERA = {
    "azimuth": 90,
    "elevation": -10,
    "distance": 1.8,
    "lookat": [0.0, 0.0, 1.15],
}

JOINT_NAMES = [
    "up_down_joint",
    *(f"ljoint{i}" for i in range(1, 8)),
    *(f"rjoint{i}" for i in range(1, 8)),
    "lgripper_finger_joint",
    "rgripper_finger_joint",
]


def build_manipulator_config():
    def hand_config(side):
        prefix = "l" if side == "left" else "r"
        return {
            "link_name": f"{prefix}tcp_link",
            "pose_source": f"{side}_controller",
            "control_trigger": f"{side}_grip",
            "vis_target": f"{prefix}_hand_vis_target",
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": f"{side}_trigger",
                "joint_names": [f"{prefix}gripper_finger_joint"],
                "open_pos": [0.0],
                "close_pos": [1.0472],
            },
        }

    return {"left_hand": hand_config("left"), "right_hand": hand_config("right")}


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
        0.04 * math.sin(_t()),
        0.0,
        0.02 * math.sin(_t() * 0.5),
        0.0, 0.0, 0.0, 1.0,
    ])
    mod.get_right_controller_pose = lambda: np.array([
        0.04 * math.sin(_t() + 1.0),
        0.0,
        0.02 * math.sin(_t() * 0.5 + 1.0),
        0.0, 0.0, 0.0, 1.0,
    ])
    mod.get_headset_pose = lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    # Alternate the two grippers fully open/closed every 2s so headless runs
    # exercise both gripper channels
    mod.get_left_trigger = lambda: 1.0 if (_t() % 4.0) < 2.0 else 0.0
    mod.get_right_trigger = lambda: 0.0 if (_t() % 4.0) < 2.0 else 1.0
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


def build_controller(
    input_source: str = "fake",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
):
    if input_source == "fake":
        _install_fake_xrt()
    elif input_source != "pico":
        raise ValueError("input_source must be 'pico' or 'fake'.")

    from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
        MujocoTeleopController,
    )

    controller = MujocoTeleopController(
        xml_path=str(NG01_ROOT / "NG01_mjcf" / "NG01_teleop.xml"),
        robot_urdf_path=str(NG01_ROOT / "urdf" / "NG01_teleop.urdf"),
        manipulator_config=build_manipulator_config(),
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        viewer_camera=VIEWER_CAMERA,
    )

    # Regularize every joint toward the home keyframe (keeps the torso lift
    # and idle joints in place, same pattern as the UR5e sample)
    joints_task = controller.solver.add_joints_task()
    home = controller.mj_model.key("home").qpos
    joint_targets = {name: 0.0 for name in controller.placo_robot.joint_names()}
    for name in JOINT_NAMES:
        joint_id = mujoco.mj_name2id(controller.mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joint_targets[name] = float(home[controller.mj_model.jnt_qposadr[joint_id]])
    joints_task.set_joints(joint_targets)
    joints_task.configure("joints_regularization", "soft", 1e-4)
    controller.joint_targets = joint_targets

    return controller


def main(
    input_source: str = "pico",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    headless_duration: float = 0.0,
):
    """
    Run dual NG01 arm teleoperation in MuJoCo.

    Args:
        input_source: "pico" for real headset, "fake" for scripted test motion.
        headless_duration: If > 0, run without viewer for this many seconds.
    """
    controller = build_controller(
        input_source=input_source,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
    )

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
