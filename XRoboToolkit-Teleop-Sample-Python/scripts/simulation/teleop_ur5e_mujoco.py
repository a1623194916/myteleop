"""Teleoperate one UR5e in MuJoCo with the right PICO controller."""

import math
import os
import sys
import tempfile
import time
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import tyro

ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = ROOT / "assets" / "universal_robots_ur5e"


def _build_matching_urdf():
    """Create a temporary URDF whose six joint names match the MuJoCo model."""
    source = ASSET_DIR / "ur5e.urdf"
    tree = ET.parse(source)
    rename = {
        "shoulder_pan_joint": "shoulder_pan",
        "shoulder_lift_joint": "shoulder_lift",
        "elbow_joint": "elbow",
        "wrist_1_joint": "wrist_1",
        "wrist_2_joint": "wrist_2",
        "wrist_3_joint": "wrist_3",
    }
    for joint in tree.getroot().iter("joint"):
        old_name = joint.get("name")
        if old_name in rename:
            joint.set("name", rename[old_name])
    handle = tempfile.NamedTemporaryFile(
        mode="wb", suffix=".urdf", prefix="ur5e_teleop_", dir=ASSET_DIR, delete=False
    )
    with handle:
        tree.write(handle, encoding="utf-8", xml_declaration=True)
    return handle.name


def _install_fake_xrt():
    mod = types.ModuleType("xrobotoolkit_sdk")
    started = time.monotonic()

    def elapsed():
        return time.monotonic() - started

    neutral = lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    mod.init = lambda: print("Fake XRoboToolkit SDK initialized.")
    mod.close = lambda: None
    mod.get_left_controller_pose = neutral
    mod.get_right_controller_pose = lambda: np.array(
        [0.04 * math.sin(elapsed()), 0.0, 0.02 * math.sin(0.5 * elapsed()), 0.0, 0.0, 0.0, 1.0]
    )
    mod.get_headset_pose = neutral
    mod.get_left_trigger = lambda: 0.0
    mod.get_right_trigger = lambda: 0.5 + 0.5 * math.sin(elapsed())
    mod.get_left_grip = lambda: 0.0
    mod.get_right_grip = lambda: 1.0
    for name in ("A", "B", "X", "Y"):
        setattr(mod, f"get_{name}_button", lambda: False)
    mod.get_left_menu_button = lambda: False
    mod.get_right_menu_button = lambda: False
    mod.get_left_axis_click = lambda: False
    mod.get_right_axis_click = lambda: False
    mod.get_left_axis = lambda: [0.0, 0.0]
    mod.get_right_axis = lambda: [0.0, 0.0]
    mod.get_time_stamp_ns = lambda: time.monotonic_ns()
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
    sys.modules["xrobotoolkit_sdk"] = mod


def _run_headless(controller, duration):
    started = time.monotonic()
    steps = 0
    while time.monotonic() - started < duration:
        controller._update_robot_state()
        controller._update_ik()
        controller._update_gripper_target()
        controller._update_mocap_target()
        controller._send_command()
        mujoco.mj_step(controller.mj_model, controller.mj_data)
        steps += 1
    print(f"UR5e headless simulation completed: {duration:.1f}s, {steps} steps")


def main(
    input_source: str = "pico",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    headless_duration: float = 0.0,
):
    """Run single-arm UR5e simulation; hold right GRIP to move the arm."""
    if input_source not in {"pico", "fake"}:
        raise ValueError("input_source must be 'pico' or 'fake'")
    if input_source == "fake":
        _install_fake_xrt()

    from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController

    robot_urdf_path = _build_matching_urdf()
    try:
        controller = MujocoTeleopController(
            xml_path=str(ASSET_DIR / "scene.xml"),
            robot_urdf_path=robot_urdf_path,
            manipulator_config={
                "right_hand": {
                    "link_name": "tool0",
                    "pose_source": "right_controller",
                    "control_trigger": "right_grip",
                    "vis_target": "target",
                }
            },
            scale_factor=scale_factor,
            visualize_placo=visualize_placo,
        )
    finally:
        os.unlink(robot_urdf_path)
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    if headless_duration > 0:
        _run_headless(controller, headless_duration)
    else:
        controller.run()


if __name__ == "__main__":
    tyro.cli(main)
