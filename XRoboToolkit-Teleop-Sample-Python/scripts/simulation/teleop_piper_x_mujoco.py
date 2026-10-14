"""PICO right-controller teleoperation of the AgileX Piper-X in MuJoCo."""

import time

import mujoco
import tyro

from xrobotoolkit_teleop.simulation.piper_x_model import (
    DEFAULT_GRIPPER_WIDTH, GRIPPER_JOINTS, HOME, JOINTS, TCP_LINK, build_scene,
)


def main(
    input_source: str = "pico",
    scale_factor: float = 0.5,
    headless_duration: float = 0.0,
    gripper_max_width: float = DEFAULT_GRIPPER_WIDTH,
) -> None:
    """Run the simulator; use ``fake`` for a headset-free smoke test."""
    if input_source not in {"pico", "fake"}:
        raise ValueError("input_source must be 'pico' or 'fake'")
    if not 0 < gripper_max_width <= 0.1:
        raise ValueError("gripper_max_width must be between 0 and 0.1 metres")
    if input_source == "fake":
        from teleop_fr3c_mujoco import _install_fake_xrt

        _install_fake_xrt()

    from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController

    urdf, scene = build_scene()
    controller = MujocoTeleopController(
        xml_path=str(scene),
        robot_urdf_path=str(urdf),
        manipulator_config={
            "right_hand": {
                "link_name": TCP_LINK,
                "pose_source": "right_controller",
                "control_trigger": "right_grip",
                "vis_target": "target",
                "gripper_config": {
                    "type": "parallel", "gripper_trigger": "right_trigger",
                    "joint_names": list(GRIPPER_JOINTS),
                    "open_pos": [gripper_max_width / 2, -gripper_max_width / 2],
                    "close_pos": [0.0, 0.0],
                },
            }
        },
        scale_factor=scale_factor,
        viewer_camera={"azimuth": 135, "elevation": -25, "distance": 1.4, "lookat": [0.15, 0, 0.28]},
    )
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints(dict(zip(JOINTS, HOME)))
    joints_task.configure("joints_regularization", "soft", 1e-4)
    if headless_duration <= 0:
        controller.run()
        return

    start = time.monotonic()
    steps = 0
    while time.monotonic() - start < headless_duration:
        controller._update_ik()
        controller._update_gripper_target()
        controller._update_mocap_target()
        controller._send_command()
        mujoco.mj_step(controller.mj_model, controller.mj_data)
        steps += 1
    print(f"Piper-X headless simulation completed: {steps} steps")


if __name__ == "__main__":
    tyro.cli(main)
