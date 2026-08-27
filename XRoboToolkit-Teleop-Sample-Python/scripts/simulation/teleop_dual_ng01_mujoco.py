import math
import os
import time
from pathlib import Path

import meshcat.transformations as tf
import mujoco
import numpy as np
import tyro

from xrobotoolkit_teleop.common.fake_xr_client import FakeXrClient
from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController


DEFAULT_NG01_ROOT = Path(__file__).resolve().parents[3] / "NG01_v4"

MARKER_COLORS = {
    "waiting": np.array([0.2, 0.9, 0.3, 0.5]),
    "active": np.array([1.0, 0.6, 0.1, 0.6]),
}


def build_manipulator_config(control_mode: str = "position"):
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'.")

    def hand_config(side: str):
        return {
            "link_name": f"{side[0]}tcp_link",
            "pose_source": f"{side}_controller",
            "control_trigger": f"{side}_grip",
            "vis_target": f"{side[0]}_hand_vis_target",
            "control_mode": control_mode,
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": f"{side}_trigger",
                "joint_names": [f"{side[0]}gripper_finger_joint"],
                "open_pos": [0.0],
                "close_pos": [1.0472],
            },
        }

    return {
        "left_hand": hand_config("left"),
        "right_hand": hand_config("right"),
    }


def _make_fake_input_updater(client: FakeXrClient, clock=time.monotonic):
    start = clock()

    def update() -> None:
        _update_fake_inputs(client, clock() - start)

    return update


def _make_gripper_reporter(controller: MujocoTeleopController, clock=time.monotonic):
    joint_names = ("lgripper_finger_joint", "rgripper_finger_joint")
    actuator_names = tuple(f"{name}_servo" for name in joint_names)
    qpos_addresses = tuple(controller.mj_model.jnt_qposadr[controller.mj_model.joint(name).id] for name in joint_names)
    actuator_ids = tuple(controller.mj_model.actuator(name).id for name in actuator_names)
    next_report = clock()

    def report() -> None:
        nonlocal next_report
        now = clock()
        if now < next_report:
            return
        next_report = now + 1.0
        triggers = [controller.xr_client.get_key_value_by_name(f"{side}_trigger") for side in ("left", "right")]
        targets = [
            controller.gripper_pos_target[hand][joint]
            for hand, joint in zip(("left_hand", "right_hand"), joint_names)
        ]
        controls = [controller.mj_data.ctrl[actuator_id] for actuator_id in actuator_ids]
        positions = [controller.mj_data.qpos[qpos_address] for qpos_address in qpos_addresses]
        print(
            "grippers "
            f"trigger L/R={triggers[0]:.1f}/{triggers[1]:.1f}  "
            f"target={targets[0]:.4f}/{targets[1]:.4f}  "
            f"ctrl={controls[0]:.4f}/{controls[1]:.4f}  "
            f"qpos={positions[0]:.4f}/{positions[1]:.4f}"
        )

    return report


def _make_xr_status_reporter(controller: "AlignAndStartTeleopController", clock=time.monotonic):
    next_report = clock()

    def report() -> None:
        nonlocal next_report
        now = clock()
        if now < next_report:
            return
        next_report = now + 2.0
        parts = []
        for name, config in controller.manipulator_config.items():
            side = "L" if name.startswith("left") else "R"
            pose = controller.xr_client.get_pose_by_name(config["pose_source"])
            grip = controller.xr_client.get_key_value_by_name(config["control_trigger"])
            trigger = controller.xr_client.get_key_value_by_name(config["gripper_config"]["gripper_trigger"])
            valid = "ok" if controller.pose_is_valid(pose) else "INVALID"
            parts.append(
                f"{side} {'ACTIVE' if controller.is_armed(name) else 'waiting'} "
                f"pose[{valid}] xyz=({pose[0]:+.2f},{pose[1]:+.2f},{pose[2]:+.2f}) "
                f"grip={grip:.2f} trig={trigger:.2f}"
            )
        print("xr | " + " | ".join(parts))

    return report


class AlignAndStartTeleopController(MujocoTeleopController):
    """Teleop with explicit alignment: markers show the home hand poses, the user
    aligns controllers to them, then squeezing the grip starts continuous tracking.
    The side button releases the arm back to its home pose."""

    GRIP_ARM_THRESHOLD = 0.9
    RELEASE_BUTTONS = {"left_hand": "X", "right_hand": "B"}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._armed = {name: False for name in self.manipulator_config}
        self._anchor_controller_pose = {}
        self._anchor_ee_pose = {}
        self._home_ee_pose = {
            name: self._get_link_pose(config["link_name"]) for name, config in self.manipulator_config.items()
        }
        self._prev_button_state = {}
        self._warned_invalid_pose = set()
        self._rearm_allowed = {name: True for name in self.manipulator_config}

    def is_armed(self, name: str) -> bool:
        return self._armed[name]

    def pose_is_valid(self, xr_pose) -> bool:
        return bool(np.linalg.norm(np.asarray(xr_pose[3:7], dtype=float)) > 0.5)

    def set_marker_color(self, name: str, color: np.ndarray) -> None:
        geom_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_GEOM, self.manipulator_config[name]["vis_target"] + "_geom")
        if geom_id != -1:
            self.mj_model.geom_rgba[geom_id] = color

    def _button_pressed(self, name: str, button: str) -> bool:
        getter = getattr(self.xr_client, "get_button_state_by_name", None)
        if getter is None:
            return False
        pressed = bool(getter(button))
        key = (name, button)
        previous = self._prev_button_state.get(key, False)
        self._prev_button_state[key] = pressed
        return pressed and not previous

    def _controller_world_pose(self, xr_pose):
        xyz = np.asarray(xr_pose[:3], dtype=float)
        quat = np.array([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]])
        pos_world = self.R_headset_world @ xyz
        rot_transform = np.eye(4)
        rot_transform[:3, :3] = self.R_headset_world
        rot_quat = tf.quaternion_from_matrix(rot_transform)
        quat_world = tf.quaternion_multiply(tf.quaternion_multiply(rot_quat, quat), tf.quaternion_conjugate(rot_quat))
        return pos_world, quat_world

    def _arm(self, name: str, config) -> bool:
        xr_pose = self.xr_client.get_pose_by_name(config["pose_source"])
        if not self.pose_is_valid(xr_pose):
            if name not in self._warned_invalid_pose:
                print(f"Warning: {config['pose_source']} pose is invalid, cannot start tracking yet.")
                self._warned_invalid_pose.add(name)
            return False
        self._warned_invalid_pose.discard(name)
        self._anchor_controller_pose[name] = self._controller_world_pose(xr_pose)
        self._anchor_ee_pose[name] = self._get_link_pose(config["link_name"])
        self._armed[name] = True
        self.active[name] = True
        self.set_marker_color(name, MARKER_COLORS["active"])
        print(f"{name} armed: tracking {config['pose_source']}. Move to drive, press "
              f"'{self.RELEASE_BUTTONS.get(name, '?')}' to release.")
        return True

    def _disarm(self, name: str) -> None:
        if not self._armed[name]:
            return
        self._armed[name] = False
        self.active[name] = False
        self._rearm_allowed[name] = False
        self._reset_task_to_home(name)
        self.set_marker_color(name, MARKER_COLORS["waiting"])
        print(f"{name} released: returning to home pose.")

    def _reset_task_to_home(self, name: str) -> None:
        home_xyz, home_quat = self._home_ee_pose[name]
        task = self.effector_task[name]
        if self.effector_control_mode[name] == "position":
            task.target_world = home_xyz.copy()
        else:
            home_frame = tf.quaternion_matrix(home_quat)
            home_frame[:3, 3] = home_xyz
            task.T_world_frame = home_frame

    def _follow_controller(self, name: str, config) -> None:
        xr_pose = self.xr_client.get_pose_by_name(config["pose_source"])
        if not self.pose_is_valid(xr_pose):
            return
        ctrl_pos, ctrl_quat = self._controller_world_pose(xr_pose)
        anchor_ctrl_pos, anchor_ctrl_quat = self._anchor_controller_pose[name]
        ee_pos, ee_quat = self._anchor_ee_pose[name]

        target_xyz = ee_pos + self.scale_factor * (ctrl_pos - anchor_ctrl_pos)
        task = self.effector_task[name]
        if self.effector_control_mode[name] == "position":
            task.target_world = target_xyz
            return
        rel_quat = tf.quaternion_multiply(ctrl_quat, tf.quaternion_conjugate(anchor_ctrl_quat))
        target_quat = tf.quaternion_multiply(rel_quat, ee_quat)
        target_frame = tf.quaternion_matrix(target_quat)
        target_frame[:3, 3] = target_xyz
        task.T_world_frame = target_frame

    def _update_ik(self):
        self._update_robot_state()
        self.placo_robot.update_kinematics()

        for name, config in self.manipulator_config.items():
            release_button = self.RELEASE_BUTTONS.get(name)
            if release_button and self._button_pressed(name, release_button):
                self._disarm(name)

            if not self._armed[name]:
                grip_value = self.xr_client.get_key_value_by_name(config["control_trigger"])
                if grip_value > self.GRIP_ARM_THRESHOLD:
                    if self._rearm_allowed[name]:
                        self._arm(name, config)
                else:
                    self._rearm_allowed[name] = True

            if self._armed[name]:
                self._follow_controller(name, config)

        try:
            self.solver.solve(True)
        except RuntimeError as e:
            print(f"IK solver failed: {e}")


def build_controller(
    input_source: str = "fake",
    control_mode: str = "position",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    ng01_root: Path = DEFAULT_NG01_ROOT,
):
    ng01_root = Path(ng01_root).resolve()
    if input_source == "fake":
        xr_client = FakeXrClient()
        pre_step_callback = _make_fake_input_updater(xr_client)
    elif input_source == "pico":
        xr_client = XrClient()
        pre_step_callback = None
    else:
        raise ValueError("input_source must be 'fake' or 'pico'.")

    controller = AlignAndStartTeleopController(
        xml_path=str(ng01_root / "NG01_mjcf" / "NG01_teleop.xml"),
        robot_urdf_path=str(ng01_root / "urdf" / "NG01_teleop.urdf"),
        manipulator_config=build_manipulator_config(control_mode),
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        xr_client=xr_client,
        pre_step_callback=pre_step_callback,
        viewer_camera={
            "azimuth": 90,
            "elevation": -10,
            "distance": 1.8,
            "lookat": [0.0, 0.0, 1.15],
        },
    )

    home = controller.mj_model.key("home").qpos
    joints_task = controller.solver.add_joints_task()
    joint_names = [
        "up_down_joint",
        *(f"ljoint{i}" for i in range(1, 8)),
        *(f"rjoint{i}" for i in range(1, 8)),
        "lgripper_finger_joint",
        "rgripper_finger_joint",
    ]
    joints_task.set_joints(dict(zip(joint_names, home)))
    joints_task.configure("ng01_home_regularization", "soft", 1e-4)
    controller.solver.enable_velocity_limits(True)
    if input_source == "fake":
        controller.post_step_callback = _make_gripper_reporter(controller)
    else:
        controller.post_step_callback = _make_xr_status_reporter(controller)
    return controller


def _update_fake_inputs(client: FakeXrClient, elapsed: float) -> None:
    phase = elapsed * 0.5
    displacement = 0.04 * math.sin(phase)
    left_pose = np.array([0.0, displacement, 0.02 * math.sin(phase * 0.5), 0.0, 0.0, 0.0, 1.0])
    right_pose = np.array([0.0, -displacement, 0.02 * math.sin(phase * 0.5), 0.0, 0.0, 0.0, 1.0])
    client.set_pose("left_controller", left_pose)
    client.set_pose("right_controller", right_pose)
    client.set_key_value("left_grip", 1.0)
    client.set_key_value("right_grip", 1.0)
    gripper_phase = elapsed % 4.0
    left_trigger = 1.0 if gripper_phase < 2.0 else 0.0
    right_trigger = 1.0 - left_trigger
    client.set_key_value("left_trigger", left_trigger)
    client.set_key_value("right_trigger", right_trigger)


def run_fake_demo(controller: MujocoTeleopController, duration: float) -> None:
    start = time.monotonic()
    while time.monotonic() - start < duration:
        step_start = time.monotonic()
        controller.step()
        remaining = controller.dt - (time.monotonic() - step_start)
        if remaining > 0:
            time.sleep(remaining)


def print_instructions() -> None:
    print(
        "\n=== NG01 teleoperation (align-and-start) ===\n"
        "1. Green spheres mark the robot's home hand poses.\n"
        "2. Hold the controllers so they align with the spheres.\n"
        "3. Squeeze the side GRIP on a controller to take over that arm\n"
        "   (sphere turns orange, arm follows your hand).\n"
        "4. Index trigger closes/opens the gripper at any time.\n"
        "5. Press X (left controller) / B (right controller) to release\n"
        "   that arm back to the home pose.\n"
        "=============================================\n"
    )


def main(
    input_source: str = "fake",
    control_mode: str = "position",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    headless_duration: float = 0.0,
    ng01_root: str = os.fspath(DEFAULT_NG01_ROOT),
):
    """Run NG01 dual-arm and gripper teleoperation in MuJoCo."""
    controller = build_controller(
        input_source=input_source,
        control_mode=control_mode,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        ng01_root=Path(ng01_root),
    )
    print_instructions()
    if headless_duration > 0:
        if input_source != "fake":
            raise ValueError("headless_duration is only supported with fake input.")
        run_fake_demo(controller, headless_duration)
    else:
        controller.run()


if __name__ == "__main__":
    tyro.cli(main)
