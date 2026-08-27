"""Teleoperate a single FR3C arm in MuJoCo with a PICO controller.

Input sources:
  fake  -- scripted sinusoidal controller motion (no headset needed)
  pico  -- XRoboToolkit SDK (requires the PC service running)

Control scheme (same as the other XRoboToolkit teleop samples):
  grip    -- hold to track the controller pose (arm takes over)
  trigger -- reserved for a gripper once FR3C gets one
"""
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
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)

PICO_ROOT = Path(__file__).resolve().parents[3]
ASSET_DIR = PICO_ROOT / "fr3c_assets"

MARKER_COLORS = {
    "waiting": np.array([0.2, 0.9, 0.3, 0.5]),
    "active": np.array([1.0, 0.6, 0.1, 0.6]),
}

RELEASE_BUTTON = "B"


def build_manipulator_config(control_mode: str = "pose") -> dict:
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'.")
    return {
        "right_hand": {
            "link_name": "wrist3_Link",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
            "control_mode": control_mode,
        }
    }


class ArmDisarmTeleopController(MujocoTeleopController):
    """Grip to take over the arm, release button (B) to return home."""

    GRIP_ARM_THRESHOLD = 0.9

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._armed = {name: False for name in self.manipulator_config}
        self._rearm_allowed = {name: True for name in self.manipulator_config}
        self._anchor_controller_pose = {}
        self._anchor_ee_pose = {}
        self._prev_button_state = {}
        self._warned_invalid_pose = set()

        # Start the simulation at the home keyframe so FK/IK and the initial
        # position task agree (otherwise the solver starts at the zero pose).
        self.mj_data.qpos[:] = self.mj_model.key("home").qpos
        mujoco.mj_forward(self.mj_model, self.mj_data)

        # Home tool pose evaluated at the home keyframe.
        self._home_ee_pose = {
            name: self._get_link_pose(config["link_name"])
            for name, config in self.manipulator_config.items()
        }

        # Seed the end-effector task at home so WAITING still solves cleanly.
        for name in self.manipulator_config:
            self._reset_task_to_home(name)

    def is_armed(self, name: str) -> bool:
        return self._armed[name]

    def pose_is_valid(self, xr_pose) -> bool:
        return bool(np.linalg.norm(np.asarray(xr_pose[3:7], dtype=float)) > 0.5)

    def set_marker_color(self, name: str, color: np.ndarray) -> None:
        geom_id = mujoco.mj_name2id(
            self.mj_model, mujoco.mjtObj.mjOBJ_GEOM,
            self.manipulator_config[name]["vis_target"] + "_geom",
        )
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
        quat_world = tf.quaternion_multiply(
            tf.quaternion_multiply(rot_quat, quat),
            tf.quaternion_conjugate(rot_quat),
        )
        return pos_world, quat_world

    def _arm(self, name: str, config) -> bool:
        xr_pose = self.xr_client.get_pose_by_name(config["pose_source"])
        if not self.pose_is_valid(xr_pose):
            if name not in self._warned_invalid_pose:
                print(f"Warning: {config['pose_source']} pose invalid, cannot start tracking yet.")
                self._warned_invalid_pose.add(name)
            return False
        self._warned_invalid_pose.discard(name)
        self._anchor_controller_pose[name] = self._controller_world_pose(xr_pose)
        self._anchor_ee_pose[name] = self._get_link_pose(config["link_name"])
        self._armed[name] = True
        self.active[name] = True
        self.set_marker_color(name, MARKER_COLORS["active"])
        print(f"{name} armed: tracking {config['pose_source']}. "
              f"Press '{RELEASE_BUTTON}' to release.")
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

    def _update_robot_state(self):
        super()._update_robot_state()
        # MuJoCo enforces joint limits as soft constraints, so the simulated
        # joints can end up *slightly* outside their range. placo treats the
        # same limits as hard QP inequalities, and one state outside the box
        # makes every subsequent solve infeasible (permanent deadlock).
        # Clamp the synced state just inside the placo limits.
        model = self.placo_robot.model
        q = np.asarray(self.placo_robot.state.q, dtype=float).copy()
        lo = model.lowerPositionLimit
        hi = model.upperPositionLimit
        q[7:13] = np.clip(q[7:13], lo[7:13] + 1e-6, hi[7:13] - 1e-6)
        self.placo_robot.state.q = q

    def _send_command(self):
        q = np.asarray(self.placo_robot.state.q, dtype=float)
        if not np.isfinite(q).all():
            # A failed solve can leave non-finite values behind; keep the last
            # good command instead of poisoning MuJoCo (which crashes the QP
            # solver with a segfault on the next step).
            return
        super()._send_command()

    def _update_ik(self):
        self._update_robot_state()
        self.placo_robot.update_kinematics()

        for name, config in self.manipulator_config.items():
            if self._button_pressed(name, RELEASE_BUTTON):
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


def _make_fake_input_updater(client: FakeXrClient, clock=time.monotonic):
    start = clock()

    def update() -> None:
        _update_fake_inputs(client, clock() - start)

    return update


def _update_fake_inputs(client: FakeXrClient, elapsed: float) -> None:
    phase = elapsed * 0.5
    displacement = 0.05 * math.sin(phase)
    pose = np.array([displacement, 0.0, 0.03 * math.sin(phase * 0.5), 0.0, 0.0, 0.0, 1.0])
    client.set_pose("right_controller", pose)
    client.set_key_value("right_grip", 1.0 if 0.5 < (elapsed % 8.0) < 5.5 else 0.0)


def _make_status_reporter(controller: ArmDisarmTeleopController, clock=time.monotonic):
    next_report = clock()

    def report() -> None:
        nonlocal next_report
        now = clock()
        if now < next_report:
            return
        next_report = now + 2.0
        config = controller.manipulator_config["right_hand"]
        pose = controller.xr_client.get_pose_by_name(config["pose_source"])
        grip = controller.xr_client.get_key_value_by_name(config["control_trigger"])
        state = "ACTIVE" if controller.is_armed("right_hand") else "waiting"
        print(
            f"xr | right {state} xyz=({pose[0]:+.2f},{pose[1]:+.2f},{pose[2]:+.2f}) grip={grip:.2f}"
        )

    return report


def build_controller(
    input_source: str = "fake",
    control_mode: str = "position",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
):
    if input_source == "fake":
        xr_client = FakeXrClient()
        pre_step_callback = _make_fake_input_updater(xr_client)
    elif input_source == "pico":
        xr_client = XrClient()
        pre_step_callback = None
    else:
        raise ValueError("input_source must be 'fake' or 'pico'.")

    controller = ArmDisarmTeleopController(
        xml_path=str(ASSET_DIR / "scene_fr3c.xml"),
        robot_urdf_path=str(ASSET_DIR / "fr3c_teleop.urdf"),
        manipulator_config=build_manipulator_config(control_mode),
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        xr_client=xr_client,
        pre_step_callback=pre_step_callback,
        viewer_camera={
            "azimuth": 90,
            "elevation": -15,
            "distance": 1.6,
            "lookat": [0.15, 0.0, 0.4],
        },
    )

    joints_task = controller.solver.add_joints_task()
    joint_names = [f"j{i}" for i in range(1, 7)]
    home = controller.mj_model.key("home").qpos
    joints_task.set_joints(dict(zip(joint_names, home)))
    # Weight must dominate in the redundant DOFs: a pure position task cannot
    # observe j6 at all (its axis passes through the tracked link origin), so
    # with a weak regularization j6 free-drifts on gravity until it hits the
    # joint limit and the QP goes transiently infeasible.
    joints_task.configure("fr3c_home_regularization", "soft", 0.05)

    controller.post_step_callback = _make_status_reporter(controller)
    return controller


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
        "\n=== FR3C teleoperation (MuJoCo, single arm) ===\n"
        "1. A green marker shows the arm's home tool pose.\n"
        "2. Squeeze the right GRIP to take over the arm "
        "(marker turns orange, tool follows your controller).\n"
        "3. Press B to release the arm back to the home pose.\n"
        "4. The index trigger is reserved for a gripper later on.\n"
        "==============================================="
    )


def main(
    input_source: str = "fake",
    control_mode: str = "position",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    headless_duration: float = 0.0,
):
    """Run single-arm FR3C teleoperation in MuJoCo."""
    controller = build_controller(
        input_source=input_source,
        control_mode=control_mode,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
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
