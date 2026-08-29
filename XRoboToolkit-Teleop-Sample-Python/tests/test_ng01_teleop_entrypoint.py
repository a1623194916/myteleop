import sys
import unittest
from pathlib import Path

import mujoco
import numpy as np

from scripts.simulation.teleop_ng01_dual_mujoco import (
    JOINT_NAMES,
    VIEWER_CAMERA,
    build_controller,
    build_manipulator_config,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
NG01_ROOT = REPO_ROOT.parent / "NG01_v4"


class Ng01TeleopEntryPointTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # build_controller installs a fake xrobotoolkit_sdk before the
        # framework's XrClient imports the SDK, so no headset is needed
        cls.controller = build_controller(input_source="fake")

    def test_configuration_targets_both_tcps_and_grippers(self):
        config = build_manipulator_config()

        self.assertEqual(set(config), {"left_hand", "right_hand"})
        self.assertEqual(config["left_hand"]["link_name"], "ltcp_link")
        self.assertEqual(config["right_hand"]["link_name"], "rtcp_link")
        self.assertEqual(
            config["left_hand"]["gripper_config"]["joint_names"],
            ["lgripper_finger_joint"],
        )
        self.assertEqual(
            config["right_hand"]["gripper_config"]["joint_names"],
            ["rgripper_finger_joint"],
        )
        self.assertEqual(config["left_hand"]["vis_target"], "l_hand_vis_target")
        self.assertEqual(config["right_hand"]["vis_target"], "r_hand_vis_target")

    def test_controller_regularizes_ik_joints_toward_home(self):
        home = self.controller.mj_model.key("home").qpos
        # the torso lift is welded out of the placo model, never an IK target
        self.assertNotIn("up_down_joint", self.controller.joint_targets)
        for name in JOINT_NAMES:
            if name == "up_down_joint":
                continue
            joint_id = mujoco.mj_name2id(
                self.controller.mj_model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            self.assertAlmostEqual(
                self.controller.joint_targets[name],
                float(home[self.controller.mj_model.jnt_qposadr[joint_id]]),
            )

    def test_viewer_camera_frames_torso_from_front(self):
        self.assertEqual(self.controller.viewer_camera, VIEWER_CAMERA)
        self.assertEqual(VIEWER_CAMERA["lookat"], [0.0, 0.0, 1.15])

    def test_trigger_drives_only_its_own_gripper(self):
        xrt = sys.modules["xrobotoolkit_sdk"]
        original = (xrt.get_left_trigger, xrt.get_right_trigger)
        xrt.get_left_trigger = lambda: 0.25
        xrt.get_right_trigger = lambda: 0.75
        try:
            self.controller._update_gripper_target()
        finally:
            xrt.get_left_trigger, xrt.get_right_trigger = original

        self.assertAlmostEqual(
            self.controller.gripper_pos_target["left_hand"]["lgripper_finger_joint"],
            1.0472 * 0.25,
        )
        self.assertAlmostEqual(
            self.controller.gripper_pos_target["right_hand"]["rgripper_finger_joint"],
            1.0472 * 0.75,
        )

    def test_fake_input_runs_steps_with_finite_state(self):
        xrt = sys.modules["xrobotoolkit_sdk"]
        original = (xrt.get_left_trigger, xrt.get_right_trigger)
        xrt.get_left_trigger = lambda: 1.0
        xrt.get_right_trigger = lambda: 0.0
        try:
            for _ in range(10):
                self.controller._update_robot_state()
                self.controller._update_ik()
                self.controller._update_gripper_target()
                self.controller._update_mocap_target()
                self.controller._send_command()
                mujoco.mj_step(self.controller.mj_model, self.controller.mj_data)
        finally:
            xrt.get_left_trigger, xrt.get_right_trigger = original

        self.assertTrue(np.isfinite(self.controller.mj_data.qpos).all())
        self.assertAlmostEqual(
            self.controller.gripper_pos_target["left_hand"]["lgripper_finger_joint"],
            1.0472,
        )
        self.assertAlmostEqual(
            self.controller.gripper_pos_target["right_hand"]["rgripper_finger_joint"],
            0.0,
        )

    def test_drag_mode_targets_follow_mocap_and_track(self):
        controller = build_controller(input_source="drag")

        # spheres start aligned with the home TCP poses
        mocap_idx = controller.target_mocap_idx["right_hand"]
        home_quat = controller.mj_data.mocap_quat[mocap_idx].copy()
        home_pos = controller.mj_data.mocap_pos[mocap_idx].copy()
        np.testing.assert_allclose(
            home_pos,
            controller.mj_data.xpos[
                mujoco.mj_name2id(
                    controller.mj_model, mujoco.mjtObj.mjOBJ_BODY, "rtcp_link"
                )
            ],
            atol=1e-6,
        )

        # drag: 0.08/-0.12/0.05 m translation plus a 25 deg rotation about
        # world Y, exactly what a viewer Ctrl-drag writes into mj_data
        # (an arm-reachable target: the lift is locked, it cannot help)
        controller.mj_data.mocap_pos[mocap_idx] += np.array([-0.08, -0.12, 0.05])
        angle = np.deg2rad(25.0)
        delta_quat = np.array([np.cos(angle / 2), 0.0, np.sin(angle / 2), 0.0])
        import meshcat.transformations as tf

        controller.mj_data.mocap_quat[mocap_idx] = tf.quaternion_multiply(
            delta_quat, home_quat
        )

        controller._update_robot_state()
        controller._update_ik()
        task_target = controller.effector_task["right_hand"].T_world_frame
        np.testing.assert_allclose(
            task_target[:3, 3], controller.mj_data.mocap_pos[mocap_idx], atol=1e-9
        )
        self.assertTrue(np.isfinite(controller.placo_robot.state.q).all())

        # let the sim settle on the dragged target and check the TCP tracks it
        # (the lift joint carries both arms, settling takes a couple of seconds)
        for _ in range(2_500):
            controller._update_robot_state()
            controller._update_ik()
            controller._update_gripper_target()
            controller._update_mocap_target()
            controller._send_command()
            mujoco.mj_step(controller.mj_model, controller.mj_data)

        tcp_id = mujoco.mj_name2id(
            controller.mj_model, mujoco.mjtObj.mjOBJ_BODY, "rtcp_link"
        )
        tracking_error = np.linalg.norm(
            controller.mj_data.xpos[tcp_id] - controller.mj_data.mocap_pos[mocap_idx]
        )
        self.assertLess(tracking_error, 0.05)
        # the torso lift must stay locked while the arms move
        self.assertLess(abs(controller.mj_data.qpos[0]), 0.005)


if __name__ == "__main__":
    unittest.main()
