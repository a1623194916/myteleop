import unittest
from pathlib import Path

import numpy as np

from scripts.simulation.teleop_dual_ng01_mujoco import (
    AlignAndStartTeleopController,
    build_manipulator_config,
)
from xrobotoolkit_teleop.common.fake_xr_client import FakeXrClient
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD


REPO_ROOT = Path(__file__).resolve().parents[1]
NG01_ROOT = REPO_ROOT.parent / "NG01_v4"
TELEOP_MJCF = NG01_ROOT / "NG01_mjcf" / "NG01_teleop.xml"
TELEOP_URDF = NG01_ROOT / "urdf" / "NG01_teleop.urdf"


def make_controller():
    client = FakeXrClient()
    controller = AlignAndStartTeleopController(
        xml_path=str(TELEOP_MJCF),
        robot_urdf_path=str(TELEOP_URDF),
        manipulator_config=build_manipulator_config("position"),
        xr_client=client,
    )
    return controller, client


class Ng01AlignmentFlowTest(unittest.TestCase):
    def test_vis_targets_are_bound_to_mocap_bodies(self):
        controller, _ = make_controller()

        self.assertEqual(controller.mj_model.nmocap, 2)
        self.assertNotEqual(controller.target_mocap_idx["left_hand"], -1)
        self.assertNotEqual(controller.target_mocap_idx["right_hand"], -1)

    def test_task_holds_home_pose_until_grip_arms_it(self):
        controller, _ = make_controller()
        home_xyz = controller._home_ee_pose["left_hand"][0]

        controller.step()

        self.assertFalse(controller.is_armed("left_hand"))
        np.testing.assert_allclose(
            controller.effector_task["left_hand"].target_world, home_xyz, atol=1e-9
        )

    def test_invalid_pose_does_not_arm(self):
        controller, client = make_controller()
        client.set_key_value("left_grip", 1.0)
        client.set_pose("left_controller", np.zeros(7))

        controller.step()

        self.assertFalse(controller.is_armed("left_hand"))

    def test_grip_arms_and_tracks_scaled_relative_motion(self):
        controller, client = make_controller()
        home_xyz = controller._home_ee_pose["left_hand"][0]

        client.set_key_value("left_grip", 1.0)
        controller.step()
        self.assertTrue(controller.is_armed("left_hand"))
        np.testing.assert_allclose(
            controller.effector_task["left_hand"].target_world, home_xyz, atol=1e-9
        )

        moved = np.array([0.10, 0.05, -0.02, 0.0, 0.0, 0.0, 1.0])
        client.set_pose("left_controller", moved)
        controller.step()

        expected = home_xyz + controller.scale_factor * (R_HEADSET_TO_WORLD @ moved[:3])
        np.testing.assert_allclose(
            controller.effector_task["left_hand"].target_world, expected, atol=1e-6
        )
        np.testing.assert_allclose(
            controller.mj_data.mocap_pos[controller.target_mocap_idx["left_hand"]],
            expected,
            atol=1e-6,
        )

    def test_release_button_disarms_and_stays_disarmed_while_grip_held(self):
        controller, client = make_controller()
        home_xyz = controller._home_ee_pose["left_hand"][0]

        client.set_key_value("left_grip", 1.0)
        controller.step()
        self.assertTrue(controller.is_armed("left_hand"))

        client.set_button_state("X", True)
        controller.step()
        self.assertFalse(controller.is_armed("left_hand"))
        np.testing.assert_allclose(
            controller.effector_task["left_hand"].target_world, home_xyz, atol=1e-9
        )

        controller.step()
        self.assertFalse(controller.is_armed("left_hand"))

        client.set_button_state("X", False)
        client.set_key_value("left_grip", 0.0)
        controller.step()
        client.set_key_value("left_grip", 1.0)
        controller.step()
        self.assertTrue(controller.is_armed("left_hand"))


if __name__ == "__main__":
    unittest.main()
