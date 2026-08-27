import unittest
from pathlib import Path

from scripts.simulation.teleop_dual_ng01_mujoco import (
    _make_fake_input_updater,
    _update_fake_inputs,
    build_controller,
    build_manipulator_config,
)
from xrobotoolkit_teleop.common.fake_xr_client import FakeXrClient


REPO_ROOT = Path(__file__).resolve().parents[1]
NG01_ROOT = REPO_ROOT.parent / "NG01_v4"


class Ng01TeleopEntryPointTest(unittest.TestCase):
    def test_configuration_controls_only_two_tcp_frames_and_grippers(self):
        config = build_manipulator_config(control_mode="position")

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

    def test_fake_input_alternates_fully_open_and_closed_grippers(self):
        client = FakeXrClient()

        _update_fake_inputs(client, 0.5)
        self.assertEqual(client.get_key_value_by_name("left_trigger"), 1.0)
        self.assertEqual(client.get_key_value_by_name("right_trigger"), 0.0)

        _update_fake_inputs(client, 2.5)
        self.assertEqual(client.get_key_value_by_name("left_trigger"), 0.0)
        self.assertEqual(client.get_key_value_by_name("right_trigger"), 1.0)

    def test_ng01_camera_targets_torso_from_front(self):
        controller = build_controller(
            input_source="fake",
            control_mode="position",
            ng01_root=NG01_ROOT,
        )

        self.assertEqual(controller.viewer_camera["azimuth"], 90)
        self.assertEqual(controller.viewer_camera["elevation"], -10)
        self.assertEqual(controller.viewer_camera["lookat"], [0.0, 0.0, 1.15])

    def test_fake_viewer_updater_advances_inputs_from_elapsed_time(self):
        client = FakeXrClient()
        update = _make_fake_input_updater(client, clock=lambda: 12.5)

        update()
        self.assertEqual(client.get_key_value_by_name("left_trigger"), 1.0)
        self.assertEqual(client.get_key_value_by_name("right_trigger"), 0.0)

    def test_fake_input_controller_can_run_headlessly(self):
        controller = build_controller(
            input_source="fake",
            control_mode="position",
            ng01_root=NG01_ROOT,
        )

        self.assertIsInstance(controller.xr_client, FakeXrClient)
        self.assertIsNotNone(controller.pre_step_callback)
        controller.step()
        self.assertEqual(controller.xr_client.get_key_value_by_name("left_trigger"), 1.0)
        self.assertEqual(controller.gripper_pos_target["left_hand"]["lgripper_finger_joint"], 1.0472)
        for _ in range(9):
            controller.step()


if __name__ == "__main__":
    unittest.main()
