import unittest
from pathlib import Path

import mujoco

from xrobotoolkit_teleop.common.fake_xr_client import FakeXrClient
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController


REPO_ROOT = Path(__file__).resolve().parents[1]
NG01_ROOT = REPO_ROOT.parent / "NG01_v4"
TELEOP_MJCF = NG01_ROOT / "NG01_mjcf" / "NG01_teleop.xml"
TELEOP_URDF = NG01_ROOT / "urdf" / "NG01_teleop.urdf"


def ng01_config():
    return {
        "left_hand": {
            "link_name": "ltcp_link",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "control_mode": "position",
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "left_trigger",
                "joint_names": ["lgripper_finger_joint"],
                "open_pos": [0.0],
                "close_pos": [1.0472],
            },
        },
        "right_hand": {
            "link_name": "rtcp_link",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "control_mode": "position",
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "right_trigger",
                "joint_names": ["rgripper_finger_joint"],
                "open_pos": [0.0],
                "close_pos": [1.0472],
            },
        },
    }


class Ng01SimulationStepTest(unittest.TestCase):
    def test_one_step_maps_each_trigger_to_only_its_gripper(self):
        fake_client = FakeXrClient()
        fake_client.set_key_value("left_trigger", 0.25)
        fake_client.set_key_value("right_trigger", 0.75)
        controller = MujocoTeleopController(
            xml_path=str(TELEOP_MJCF),
            robot_urdf_path=str(TELEOP_URDF),
            manipulator_config=ng01_config(),
            xr_client=fake_client,
        )

        controller.step()

        left_actuator = mujoco.mj_name2id(
            controller.mj_model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            "lgripper_finger_joint_servo",
        )
        right_actuator = mujoco.mj_name2id(
            controller.mj_model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            "rgripper_finger_joint_servo",
        )
        self.assertAlmostEqual(controller.mj_data.ctrl[left_actuator], 1.0472 * 0.25)
        self.assertAlmostEqual(controller.mj_data.ctrl[right_actuator], 1.0472 * 0.75)


if __name__ == "__main__":
    unittest.main()
