import unittest
from pathlib import Path

import mujoco
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
NG01_ROOT = REPO_ROOT.parent / "NG01_v4"
TELEOP_MJCF = NG01_ROOT / "NG01_mjcf" / "NG01_teleop.xml"


class Ng01MujocoModelTest(unittest.TestCase):
    def test_teleop_model_loads_with_expected_joints_and_actuators(self):
        model = mujoco.MjModel.from_xml_path(str(TELEOP_MJCF))

        self.assertEqual(model.nq, 17)
        self.assertEqual(model.nu, 17)

        expected_joints = {
            "up_down_joint",
            *(f"ljoint{i}" for i in range(1, 8)),
            *(f"rjoint{i}" for i in range(1, 8)),
            "lgripper_finger_joint",
            "rgripper_finger_joint",
        }
        actual_joints = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            for joint_id in range(model.njnt)
        }
        self.assertEqual(actual_joints, expected_joints)

        left_gripper_geom = model.geom("lgripper_link_visual")
        right_gripper_geom = model.geom("rgripper_link_visual")
        self.assertNotEqual(left_gripper_geom.dataid, right_gripper_geom.dataid)

    def test_teleop_model_has_tcp_and_mocap_target_bodies(self):
        model = mujoco.MjModel.from_xml_path(str(TELEOP_MJCF))

        for body_name in ("ltcp_link", "rtcp_link", "l_hand_vis_target", "r_hand_vis_target"):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            self.assertNotEqual(body_id, -1, f"body '{body_name}' missing from the model")

        for body_name in ("l_hand_vis_target", "r_hand_vis_target"):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            self.assertNotEqual(model.body_mocapid[body_id], -1)

    def test_teleop_model_steps_without_non_finite_state(self):
        model = mujoco.MjModel.from_xml_path(str(TELEOP_MJCF))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)

        for _ in range(1_000):
            mujoco.mj_step(model, data)

        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())
        np.testing.assert_allclose(data.qpos, model.key("home").qpos, atol=1e-3)


if __name__ == "__main__":
    unittest.main()
