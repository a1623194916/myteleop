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

    def test_base_and_torso_presented_as_meshes_with_box_collision(self):
        model = mujoco.MjModel.from_xml_path(str(TELEOP_MJCF))

        # visual: the real (decimated) chassis and torso meshes
        for geom_name, mesh_name in (
            ("base_visual", "base_LINK"),
            ("torso_visual", "up_down_link"),
        ):
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            self.assertNotEqual(geom_id, -1)
            self.assertEqual(model.geom_type[geom_id], mujoco.mjtGeom.mjGEOM_MESH)
            mesh_id = model.geom_dataid[geom_id]
            self.assertEqual(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id), mesh_name
            )
            face_count = (
                model.mesh_faceadr[mesh_id + 1] - model.mesh_faceadr[mesh_id]
                if mesh_id + 1 < model.nmesh
                else model.nface - model.mesh_faceadr[mesh_id]
            )
            self.assertLessEqual(face_count, 200_000)

        # collision: the original boxes must stay (the full-mesh convex hull
        # of the base would swallow the torso) but render invisible (alpha 0)
        # now that the real meshes are drawn
        for geom_name in ("base_collision", "torso_collision"):
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            self.assertEqual(model.geom_type[geom_id], mujoco.mjtGeom.mjGEOM_BOX)
            self.assertEqual(model.geom_rgba[geom_id][3], 0.0)

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
