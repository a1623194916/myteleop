import unittest
from pathlib import Path

import mujoco
import numpy as np

from xrobotoolkit_teleop.hardware.fr3c_mujoco_mirror import Fr3cMujocoMirror, MirrorProcess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "fr3c_assets" / "scene_fr3c.xml"


class Fr3cMujocoMirrorTests(unittest.TestCase):
    def test_set_joint_positions_uses_named_joint_addresses(self):
        mirror = Fr3cMujocoMirror(str(SCENE_XML))
        expected = np.linspace(-0.5, 0.5, 6)

        mirror.set_joint_positions(expected)

        for index, joint_name in enumerate(mirror.joint_names):
            joint_id = mujoco.mj_name2id(
                mirror.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )
            qpos_address = mirror.model.jnt_qposadr[joint_id]
            self.assertAlmostEqual(mirror.data.qpos[qpos_address], expected[index])

    def test_set_joint_positions_rejects_wrong_joint_count(self):
        mirror = Fr3cMujocoMirror(str(SCENE_XML))

        with self.assertRaisesRegex(ValueError, "6 joint positions"):
            mirror.set_joint_positions(np.zeros(5))


class MirrorProcessTests(unittest.TestCase):
    def test_publish_writes_measured_joints_to_shared_buffer(self):
        mirror_process = MirrorProcess(str(SCENE_XML))

        try:
            mirror_process.publish(np.linspace(-0.2, 0.2, 6))
            with mirror_process.shared_q.get_lock():
                published = np.array(mirror_process.shared_q)
        finally:
            mirror_process.stop()

        np.testing.assert_allclose(published, np.linspace(-0.2, 0.2, 6))
        self.assertFalse(mirror_process.is_running())


if __name__ == "__main__":
    unittest.main()
