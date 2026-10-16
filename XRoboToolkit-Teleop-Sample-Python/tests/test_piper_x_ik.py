"""Reachability, continuity and state isolation on the real Piper-X model."""

import unittest

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from xrobotoolkit_teleop.hardware.piper_x_ik import ContinuousPoseIK
from xrobotoolkit_teleop.simulation.piper_x_model import build_scene


class ContinuousPoseIKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, scene = build_scene()
        cls.model = mujoco.MjModel.from_xml_path(str(scene))

    def setUp(self):
        self.ik = ContinuousPoseIK(self.model)
        self.data = mujoco.MjData(self.model)

    def pose(self, q):
        self.data.qpos[self.ik.qadr] = q
        mujoco.mj_forward(self.model, self.data)
        pose = np.eye(4)
        pose[:3, :3] = self.data.xmat[self.ik.body].reshape(3, 3)
        pose[:3, 3] = self.data.xpos[self.ik.body]
        return pose

    def test_reachable_poses_from_bent_and_recorded_home(self):
        for seed in (np.deg2rad([0, 60, -80, 20, 20, 10]),
                     np.deg2rad([-2.402, 0, 0, 36.154, 8.421, 6.469])):
            self.ik.reset(seed)
            target_q = seed + np.deg2rad([3, 4, -5, 3, -3, 4])
            target = self.pose(target_q)
            result = self.ik.solve(target, seed)
            actual = self.pose(result.joints)
            self.assertTrue(result.reached, result)
            self.assertLess(np.linalg.norm(actual[:3, 3] - target[:3, 3]), 0.002)
            self.assertLess(Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).magnitude(), 0.02)

    def test_smooth_path_with_lagging_feedback_and_no_branch_jumps(self):
        seed = np.deg2rad([0, 60, -80, 20, 20, 10])
        measured, previous = seed.copy(), seed.copy()
        for phase in np.linspace(0, 2 * np.pi, 121):
            target_q = seed + np.deg2rad([5, 6, -8, 5, 6, 10]) * np.sin(phase)
            result = self.ik.solve(self.pose(target_q), measured)
            self.assertTrue(result.reached, result)
            self.assertLess(result.position_error_m, 0.002)
            self.assertLess(np.max(abs(result.joints - previous)), 0.035)
            measured += 0.25 * (result.joints - measured)
            previous = result.joints

    def test_unreachable_goal_is_flagged_and_respects_tracking_envelope(self):
        measured = np.deg2rad([0, 60, -80, 20, 20, 10])
        target = self.pose(measured)
        target[0, 3] += 2
        before = self.data.qpos.copy()
        result = self.ik.solve(target, measured)
        self.assertFalse(result.reached)
        self.assertGreater(result.position_error_m, 1)
        self.assertTrue(np.all(result.joints >= self.ik.lower))
        self.assertTrue(np.all(result.joints <= self.ik.upper))
        self.assertLessEqual(np.max(abs(result.joints - measured)), 0.5500001)
        np.testing.assert_array_equal(self.data.qpos, before)

    def test_rotation_log_jacobian_matches_world_rotation_perturbations(self):
        vector = np.array([0.4, -0.6, 0.8])
        rotation = Rotation.from_rotvec(vector)
        numerical = np.column_stack([
            ((Rotation.from_rotvec(np.eye(3)[i] * 1e-6) * rotation).as_rotvec() -
             (Rotation.from_rotvec(-np.eye(3)[i] * 1e-6) * rotation).as_rotvec()) / 2e-6
            for i in range(3)
        ])
        np.testing.assert_allclose(self.ik._log_jacobian(vector), numerical, atol=1e-7)

    def test_invalid_inputs_fail_before_optimization(self):
        for target, measured in ((np.full((4, 4), np.nan), np.zeros(6)),
                                 (np.eye(4), np.zeros(5))):
            with self.assertRaises(ValueError):
                self.ik.solve(target, measured)


if __name__ == "__main__":
    unittest.main()
