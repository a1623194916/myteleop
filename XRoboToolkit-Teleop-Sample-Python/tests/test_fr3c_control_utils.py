import unittest

import numpy as np

from xrobotoolkit_teleop.hardware.fr3c_control_utils import (
    JointCommandTrajectory,
    resolve_controller_side,
)


class ControllerSideTests(unittest.TestCase):
    def test_auto_side_uses_left_controller_for_robot_22(self):
        self.assertEqual(resolve_controller_side("192.168.5.22", "auto"), "left")

    def test_auto_side_uses_right_controller_for_robot_23(self):
        self.assertEqual(resolve_controller_side("192.168.5.23", "auto"), "right")

    def test_unknown_ip_preserves_right_controller_default(self):
        self.assertEqual(resolve_controller_side("192.168.58.2", "auto"), "right")

    def test_explicit_side_overrides_ip_mapping(self):
        self.assertEqual(resolve_controller_side("192.168.5.22", "right"), "right")

    def test_invalid_side_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "auto, left, right"):
            resolve_controller_side("192.168.5.22", "centre")


class JointCommandTrajectoryTests(unittest.TestCase):
    def test_trajectory_advances_from_previous_command(self):
        trajectory = JointCommandTrajectory(alpha=0.5, max_step_rad=1.0)
        trajectory.reset(np.zeros(1))

        self.assertAlmostEqual(trajectory.advance(np.ones(1))[0], 0.5)
        self.assertAlmostEqual(trajectory.advance(np.ones(1))[0], 0.75)

    def test_trajectory_caps_each_joint_step(self):
        trajectory = JointCommandTrajectory(alpha=1.0, max_step_rad=0.1)
        trajectory.reset(np.array([0.0, 0.0]))

        np.testing.assert_allclose(
            trajectory.advance(np.array([1.0, -1.0])),
            np.array([0.1, -0.1]),
        )

    def test_advance_requires_reset(self):
        trajectory = JointCommandTrajectory(alpha=1.0, max_step_rad=0.1)

        with self.assertRaisesRegex(RuntimeError, "reset"):
            trajectory.advance(np.zeros(1))


if __name__ == "__main__":
    unittest.main()
