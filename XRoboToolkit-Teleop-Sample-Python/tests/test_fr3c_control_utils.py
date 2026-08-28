import unittest

import numpy as np

from xrobotoolkit_teleop.hardware.fr3c_control_utils import (
    JointCommandTrajectory,
    PoseDeltaFilter,
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


class PoseDeltaFilterTests(unittest.TestCase):
    def test_sub_threshold_position_jitter_does_not_move_target(self):
        pose_filter = PoseDeltaFilter(
            alpha=0.5,
            position_deadband_m=0.002,
            rotation_deadband_rad=np.deg2rad(0.5),
        )

        for index, x in enumerate([0.001, -0.001, 0.0015, -0.0015], start=1):
            position, rotation = pose_filter.update(
                np.array([x, 0.0, 0.0]),
                np.zeros(3),
                timestamp_ns=index,
            )
            np.testing.assert_allclose(position, np.zeros(3))
            np.testing.assert_allclose(rotation, np.zeros(3))

    def test_slow_consistent_motion_accumulates_through_deadband(self):
        pose_filter = PoseDeltaFilter(
            alpha=0.5,
            position_deadband_m=0.002,
            rotation_deadband_rad=np.deg2rad(0.5),
        )

        outputs = [
            pose_filter.update(
                np.array([x, 0.0, 0.0]),
                np.zeros(3),
                timestamp_ns=index,
            )[0][0]
            for index, x in enumerate([0.001, 0.002, 0.003, 0.004, 0.005], start=1)
        ]

        self.assertEqual(outputs[0], 0.0)
        self.assertGreater(outputs[-1], 0.002)

    def test_duplicate_timestamp_does_not_refilter_same_xr_sample(self):
        pose_filter = PoseDeltaFilter(
            alpha=0.5,
            position_deadband_m=0.0,
            rotation_deadband_rad=0.0,
        )

        first, _ = pose_filter.update(
            np.array([0.01, 0.0, 0.0]),
            np.zeros(3),
            timestamp_ns=123,
        )
        duplicate, _ = pose_filter.update(
            np.array([0.01, 0.0, 0.0]),
            np.zeros(3),
            timestamp_ns=123,
        )

        np.testing.assert_allclose(duplicate, first)

    def test_sub_threshold_rotation_jitter_does_not_move_target(self):
        pose_filter = PoseDeltaFilter(
            alpha=1.0,
            position_deadband_m=0.0,
            rotation_deadband_rad=np.deg2rad(0.5),
        )

        _, rotation = pose_filter.update(
            np.zeros(3),
            np.array([0.0, 0.0, np.deg2rad(0.3)]),
            timestamp_ns=1,
        )

        np.testing.assert_allclose(rotation, np.zeros(3))

    def test_crossing_deadband_has_no_full_threshold_jump(self):
        pose_filter = PoseDeltaFilter(
            alpha=1.0,
            position_deadband_m=0.005,
            rotation_deadband_rad=0.0,
        )

        below, _ = pose_filter.update(
            np.array([0.0049, 0.0, 0.0]),
            np.zeros(3),
            timestamp_ns=1,
        )
        above, _ = pose_filter.update(
            np.array([0.0051, 0.0, 0.0]),
            np.zeros(3),
            timestamp_ns=2,
        )

        np.testing.assert_allclose(below, np.zeros(3))
        self.assertGreater(above[0], 0.0)
        self.assertLess(above[0], 0.0002)


if __name__ == "__main__":
    unittest.main()
