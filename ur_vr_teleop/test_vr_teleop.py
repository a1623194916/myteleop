import math
import unittest

import numpy as np

from vr_input import VRInput
from vr_teleop_math import (
    CartesianRateLimiter,
    ControllerJumpGuard,
    OneEuroPoseFilter,
    RelativePoseMapper,
    trigger_to_gripper,
)


class RelativePoseMapperTest(unittest.TestCase):
    def test_engagement_has_no_pose_jump(self):
        tcp = np.array([0.4, -0.1, 0.5, 0.0, math.pi, 0.0])
        hand = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
        mapper = RelativePoseMapper()
        mapper.engage(tcp, hand)
        np.testing.assert_allclose(mapper.target(hand), tcp, atol=1e-7)

    def test_positive_xr_x_maps_to_positive_base_y(self):
        mapper = RelativePoseMapper(scale=2.0)
        mapper.engage(np.zeros(6), np.array([0, 0, 0, 0, 0, 0, 1], dtype=float))
        target = mapper.target(np.array([0.1, 0, 0, 0, 0, 0, 1], dtype=float))
        np.testing.assert_allclose(target[:3], [0.0, 0.2, 0.0], atol=1e-8)

    def test_positive_xr_z_maps_to_positive_base_x(self):
        mapper = RelativePoseMapper(scale=2.0)
        mapper.engage(np.zeros(6), np.array([0, 0, 0, 0, 0, 0, 1], dtype=float))
        target = mapper.target(np.array([0, 0, 0.1, 0, 0, 0, 1], dtype=float))
        np.testing.assert_allclose(target[:3], [0.2, 0.0, 0.0], atol=1e-8)

    def test_signed_translation_axes_with_nonzero_anchor_and_rotated_tcp(self):
        tcp = np.array([0.4, -0.1, 0.5, 0.0, math.pi, 0.0])
        hand = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
        expected_axes = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
        for axis in range(3):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    mapper = RelativePoseMapper(scale=2.0)
                    mapper.engage(tcp, hand)
                    moved = hand.copy()
                    moved[axis] += sign * 0.01
                    target = mapper.target(moved)
                    np.testing.assert_allclose(
                        target[:3], tcp[:3] + sign * 0.02 * expected_axes[axis],
                        atol=1e-8,
                    )
                    np.testing.assert_allclose(target[3:], tcp[3:], atol=1e-7)

    def test_translation_sign_change_preserves_rotation_mapping(self):
        expected_axes = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        for axis in range(3):
            with self.subTest(axis=axis):
                mapper = RelativePoseMapper()
                mapper.engage(np.zeros(6), np.array([0, 0, 0, 0, 0, 0, 1]))
                hand = np.array([0, 0, 0, 0, 0, 0, math.cos(0.05)])
                hand[3 + axis] = math.sin(0.05)
                target = mapper.target(hand)
                np.testing.assert_allclose(target[:3], 0.0, atol=1e-8)
                np.testing.assert_allclose(target[3:], 0.1 * expected_axes[axis], atol=1e-8)

    def test_translation_is_bounded_from_anchor(self):
        mapper = RelativePoseMapper(max_translation=0.1)
        mapper.engage(np.zeros(6), np.array([0, 0, 0, 0, 0, 0, 1], dtype=float))
        target = mapper.target(np.array([10, 0, 0, 0, 0, 0, 1], dtype=float))
        self.assertAlmostEqual(np.linalg.norm(target[:3]), 0.1)

    def test_invalid_controller_anchor_is_rejected(self):
        mapper = RelativePoseMapper()
        with self.assertRaises(ValueError):
            mapper.engage(np.zeros(6), np.array([np.nan, 0, 0, 0, 0, 0, 1]))

    def test_rate_limiter_uses_previous_command(self):
        limiter = CartesianRateLimiter(max_linear_speed=0.2, max_angular_speed=1.0)
        limiter.reset(np.zeros(6))
        first = limiter.advance(np.array([1, 0, 0, 0, 0, 1], dtype=float), 0.1)
        second = limiter.advance(np.array([1, 0, 0, 0, 0, 1], dtype=float), 0.1)
        self.assertAlmostEqual(first[0], 0.02)
        self.assertAlmostEqual(second[0], 0.04)
        self.assertAlmostEqual(np.linalg.norm(first[3:]), 0.1, places=6)

    def test_one_euro_filter_smooths_position_and_rotation(self):
        pose_filter = OneEuroPoseFilter(min_cutoff=1.5, beta=0.25)
        pose_filter.reset(np.zeros(6))
        target = np.array([1.0, 0, 0, 0, 0, 1.0])
        filtered = pose_filter.advance(target, 0.02)
        self.assertGreater(filtered[0], 0.0)
        self.assertLess(filtered[0], 1.0)
        self.assertGreater(np.linalg.norm(filtered[3:]), 0.0)
        self.assertLess(np.linalg.norm(filtered[3:]), 1.0)

    def test_jump_guard_clamps_jump_and_rejects_teleport(self):
        guard = ControllerJumpGuard(jump_threshold=0.12, teleport_threshold=0.30)
        start = np.array([0, 0, 0, 0, 0, 0, 1], dtype=float)
        guard.reset(start)
        clamped = guard.filter(np.array([0.2, 0, 0, 0, 0, 0, 1], dtype=float))
        self.assertAlmostEqual(clamped[0], 0.12)
        rejected = guard.filter(np.array([0.5, 0, 0, 0, 0, 0, 1], dtype=float))
        self.assertIsNone(rejected)

    def test_trigger_maps_open_to_closed(self):
        self.assertAlmostEqual(trigger_to_gripper(0.0), 1.71)
        self.assertAlmostEqual(trigger_to_gripper(1.0), 0.0)
        self.assertAlmostEqual(trigger_to_gripper(0.5), 0.855)

    def test_disconnected_headset_zero_quaternion_is_rejected(self):
        controller = {
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 0.0],
        }
        with self.assertRaises(ValueError):
            VRInput._validate({"left_controller": controller, "right_controller": controller})

    def test_unused_left_controller_may_be_disconnected(self):
        disconnected = {
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 0.0],
        }
        connected = {
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        }
        VRInput._validate(
            {"left_controller": disconnected, "right_controller": connected},
            ("right",),
        )


if __name__ == "__main__":
    unittest.main()
