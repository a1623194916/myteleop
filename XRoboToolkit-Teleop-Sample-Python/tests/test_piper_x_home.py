"""Home-button isolation, bounded movement and clutch reanchoring without CAN."""

import unittest
from pathlib import Path

import numpy as np

from xrobotoolkit_teleop.hardware.piper_x_home import (
    JointHomeReturn, bounded_home_target, load_home_targets,
)


def home(side="right", target=None):
    return JointHomeReturn(side, np.full(6, 0.1) if target is None else target,
                           np.full(6, -1), np.full(6, 1), 0.01, timeout=1)


class HomeTests(unittest.TestCase):
    def test_b_and_y_route_only_to_selected_arm_and_held_buttons_do_not_repeat(self):
        left, right = home("left"), home("right")
        measured = np.zeros(6)
        # Release both buttons after startup, then B requests only the right arm.
        left.step(measured, False, False, 0)
        right.step(measured, False, False, 0)
        self.assertFalse(left.step(measured, False, False, 0.1)[1])
        command, handled = right.step(measured, True, False, 0.1)
        self.assertTrue(handled)
        np.testing.assert_allclose(command, 0.01)
        command, _ = right.step(command, True, False, 0.2)
        np.testing.assert_allclose(command, 0.02)
        self.assertTrue(right.returning)
        self.assertFalse(left.returning)
        self.assertTrue(left.step(measured, True, False, 0.2)[1])  # Y independently requests left.

    def test_held_startup_button_does_not_start_return(self):
        state = home()
        self.assertEqual(state.step(np.zeros(6), True, False, 0), (None, False))

    def test_completion_and_cancel_require_grip_release_before_teleop(self):
        state = home()
        state.step(np.zeros(6), False, True, 0)
        state.step(np.zeros(6), True, True, 0.1)
        self.assertEqual(state.step(np.full(6, 0.1), True, True, 0.2), (None, True))
        self.assertFalse(state.returning)
        self.assertEqual(state.step(np.full(6, 0.1), False, True, 0.3), (None, True))
        self.assertEqual(state.step(np.full(6, 0.1), False, False, 0.4), (None, False))
        state.step(np.zeros(6), True, False, 0.5)
        state.step(np.zeros(6), False, False, 0.6)
        self.assertEqual(state.step(np.zeros(6), True, True, 0.7), (None, True))
        self.assertFalse(state.returning)

    def test_feedback_failure_or_timeout_aborts_return(self):
        state = home()
        state.step(np.zeros(6), False, False, 0)
        state.step(np.zeros(6), True, False, 0.1)
        with self.assertRaisesRegex(RuntimeError, "tracking"):
            state.step(np.full(6, 0.5), True, False, 0.2)
        state = home()
        state.step(np.zeros(6), False, False, 0)
        state.step(np.zeros(6), True, False, 0.1)
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            state.step(np.zeros(6), True, False, 1.2)

    def test_capture_keeps_raw_values_but_return_target_respects_limits(self):
        state = JointHomeReturn("left", [0, 0, 0.05318, 0.627, 0.021, 0.096],
                                [-1, 0, -3, -1, -1, -2], [1, 3, 0, 1, 1, 2], 0.01)
        self.assertAlmostEqual(state.raw_target[2], 0.05318)
        self.assertEqual(state.target[2], 0)
        self.assertGreater(state.feedback_margin, 0.05318)
        with self.assertRaises(ValueError):
            home(target=np.full(6, 2.0))

    def test_saved_channel_mismatch_is_rejected_before_hardware_enable(self):
        path = Path(__file__).resolve().parents[2] / "configs/piper_x_home.json"
        targets = load_home_targets(path, {"left": "can0", "right": "can1"})
        self.assertEqual(targets["left"].shape, (6,))
        with self.assertRaisesRegex(ValueError, "belongs to"):
            load_home_targets(path, {"left": "can1"})

    def test_bounded_target_matches_what_the_home_button_reaches(self):
        # Limits read straight off the MuJoCo scene, in degrees: J1 +-150,
        # J2 [0, 180], J3 [-170, 0], J4/J5 +-89, J6 +-120. Both saved poses sit
        # outside J2/J3, so the bounded target is exactly the pose the teleop
        # startup log prints, and the standalone home tool reuses this function.
        lower = np.deg2rad([-150, 0, -170, -89, -89, -120])
        upper = np.deg2rad([150, 180, 0, 89, 89, 120])
        path = Path(__file__).resolve().parents[2] / "configs/piper_x_home.json"
        targets = load_home_targets(path, {"left": "can0", "right": "can1"})

        target, correction, margin = bounded_home_target("right", targets["right"], lower, upper)
        np.testing.assert_allclose(np.rad2deg(target).round(3),
                                   [-2.402, 0.0, 0.0, 36.154, 8.421, 6.469])
        self.assertGreater(correction, 0)
        self.assertGreaterEqual(margin, correction)

        left_target, _, _ = bounded_home_target("left", targets["left"], lower, upper)
        np.testing.assert_allclose(np.rad2deg(left_target).round(3),
                                   [0.0, 0.0, 0.0, 35.946, 1.225, 5.492])


if __name__ == "__main__":
    unittest.main()
