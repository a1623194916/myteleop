"""Tracking protection accepts motor response time without disabling faults."""
import unittest
from unittest.mock import patch

import numpy as np

from xrobotoolkit_teleop.hardware.piper_x import JointFollower, JointTrackingError


class TrackingTests(unittest.TestCase):
    def follower(self):
        return JointFollower([-1] * 6, [1] * 6, np.inf, tracking_timeout=0.5)

    def test_short_lag_is_allowed_and_recovery_resets_timer(self):
        state = self.follower()
        actual, target = np.zeros(6), np.full(6, 0.3)
        state.next(actual, target, True)
        with patch('xrobotoolkit_teleop.hardware.piper_x.time.monotonic', side_effect=[1.0, 1.4, 3.0]):
            np.testing.assert_allclose(state.next(actual, target, True), target)
            state.next(actual, target, True)
            state.next(target, target, True)
            state.next(actual, target, True)  # A new lag interval, not a 2 s fault.

    def test_persistent_lag_and_large_targets_still_fault(self):
        state = self.follower()
        actual, target = np.zeros(6), np.full(6, 0.3)
        state.next(actual, target, True)
        with patch('xrobotoolkit_teleop.hardware.piper_x.time.monotonic', side_effect=[1.0, 1.51]):
            state.next(actual, target, True)
            with self.assertRaises(JointTrackingError):
                state.next(actual, target, True)
        state.next(actual, target, False)
        self.assertIsNone(state.error_started)
        with self.assertRaises(JointTrackingError):
            state.next(actual, np.full(6, 0.8), True)
        self.assertIsNone(state.last_command)  # Never send the rejected large target.


if __name__ == '__main__':
    unittest.main()
