import math
import time
import unittest

import numpy as np

from xrobotoolkit_teleop.hardware.fr3c_control_utils import (
    AbsoluteDeadlinePacer,
    JointCommandTrajectory,
    OneEuroFilter,
    PoseDeltaFilter,
    ema_alpha_for_dt,
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
    TAU = 0.01 / math.log(2.0)  # alpha = 0.5 for a 10 ms tick

    def test_regular_ticks_reproduce_classic_ema(self):
        trajectory = JointCommandTrajectory(tau_s=self.TAU, max_step_rad=1.0)
        trajectory.reset(np.zeros(1), now_s=0.0)

        self.assertAlmostEqual(trajectory.advance(np.ones(1), now_s=0.01)[0], 0.5)
        self.assertAlmostEqual(trajectory.advance(np.ones(1), now_s=0.02)[0], 0.75)

    def test_irregular_ticks_match_regular_ticks_exactly(self):
        """The core jitter-immunity property: the command depends only on the
        elapsed wall time, never on how ticks happened to be spaced."""
        target = np.ones(1)
        total_s = 0.1

        regular = JointCommandTrajectory(tau_s=0.02, max_step_rad=1.0)
        regular.reset(np.zeros(1), now_s=0.0)
        for index in range(1, 11):
            regular.advance(target, now_s=0.01 * index)

        jittered = JointCommandTrajectory(tau_s=0.02, max_step_rad=1.0)
        jittered.reset(np.zeros(1), now_s=0.0)
        schedule = [0.001, 0.002, 0.003, 0.030, 0.065, 0.1]  # ends at 0.1 s too
        for stamp in schedule:
            jittered.advance(target, now_s=stamp)

        np.testing.assert_allclose(jittered.advance(target, now_s=0.1), regular.advance(target, now_s=0.1))

    def test_first_advance_uses_default_dt(self):
        trajectory = JointCommandTrajectory(tau_s=self.TAU, max_step_rad=1.0, default_dt_s=0.01)
        trajectory.reset(np.zeros(1), now_s=None)

        self.assertAlmostEqual(trajectory.advance(np.ones(1))[0], 0.5)

    def test_zero_elapsed_time_does_not_move(self):
        trajectory = JointCommandTrajectory(tau_s=0.01, max_step_rad=1.0)
        trajectory.reset(np.zeros(1), now_s=0.0)

        output = trajectory.advance(np.ones(1), now_s=0.0)

        np.testing.assert_allclose(output, np.zeros(1))

    def test_caps_each_joint_step(self):
        trajectory = JointCommandTrajectory(tau_s=1e-6, max_step_rad=0.1)
        trajectory.reset(np.array([0.0, 0.0]), now_s=0.0)

        np.testing.assert_allclose(
            trajectory.advance(np.array([1.0, -1.0]), now_s=0.05),
            np.array([0.1, -0.1]),
        )

    def test_advance_requires_reset(self):
        trajectory = JointCommandTrajectory(tau_s=0.01, max_step_rad=0.1)

        with self.assertRaisesRegex(RuntimeError, "reset"):
            trajectory.advance(np.zeros(1), now_s=0.0)

    def test_reset_does_not_inherit_previous_clock(self):
        trajectory = JointCommandTrajectory(tau_s=self.TAU, max_step_rad=1.0, default_dt_s=0.01)
        trajectory.reset(np.zeros(1), now_s=100.0)
        trajectory.advance(np.ones(1), now_s=100.0)
        trajectory.reset(np.zeros(1), now_s=0.0)

        self.assertAlmostEqual(trajectory.advance(np.ones(1), now_s=0.01)[0], 0.5)


class AbsoluteDeadlinePacerTests(unittest.TestCase):
    def test_early_tick_waits_for_deadline_and_reports_no_lateness(self):
        pacer = AbsoluteDeadlinePacer(0.01)
        pacer.reset(now_s=1.0)

        self.assertEqual(pacer.tick(now_s=1.0), 0.0)
        # Deadline is now 1.01: waking at 1.0095 is early -> no lateness.
        self.assertEqual(pacer.tick(now_s=1.0095), 0.0)

    def test_late_tick_reports_lateness_and_schedule_stays_on_grid(self):
        pacer = AbsoluteDeadlinePacer(0.01)
        pacer.reset(now_s=1.0)

        # Woke 4 ms past the first deadline (grid: 1.000, 1.010, 1.020, ...).
        self.assertAlmostEqual(pacer.tick(now_s=1.004), 0.004)
        self.assertAlmostEqual(pacer.next_deadline_s, 1.010)
        # A call arriving before the next deadline is early: it fires at once
        # (no sleep), so one slow tick never pushes the following sends later.
        self.assertEqual(pacer.tick(now_s=1.0045), 0.0)
        self.assertAlmostEqual(pacer.next_deadline_s, 1.020)
        self.assertEqual(pacer.tick(now_s=1.0145), 0.0)
        self.assertAlmostEqual(pacer.next_deadline_s, 1.030)

    def test_resyncs_when_more_than_one_period_behind(self):
        pacer = AbsoluteDeadlinePacer(0.01)
        pacer.reset(now_s=1.0)
        pacer.tick(now_s=1.0)

        # 3 periods late: must not burst stale ticks, schedule restarts now.
        self.assertAlmostEqual(pacer.tick(now_s=1.031), 0.0)
        # Deadline is now 1.041.
        self.assertAlmostEqual(pacer.tick(now_s=1.035), 0.0)

    def test_periods_advance_exactly(self):
        pacer = AbsoluteDeadlinePacer(0.008)
        pacer.reset(now_s=0.0)

        pacer.tick(now_s=0.0)
        pacer.tick(now_s=0.008)
        self.assertAlmostEqual(pacer.tick(now_s=0.016), 0.0)


class OneEuroFilterTests(unittest.TestCase):
    def test_first_sample_is_passed_through(self):
        filter_ = OneEuroFilter(3, min_cutoff_hz=1.0, beta=0.01)

        np.testing.assert_allclose(filter_.filter(np.array([1.0, 2.0, 3.0]), 0.0), [1.0, 2.0, 3.0])

    def test_zero_dt_holds_output(self):
        filter_ = OneEuroFilter(1, min_cutoff_hz=1.0, beta=0.01)
        filter_.filter(np.array([0.0]), 0.0)

        np.testing.assert_allclose(filter_.filter(np.array([5.0]), 0.0), [0.0])

    def test_rejects_wrong_shape(self):
        filter_ = OneEuroFilter(3, min_cutoff_hz=1.0, beta=0.01)

        with self.assertRaisesRegex(ValueError, "shape"):
            filter_.filter(np.zeros(2), 0.0)

    def test_rest_noise_is_attenuated(self):
        filter_ = OneEuroFilter(1, min_cutoff_hz=2.0, beta=0.02)
        filter_.filter(np.array([0.0]), 0.0)

        rng = np.random.default_rng(3)
        noise = 0.001 * np.cos(np.arange(1, 201) * 0.7)
        peaks = [
            abs(filter_.filter(np.array([value]), 0.01 * index)[0])
            for index, value in enumerate(noise, start=1)
        ]

        self.assertLess(max(peaks), 0.5 * 0.001)

    def test_sustained_motion_is_tracked(self):
        filter_ = OneEuroFilter(1, min_cutoff_hz=2.0, beta=0.02)
        filter_.filter(np.array([0.0]), 0.0)

        output = filter_.filter(np.array([0.1]), 0.05)

        self.assertGreater(output[0], 0.03)


class PoseDeltaFilterTests(unittest.TestCase):
    def test_sub_threshold_position_jitter_does_not_move_target(self):
        pose_filter = PoseDeltaFilter(
            min_cutoff_hz=2.0,
            beta=0.02,
            position_deadband_m=0.002,
            rotation_deadband_rad=np.deg2rad(0.5),
        )

        for index, x in enumerate([0.001, -0.001, 0.0015, -0.0015], start=1):
            position, rotation = pose_filter.update(
                np.array([x, 0.0, 0.0]),
                np.zeros(3),
                timestamp_ns=index * 10**9,
            )
            np.testing.assert_allclose(position, np.zeros(3))
            np.testing.assert_allclose(rotation, np.zeros(3))

    def test_slow_consistent_motion_accumulates_through_deadband(self):
        pose_filter = PoseDeltaFilter(
            min_cutoff_hz=2.0,
            beta=0.02,
            position_deadband_m=0.002,
            rotation_deadband_rad=np.deg2rad(0.5),
        )

        outputs = [
            pose_filter.update(
                np.array([x, 0.0, 0.0]),
                np.zeros(3),
                timestamp_ns=index * 10**9,
            )[0][0]
            for index, x in enumerate([0.001, 0.002, 0.003, 0.004, 0.005], start=1)
        ]

        self.assertEqual(outputs[0], 0.0)
        self.assertGreater(outputs[-1], 0.002)

    def test_duplicate_timestamp_does_not_refilter_same_xr_sample(self):
        pose_filter = PoseDeltaFilter(
            min_cutoff_hz=2.0,
            beta=0.02,
            position_deadband_m=0.0,
            rotation_deadband_rad=0.0,
        )

        first, _ = pose_filter.update(
            np.array([0.01, 0.0, 0.0]),
            np.zeros(3),
            timestamp_ns=123 * 10**6,
        )
        duplicate, _ = pose_filter.update(
            np.array([0.01, 0.0, 0.0]),
            np.zeros(3),
            timestamp_ns=123 * 10**6,
        )

        np.testing.assert_allclose(duplicate, first)

    def test_sub_threshold_rotation_jitter_does_not_move_target(self):
        pose_filter = PoseDeltaFilter(
            min_cutoff_hz=2.0,
            beta=0.02,
            position_deadband_m=0.0,
            rotation_deadband_rad=np.deg2rad(0.5),
        )

        _, rotation = pose_filter.update(
            np.zeros(3),
            np.array([0.0, 0.0, np.deg2rad(0.3)]),
            timestamp_ns=10**9,
        )

        np.testing.assert_allclose(rotation, np.zeros(3))

    def test_crossing_deadband_has_no_full_threshold_jump(self):
        pose_filter = PoseDeltaFilter(
            min_cutoff_hz=2.0,
            beta=0.02,
            position_deadband_m=0.005,
            rotation_deadband_rad=0.0,
        )

        below, _ = pose_filter.update(
            np.array([0.0049, 0.0, 0.0]),
            np.zeros(3),
            timestamp_ns=10**9,
        )
        above, _ = pose_filter.update(
            np.array([0.0051, 0.0, 0.0]),
            np.zeros(3),
            timestamp_ns=2 * 10**9,
        )

        np.testing.assert_allclose(below, np.zeros(3))
        self.assertGreater(above[0], 0.0)
        self.assertLess(above[0], 0.0002)


class EmaAlphaForDtTests(unittest.TestCase):
    def test_time_constant_semantics(self):
        self.assertAlmostEqual(ema_alpha_for_dt(0.01, 0.01 / math.log(2.0)), 0.5)

    def test_zero_dt_returns_zero(self):
        self.assertEqual(ema_alpha_for_dt(0.0, 0.01), 0.0)

    def test_zero_tau_returns_one(self):
        self.assertEqual(ema_alpha_for_dt(0.01, 0.0), 1.0)


class MonotonicDefaultSmokeTest(unittest.TestCase):
    def test_trajectory_advance_without_explicit_clock(self):
        trajectory = JointCommandTrajectory(tau_s=0.01, max_step_rad=1.0)
        trajectory.reset(np.zeros(1))

        value = trajectory.advance(np.ones(1))

        self.assertTrue(math.isfinite(value[0]))


if __name__ == "__main__":
    unittest.main()
