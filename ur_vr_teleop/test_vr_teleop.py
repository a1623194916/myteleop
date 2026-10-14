import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from camera_health import CameraHealth
from teleop_ur_vr import EpisodeTracker, RosDataPublisher, URHardware
from vr_input import VRInput
from vr_teleop_math import (
    CartesianDeadband,
    CartesianRateLimiter,
    ControllerJumpGuard,
    OneEuroPoseFilter,
    R_XR_TO_ROBOT,
    RelativePoseMapper,
    T_XR_TO_ROBOT,
    trigger_to_gripper,
)


class RelativePoseMapperTest(unittest.TestCase):
    def test_translation_and_rotation_use_the_same_right_handed_frame(self):
        np.testing.assert_allclose(R_XR_TO_ROBOT, T_XR_TO_ROBOT, atol=1e-12)
        np.testing.assert_allclose(R_XR_TO_ROBOT @ R_XR_TO_ROBOT.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(R_XR_TO_ROBOT)), 1.0)

    def test_engagement_has_no_pose_jump(self):
        tcp = np.array([0.4, -0.1, 0.5, 0.0, math.pi, 0.0])
        hand = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
        mapper = RelativePoseMapper()
        mapper.engage(tcp, hand)
        np.testing.assert_allclose(mapper.target(hand), tcp, atol=1e-7)

    def test_positive_xr_x_maps_to_positive_base_x(self):
        mapper = RelativePoseMapper(scale=2.0)
        mapper.engage(np.zeros(6), np.array([0, 0, 0, 0, 0, 0, 1], dtype=float))
        target = mapper.target(np.array([0.1, 0, 0, 0, 0, 0, 1], dtype=float))
        np.testing.assert_allclose(target[:3], [0.2, 0.0, 0.0], atol=1e-8)

    def test_negative_xr_z_maps_to_positive_base_y(self):
        mapper = RelativePoseMapper(scale=2.0)
        mapper.engage(np.zeros(6), np.array([0, 0, 0, 0, 0, 0, 1], dtype=float))
        target = mapper.target(np.array([0, 0, -0.1, 0, 0, 0, 1], dtype=float))
        np.testing.assert_allclose(target[:3], [0.0, 0.2, 0.0], atol=1e-8)

    def test_signed_translation_axes_with_nonzero_anchor_and_rotated_tcp(self):
        tcp = np.array([0.4, -0.1, 0.5, 0.0, math.pi, 0.0])
        hand = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
        expected_axes = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
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

    def test_scale_changes_translation_only(self):
        hand_anchor = np.array([0, 0, 0, 0, 0, 0, 1], dtype=float)
        moved = np.array(
            [0.1, 0, 0, math.sin(0.05), 0, 0, math.cos(0.05)],
            dtype=float,
        )
        normal = RelativePoseMapper(scale=1.0)
        amplified = RelativePoseMapper(scale=1.1)
        normal.engage(np.zeros(6), hand_anchor)
        amplified.engage(np.zeros(6), hand_anchor)

        normal_target = normal.target(moved)
        amplified_target = amplified.target(moved)
        np.testing.assert_allclose(
            amplified_target[:3], normal_target[:3] * 1.1, atol=1e-8
        )
        np.testing.assert_allclose(
            amplified_target[3:], normal_target[3:], atol=1e-8
        )

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

    def test_deadband_suppresses_small_translation_and_keeps_excess_motion(self):
        deadband = CartesianDeadband(position_threshold=0.002, rotation_threshold=0.0)
        deadband.reset(np.zeros(6))

        np.testing.assert_allclose(
            deadband.advance(np.array([0.001, 0, 0, 0, 0, 0], dtype=float)),
            np.zeros(6),
            atol=1e-12,
        )
        moved = deadband.advance(np.array([0.005, 0, 0, 0, 0, 0], dtype=float))
        np.testing.assert_allclose(moved[:3], [0.003, 0, 0], atol=1e-12)

    def test_deadband_suppresses_small_rotation_and_keeps_excess_motion(self):
        threshold = math.radians(0.5)
        deadband = CartesianDeadband(position_threshold=0.0, rotation_threshold=threshold)
        deadband.reset(np.zeros(6))

        small = deadband.advance(
            np.array([0, 0, 0, 0, 0, math.radians(0.25)], dtype=float)
        )
        np.testing.assert_allclose(small[3:], 0.0, atol=1e-12)
        moved = deadband.advance(np.array([0, 0, 0, 0, 0, math.radians(1.0)]))
        np.testing.assert_allclose(moved[3:], [0, 0, threshold], atol=1e-12)

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

    def test_position_and_rotation_filter_parameters_are_independent(self):
        combined = OneEuroPoseFilter(3.0, 40.0, 10.0, 2.0, 3.0)
        position_only = OneEuroPoseFilter(3.0, 40.0, 10.0)
        rotation_only = OneEuroPoseFilter(2.0, 3.0, 10.0)
        for pose_filter in (combined, position_only, rotation_only):
            pose_filter.reset(np.zeros(6))
        for sample in range(1, 51):
            target = np.array([sample * 0.003, 0, 0, 0, 0, sample * 0.01])
            result = combined.advance(target, 0.01)
            np.testing.assert_allclose(result[:3], position_only.advance(target, 0.01)[:3])
            np.testing.assert_allclose(result[3:], rotation_only.advance(target, 0.01)[3:])

    def test_responsive_filter_reduces_tracking_error_vs_old_parameters(self):
        previous_filter = OneEuroPoseFilter(1.5, 0.25, 1.0)
        responsive_filter = OneEuroPoseFilter(3.0, 40.0, 10.0, 3.0, 3.0)
        for pose_filter in (previous_filter, responsive_filter):
            pose_filter.reset(np.zeros(6))
        for sample in range(1, 101):
            target = np.array([sample * 0.003, 0, 0, 0, 0, sample * 0.01])
            previous = previous_filter.advance(target, 0.01)
            responsive = responsive_filter.advance(target, 0.01)
        self.assertLess(target[0] - responsive[0], (target[0] - previous[0]) * 0.25)
        self.assertLess(target[5] - responsive[5], (target[5] - previous[5]) * 0.5)

    def test_responsive_filter_suppresses_stationary_high_frequency_noise(self):
        pose_filter = OneEuroPoseFilter(3.0, 40.0, 10.0, 3.0, 3.0)
        pose_filter.reset(np.zeros(6))
        raw_samples = []
        filtered_samples = []
        for sample in range(200):
            noise = math.sin(2.0 * math.pi * 20.0 * sample * 0.01)
            target = np.array([0.001 * noise, 0, 0, 0, 0, math.radians(0.2) * noise])
            filtered = pose_filter.advance(target, 0.01)
            if sample >= 50:
                raw_samples.append(target[[0, 5]])
                filtered_samples.append(filtered[[0, 5]])
        raw_rms = np.sqrt(np.mean(np.square(raw_samples), axis=0))
        filtered_rms = np.sqrt(np.mean(np.square(filtered_samples), axis=0))
        self.assertTrue(np.all(filtered_rms < raw_rms * 0.25))

    def test_invalid_rotation_filter_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            OneEuroPoseFilter(rotation_min_cutoff=0.0)
        with self.assertRaises(ValueError):
            OneEuroPoseFilter(rotation_beta=-1.0)

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


class CameraHealthTest(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.health = CameraHealth(("/camera_a", "/camera_b"), clock=lambda: self.now)

    def image(self, stamp, valid=True):
        return SimpleNamespace(
            width=1, height=1, data=b"rgb" if valid else b"",
            header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)),
        )

    def ready(self):
        for topic in self.health.topics:
            self.health.observe(topic, self.image(1))
            self.health.observe(topic, self.image(2))

    def test_requires_two_live_frames_from_every_camera(self):
        self.assertEqual(len(self.health.problems()), 2)
        self.health.observe("/camera_a", self.image(1))
        self.assertEqual(len(self.health.problems()), 2)
        self.health.observe("/camera_a", self.image(2))
        self.assertEqual(len(self.health.problems()), 1)
        self.health.observe("/camera_b", self.image(1))
        self.health.observe("/camera_b", self.image(2))
        self.assertEqual(self.health.problems(), [])

    def test_invalid_or_repeated_images_do_not_refresh_health(self):
        self.ready()
        self.now = 1.1
        self.health.observe("/camera_a", self.image(2))
        self.health.observe("/camera_b", self.image(3, valid=False))
        self.assertEqual(len(self.health.problems()), 2)

    def test_recovery_requires_continuous_new_frames(self):
        self.ready()
        self.now = 1.1
        self.health.observe("/camera_a", self.image(3))
        self.assertEqual(len(self.health.problems()), 2)
        self.health.observe("/camera_a", self.image(4))
        self.assertEqual(len(self.health.problems()), 1)

    def test_timestamp_reset_requires_new_frames(self):
        self.ready()
        self.health.observe("/camera_a", self.image(0))
        self.assertEqual(len(self.health.problems()), 1)
        self.health.observe("/camera_a", self.image(1))
        self.health.observe("/camera_a", self.image(2))
        self.assertEqual(self.health.problems(), [])

    def test_capture_start_is_rejected_before_service_call(self):
        publisher = RosDataPublisher.__new__(RosDataPublisher)
        publisher.capture_client = Mock()
        publisher.camera_health = self.health
        self.assertFalse(publisher.request_capture(True, SimpleNamespace()))
        publisher.capture_client.call_async.assert_not_called()

    def test_camera_loss_requests_stop_but_never_start(self):
        publisher = RosDataPublisher.__new__(RosDataPublisher)
        publisher.camera_health = self.health
        publisher.last_camera_warning = float("-inf")
        publisher.last_camera_stop_request = float("-inf")
        publisher.recording = True
        publisher.capture_pending = None
        publisher.request_capture = Mock()
        args = SimpleNamespace()
        publisher.check_capture_cameras(args)
        publisher.check_capture_cameras(args)
        publisher.request_capture.assert_called_once_with(False, args)

    def test_camera_loss_waits_for_pending_capture_request(self):
        publisher = RosDataPublisher.__new__(RosDataPublisher)
        publisher.camera_health = self.health
        publisher.last_camera_warning = float("-inf")
        publisher.last_camera_stop_request = float("-inf")
        publisher.recording = True
        publisher.capture_pending = Mock()
        publisher.capture_pending.done.return_value = False
        publisher.request_capture = Mock()
        publisher.check_capture_cameras(SimpleNamespace())
        publisher.request_capture.assert_not_called()


class EpisodeTrackerTest(unittest.TestCase):
    def test_deletes_only_episode_created_after_capture_started(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old_episode = root / "episode_20260101_000000_000"
            old_episode.mkdir()
            tracker = EpisodeTracker(root)

            tracker.capture_started()
            new_episode = root / "episode_20260101_000001_000"
            new_episode.mkdir()
            tracker.capture_stopped()

            self.assertTrue(tracker.delete_last())
            self.assertTrue(old_episode.is_dir())
            self.assertFalse(new_episode.exists())
            self.assertFalse(tracker.delete_last())

    def test_ignores_non_episode_directories(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            tracker = EpisodeTracker(root)
            tracker.capture_started()
            (root / "camera_cache").mkdir()

            self.assertIsNone(tracker.capture_stopped())
            self.assertFalse(tracker.delete_last())

    def test_ros_publisher_refuses_delete_while_recording(self):
        class Tracker:
            deleted = False

            def delete_last(self):
                self.deleted = True
                return True

        publisher = RosDataPublisher.__new__(RosDataPublisher)
        publisher.capture_pending = None
        publisher.recording = True
        publisher.episode_tracker = Tracker()

        self.assertFalse(publisher.delete_last_episode())
        self.assertFalse(publisher.episode_tracker.deleted)


class URHardwareTest(unittest.TestCase):
    def test_prepare_servo_reuploads_stopped_control_script(self):
        class Control:
            running = False
            uploads = 0

            def isConnected(self):
                return True

            def isProgramRunning(self):
                return self.running

            def reuploadScript(self):
                self.uploads += 1
                self.running = True
                return True

        robot = URHardware.__new__(URHardware)
        robot.control = Control()

        robot.prepare_servo()

        self.assertTrue(robot.control.running)
        self.assertEqual(robot.control.uploads, 1)


if __name__ == "__main__":
    unittest.main()
