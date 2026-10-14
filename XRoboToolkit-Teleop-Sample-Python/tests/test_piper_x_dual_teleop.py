"""Dual hand/CAN isolation, shared XR and failure cleanup without robot motion."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from scripts.hardware import teleop_piper_x_hardware as entry
from xrobotoolkit_teleop.simulation.piper_x_model import HOME, build_scene


class FakeXR:
    def __init__(self):
        self.values = {"left_grip": 1.0, "right_grip": 0.0,
                       "left_trigger": 0.8, "right_trigger": 0.2}
        self.positions = {"left": 0.0, "right": 0.0}
        self.stamp = 1
        self.closed = False

    def get_pose_by_name(self, name):
        side = name.split("_")[0]
        return np.array([self.positions.get(side, 0), 0, 0, 0, 0, 0, 1.0])

    def get_key_value_by_name(self, name):
        return self.values[name]

    def get_motion_tracker_data(self):
        return {}

    def get_timestamp_ns(self):
        self.stamp += 1
        return self.stamp

    def get_button_state_by_name(self, name):
        return False

    def close(self):
        self.closed = True


class FakeArm:
    gripper_max_width = 0.07

    def __init__(self, fail=False):
        self.commands, self.grippers = [], []
        self.closed = False
        self.fail = fail

    def move_joints(self, command):
        self.commands.append(command)

    def set_gripper(self, opening, force=1.0):
        self.grippers.append(opening)
        self.gripper_force = force

    def open_gripper(self, force=5.0):
        pass

    def read_joints(self):
        if self.fail:
            raise RuntimeError("CAN feedback lost")
        return np.asarray(HOME)

    def close(self):
        self.closed = True


class DualPiperTest(unittest.TestCase):
    def test_startup_waits_past_five_seconds_and_for_both_hands(self):
        xr = FakeXR()
        clock = [0.0]
        def pose(name):
            valid = clock[0] >= (12 if name == "left_controller" else 6)
            return np.array([0, 0, 0, 0, 0, 0, float(valid)])
        xr.get_pose_by_name = pose
        with patch.object(entry.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(entry.time, "sleep", side_effect=lambda _: clock.__setitem__(0, clock[0] + 1)), \
             patch.object(entry, "PiperXCanArm") as can:
            entry.wait_for_pico(xr, ("left", "right"))
            can.assert_not_called()
        self.assertEqual(clock[0], 12)

    def test_frozen_packets_do_not_pass_startup_wait(self):
        xr = FakeXR()
        xr.get_timestamp_ns = lambda: 123
        clock = [0.0]
        with patch.object(entry.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(entry.time, "sleep", side_effect=lambda _: clock.__setitem__(0, clock[0] + 1)):
            with self.assertRaisesRegex(RuntimeError, "after 3 seconds"):
                entry.wait_for_pico(xr, ("right",), timeout=3)

    def test_cancel_wait_closes_xr_without_opening_can(self):
        xr = FakeXR()
        with patch("xrobotoolkit_teleop.common.xr_client.XrClient", return_value=xr), \
             patch.object(entry, "TeleopArm", return_value=SimpleNamespace(arm=None)), \
             patch.object(entry, "wait_for_pico", side_effect=KeyboardInterrupt), \
             patch.object(entry, "PiperXCanArm") as can:
            with self.assertRaises(KeyboardInterrupt):
                entry.main(mode="dual", enable_hardware=True)
            can.assert_not_called()
        self.assertTrue(xr.closed)

    def test_live_feedback_failure_closes_both_arms_before_any_new_command(self):
        xr = FakeXR()
        live = {"can0": FakeArm(), "can1": FakeArm()}
        reads = [0]
        def right_feedback():
            reads[0] += 1
            if reads[0] > 1:
                raise RuntimeError("CAN feedback lost")
            return np.asarray(HOME)
        live["can1"].read_joints = right_feedback
        def runtime(side, channel, *args, **kwargs):
            return SimpleNamespace(side=side, channel=channel, arm=None,
                                   solve=lambda measured: self.fail("IK called before checking both feedbacks"))
        def connect(channel, read_only=False, **kwargs):
            return FakeArm() if read_only else live[channel]
        with patch("xrobotoolkit_teleop.common.xr_client.XrClient", return_value=xr), \
             patch.object(entry, "TeleopArm", side_effect=runtime), \
             patch.object(entry, "PiperXCanArm", side_effect=connect):
            with self.assertRaisesRegex(RuntimeError, "feedback lost"):
                entry.main(mode="dual", enable_hardware=True)
        self.assertTrue(all(arm.closed for arm in live.values()))
        self.assertEqual(live["can0"].commands, [])
        self.assertTrue(xr.closed)

    def test_mode_mapping_and_duplicate_channels_rejected(self):
        self.assertEqual(entry.select_arms("dual", "right", None, "can0", "can1"),
                         {"left": "can0", "right": "can1"})
        self.assertEqual(entry.select_arms("single", "right", None, "can0", "can1"), {"right": "can1"})
        self.assertEqual(entry.select_arms("single", "left", "can7", "can0", "can1"), {"left": "can7"})
        with self.assertRaisesRegex(ValueError, "different"):
            entry.select_arms("dual", "right", None, "can0", "can0")
        with self.assertRaisesRegex(ValueError, "single mode"):
            entry.select_arms("dual", "right", "can0", "can0", "can1")

    def test_independent_clutches_and_triggers_with_one_shared_xr(self):
        xr = FakeXR()
        urdf, scene = build_scene()
        with patch("xrobotoolkit_teleop.common.base_teleop_controller.XrClient") as native_client:
            left = entry.TeleopArm("left", "can0", xr, urdf, scene, 0.5, 0.35)
            right = entry.TeleopArm("right", "can1", xr, urdf, scene, 0.5, 0.35)
            native_client.assert_not_called()
        self.assertIs(left.controller.xr_client, right.controller.xr_client)
        left.arm, right.arm = FakeArm(), FakeArm()
        left.gripper_armed = right.gripper_armed = True
        measured = np.asarray(HOME)
        left.solve(measured)  # Anchor left hand.
        xr.positions["left"] = 0.06
        command, active = left.solve(measured)
        self.assertTrue(active)
        self.assertGreater(np.max(np.abs(command - measured)), 0)
        left.send(command, time.monotonic())
        command, active = right.solve(measured)
        self.assertFalse(active)
        right.send(command, time.monotonic())
        self.assertEqual(len(left.arm.commands), 1)
        self.assertEqual(len(right.arm.commands), 0)
        self.assertEqual(len(right.arm.grippers), 1)  # Trigger works with GRIP released.
        self.assertAlmostEqual(right.arm.grippers[0], 1 - (0.2 - 0.05) / 0.9)
        self.assertEqual(right.arm.gripper_force, 5.0)
        self.assertAlmostEqual(left.arm.grippers[0], 1 - (0.8 - 0.05) / 0.9)
        xr.values["left_grip"], xr.values["right_grip"] = 0, 1
        self.assertIsNone(left.solve(measured)[0])
        command, active = right.solve(measured)
        right.send(command, time.monotonic())
        self.assertTrue(active)
        self.assertEqual(len(right.arm.commands), 1)
        self.assertAlmostEqual(right.arm.grippers[0], 1 - (0.2 - 0.05) / 0.9)

    def test_gripper_refresh_and_rate_limit_without_joint_commands(self):
        xr = FakeXR()
        runtime = entry.TeleopArm("right", "can1", xr, *build_scene(), 1.0, 0.6)
        runtime.arm = FakeArm()
        runtime.gripper_armed = True
        runtime.send(None, 10.0)
        runtime.send(None, 10.05)
        self.assertEqual(len(runtime.arm.grippers), 1)
        runtime.send(None, 10.6)
        self.assertEqual(len(runtime.arm.grippers), 2)
        self.assertEqual(runtime.arm.commands, [])
        self.assertAlmostEqual(runtime.gripper_command_m, (1 - (0.2 - 0.05) / 0.9) * 0.07)

    def test_ik_period_matches_loop_and_filters_reset_when_regripping(self):
        xr = FakeXR()
        runtime = entry.TeleopArm("left", "can0", xr, *build_scene(), 1.0, 0.6, rate_hz=50)
        controller = runtime.controller
        self.assertAlmostEqual(controller.dt, 0.02)
        self.assertAlmostEqual(controller.solver.dt, 0.02)
        controller._process_xr_pose(xr.get_pose_by_name("left_controller"), "left_hand")
        xr.positions["left"] = 0.1
        delta, _ = controller._process_xr_pose(xr.get_pose_by_name("left_controller"), "left_hand")
        self.assertLessEqual(np.linalg.norm(delta), 0.5 * 0.02 + 1e-9)
        xr.positions["left"] = 1.0  # Tracker teleport must hold the target.
        held, _ = controller._process_xr_pose(xr.get_pose_by_name("left_controller"), "left_hand")
        np.testing.assert_allclose(held, delta)
        controller.ref_controller_xyz["left_hand"] = None
        reset, _ = controller._process_xr_pose(xr.get_pose_by_name("left_controller"), "left_hand")
        np.testing.assert_allclose(reset, 0)

    def test_initial_gripper_stays_open_until_trigger_release_and_endpoints_snap(self):
        xr = FakeXR()
        runtime = entry.TeleopArm("right", "can1", xr, *build_scene(), 1.0, 0.6)
        runtime.arm = FakeArm()
        xr.values["right_trigger"] = 1.0
        runtime.send(None, 10)
        self.assertEqual(runtime.arm.grippers[-1], 1.0)
        xr.values["right_trigger"] = 0.03
        runtime.send(None, 10.2)
        self.assertTrue(runtime.gripper_armed)
        xr.values["right_trigger"] = 0.97
        runtime.send(None, 10.4)
        self.assertEqual(runtime.arm.grippers[-1], 0.0)
        xr.values["right_trigger"] = 0.01
        runtime.send(None, 10.6)
        self.assertEqual(runtime.arm.grippers[-1], 1.0)

    def test_filtered_hand_step_reaches_within_six_mm_in_100ms(self):
        xr = FakeXR()
        runtime = entry.TeleopArm("left", "can0", xr, *build_scene(), 1.0, 0.6)
        controller = runtime.controller
        controller._process_xr_pose(xr.get_pose_by_name("left_controller"), "left_hand")
        xr.positions["left"] = 0.03
        for _ in range(5):
            delta, _ = controller._process_xr_pose(xr.get_pose_by_name("left_controller"), "left_hand")
        self.assertGreater(np.linalg.norm(delta), 0.024)
        self.assertLessEqual(np.linalg.norm(delta), 0.03)

    def test_second_arm_preflight_failure_never_enables_first(self):
        xr, first_probe = FakeXR(), FakeArm()
        calls = []
        def connect(channel, read_only=False, **kwargs):
            calls.append((channel, read_only))
            if channel == "can1":
                raise RuntimeError("second arm unavailable")
            return first_probe
        with patch("xrobotoolkit_teleop.common.xr_client.XrClient", return_value=xr), \
             patch.object(entry, "TeleopArm", return_value=SimpleNamespace(arm=None)), \
             patch.object(entry, "PiperXCanArm", side_effect=connect):
            with self.assertRaisesRegex(RuntimeError, "second arm"):
                entry.main(mode="dual", enable_hardware=True)
        self.assertEqual(calls, [("can0", True), ("can1", True)])
        self.assertTrue(first_probe.closed)
        self.assertTrue(xr.closed)


if __name__ == "__main__":
    unittest.main()
