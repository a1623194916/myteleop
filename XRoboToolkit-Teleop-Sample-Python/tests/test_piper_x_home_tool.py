"""Standalone home tool: bounded, gripper-free motion without CAN hardware."""

import contextlib
import io
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from scripts.hardware import piper_x_home_tool as tool
from xrobotoolkit_teleop.hardware.piper_x import PiperXCanArm as RealPiperXCanArm
from xrobotoolkit_teleop.hardware.piper_x_home import load_home_targets

# The limits the MuJoCo scene reports, in degrees: J2 cannot go below 0 and J3
# cannot go above 0, which is why the saved poses get bounded on those two.
LIMITS_DEG = ((-150.0, 150.0), (0.0, 180.0), (-170.0, 0.0),
              (-89.0, 89.0), (-89.0, 89.0), (-120.0, 120.0))
HOME_CONFIG = tool.DEFAULT_HOME_CONFIG


def limits_rad():
    return (np.deg2rad([low for low, _ in LIMITS_DEG]),
            np.deg2rad([high for _, high in LIMITS_DEG]))


def bounded(side, channel):
    lower, upper = limits_rad()
    raw = load_home_targets(HOME_CONFIG, {side: channel})[side]
    return np.clip(raw, lower, upper)


class FakeDriver:
    """A stiff servo: the joints hold whatever was commanded last."""

    class OPTIONS:
        class EFFECTOR:
            AGX_GRIPPER = "agx_gripper"

    def __init__(self, joints, follow=True):
        self.joints = np.asarray(joints, dtype=float)
        self.moves = []
        self.effector_calls = []
        self.enabled = False
        self.connected = False
        self.speed = None
        self.follow = follow

    def init_effector(self, kind):
        self.effector_calls.append(("init_effector", kind))
        return SimpleNamespace(kind=kind)

    def connect(self):
        self.connected = True

    def is_connected(self):
        return self.connected

    def is_ok(self):
        return True

    def enable(self):
        self.enabled = True
        return True

    def set_speed_percent(self, percent):
        self.speed = percent

    def get_joint_angles(self):
        if self.follow and self.moves:
            self.joints = np.asarray(self.moves[-1][1], dtype=float)
        return SimpleNamespace(msg=self.joints.copy(), timestamp=time.time())

    def move_j(self, values):
        self.moves.append(("j", list(values)))

    def move_js(self, values):
        self.moves.append(("js", list(values)))

    def disconnect(self):
        self.connected = False


class HomeToolTest(unittest.TestCase):
    def setUp(self):
        limiter = patch.object(tool, "joint_limits", side_effect=limits_rad)
        self.addCleanup(limiter.stop)
        limiter.start()

    def install_arm(self, start, follow=True):
        driver = FakeDriver(start, follow=follow)

        def factory(channel="can1", firmware=None, read_only=False, speed_percent=10):
            # The real class, not the patched name, or this would recurse.
            return RealPiperXCanArm(channel=channel, driver=driver,
                                    read_only=read_only, speed_percent=speed_percent)

        arm = patch.object(tool, "PiperXCanArm", side_effect=factory)
        self.addCleanup(arm.stop)
        arm.start()
        return driver

    def run_tool(self, argv):
        with contextlib.redirect_stdout(io.StringIO()), \
             contextlib.redirect_stderr(io.StringIO()) as noise:
            code = tool.main(argv)
        return code, noise.getvalue()

    def test_go_reaches_the_bounded_home_in_slew_limited_planned_steps(self):
        driver = self.install_arm(np.zeros(6))
        code, _ = self.run_tool(["go", "--mode", "single", "--channel", "can1", "--yes"])
        self.assertEqual(code, 0)
        self.assertTrue(driver.moves)
        # Home uses planned motion, exactly like the teleop home button, and
        # nothing else is sent on a joint channel.
        self.assertEqual({mode for mode, _ in driver.moves}, {"j"})
        steps = np.diff(np.vstack([np.zeros(6), *[v for _, v in driver.moves]]), axis=0)
        self.assertLessEqual(float(np.max(np.abs(steps))), np.deg2rad(0.15) + 1e-9)
        np.testing.assert_allclose(driver.moves[-1][1], bounded("right", "can1"),
                                   atol=np.deg2rad(0.5))

    def test_go_never_commands_the_gripper_and_leaves_motors_enabled(self):
        driver = self.install_arm(np.zeros(6))
        code, _ = self.run_tool(["go", "--mode", "single", "--channel", "can1", "--yes"])
        self.assertEqual(code, 0)
        # Only the constructor's effector lookup happens; no width is ever sent.
        self.assertEqual(driver.effector_calls, [("init_effector", "agx_gripper")])
        self.assertTrue(driver.enabled)
        self.assertEqual(driver.speed, 100)

    def test_already_at_home_sends_nothing(self):
        driver = self.install_arm(bounded("right", "can1"))
        code, _ = self.run_tool(["go", "--mode", "single", "--channel", "can1"])
        self.assertEqual(code, 0)
        self.assertEqual(driver.moves, [])

    def test_show_is_read_only_and_never_enables_the_motors(self):
        driver = self.install_arm(np.zeros(6))
        code, _ = self.run_tool(["show", "--mode", "single", "--channel", "can1"])
        self.assertEqual(code, 0)
        self.assertEqual(driver.moves, [])
        self.assertFalse(driver.enabled)
        self.assertEqual(driver.effector_calls, [])

    def test_dry_run_never_opens_can(self):
        opener = Mock(side_effect=AssertionError("CAN must not be opened"))
        with patch.object(tool, "open_arms", opener):
            code, _ = self.run_tool(["go", "--mode", "single", "--channel", "can1", "--dry-run"])
        self.assertEqual(code, 0)
        opener.assert_not_called()

    def test_running_teleop_blocks_the_tool_before_any_can_access(self):
        listing = SimpleNamespace(
            stdout=" 4242 /usr/bin/python3 scripts/hardware/teleop_piper_x_hardware.py\n"
                   " 4243 /bin/bash -c something else\n")
        opener = Mock(side_effect=AssertionError("CAN must not be opened"))
        with patch.object(tool.subprocess, "run", return_value=listing), \
             patch.object(tool, "open_arms", opener):
            code, noise = self.run_tool(["go", "--mode", "single", "--channel", "can1", "--yes"])
        self.assertEqual(code, 1)
        self.assertIn("CAN bus", noise)
        opener.assert_not_called()

    def test_frozen_feedback_times_out_instead_of_commanding_forever(self):
        driver = self.install_arm(np.zeros(6), follow=False)
        code, noise = self.run_tool(["go", "--mode", "single", "--channel", "can1",
                                     "--yes", "--timeout", "0.05"])
        self.assertEqual(code, 1)
        self.assertIn("timed out", noise)
        self.assertLess(len(driver.moves), 20)

    def test_can_error_state_is_parsed(self):
        healthy = SimpleNamespace(stdout="440: can1: <NOARP,UP,LOWER_UP> mtu 16 "
                                         "qdisc pfifo_fast state UP mode DEFAULT\n"
                                         "    can state ERROR-ACTIVE restart-ms 0 \n")
        with patch.object(tool.subprocess, "run", return_value=healthy):
            self.assertEqual(tool.can_link_state("can1"), "ERROR-ACTIVE")
        dead = SimpleNamespace(stdout="    can state ERROR-PASSIVE restart-ms 0 \n")
        with patch.object(tool.subprocess, "run", return_value=dead):
            self.assertEqual(tool.can_link_state("can1"), "ERROR-PASSIVE")
        with patch.object(tool.subprocess, "run", return_value=SimpleNamespace(stdout="nonsense")):
            self.assertEqual(tool.can_link_state("can1"), "unknown")

    def test_powered_off_arm_reports_the_link_state_not_just_the_sdk_error(self):
        # A powered-off arm stops acknowledging frames: the channel drops out of
        # ERROR-ACTIVE and the SDK only says 'firmware version unavailable'.
        dead = SimpleNamespace(stdout="    can state ERROR-PASSIVE restart-ms 0 \n")
        opener = Mock(side_effect=RuntimeError("Piper-X firmware version unavailable"))
        with patch.object(tool, "open_arms", opener), \
             patch.object(tool.subprocess, "run", return_value=dead):
            code, noise = self.run_tool(["show", "--mode", "single", "--channel", "can1"])
        self.assertEqual(code, 1)
        self.assertIn("can1=ERROR-PASSIVE", noise)
        self.assertIn("powered off", noise)


if __name__ == "__main__":
    unittest.main()
