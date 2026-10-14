"""Verify delayed disable with fake CAN; no real motors are touched."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import disable_piper_x as script


class FakeDriver:
    def __init__(self, stale=False, ignores_disable=False):
        self.enabled = [True] * 6
        self.commands = []
        self.stale, self.ignores_disable = stale, ignores_disable

    def disable(self, joint_index):
        self.commands.append(joint_index)
        if not self.ignores_disable:
            self.enabled[joint_index - 1] = False

    def get_driver_states(self, joint):
        return SimpleNamespace(timestamp=time.time() - (1 if self.stale else 0),
                               msg=SimpleNamespace(foc_status=SimpleNamespace(
                                   driver_enable_status=self.enabled[joint - 1])))


class DisableTests(unittest.TestCase):
    def test_targets_are_explicit_and_distinct(self):
        self.assertEqual(script.selected_channels("single", None, "can0", "can1"), ["can1"])
        self.assertEqual(script.selected_channels("dual", None, "can0", "can1"), ["can0", "can1"])
        with self.assertRaises(ValueError):
            script.selected_channels("dual", "can1", "can0", "can1")
        with self.assertRaises(ValueError):
            script.selected_channels("dual", None, "can0", "can0")

    def test_countdown_delays_disable_for_five_seconds(self):
        clock = [0.0]
        driver = FakeDriver()
        def advance(seconds):
            self.assertEqual(driver.commands, [])
            clock[0] += seconds
        with patch.object(script.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(script.time, "sleep", side_effect=advance):
            script.countdown(5)
            self.assertEqual(clock[0], 5)
            script.disable_arms({"can1": SimpleNamespace(driver=driver)})
        self.assertEqual(driver.commands, [1, 2, 3, 4, 5, 6])

    def test_cancelled_countdown_never_disables(self):
        driver = FakeDriver()
        with patch.object(script.time, "sleep", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                script.countdown(5)
        self.assertEqual(driver.commands, [])

    def test_missing_feedback_is_not_reported_as_disabled(self):
        self.assertEqual(script.motor_states(SimpleNamespace(get_driver_states=lambda _: None)), [None] * 6)

    def test_stale_or_still_enabled_feedback_fails_but_other_arm_is_disabled(self):
        for failed in (FakeDriver(stale=True), FakeDriver(ignores_disable=True)):
            clock = [0.0]
            good = FakeDriver()
            with patch.object(script.time, "monotonic", side_effect=lambda: clock[0]), \
                 patch.object(script.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)):
                with self.assertRaisesRegex(RuntimeError, "can1"):
                    script.disable_arms({"can0": SimpleNamespace(driver=good),
                                         "can1": SimpleNamespace(driver=failed)}, timeout=0.3)
            self.assertEqual(good.enabled, [False] * 6)


if __name__ == "__main__":
    unittest.main()
