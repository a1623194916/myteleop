"""ServoJ error-14 fault recovery and preflight gating (offline, fake SDK).

Error 14 from the controller means "interface execution failed": while any
robot-level fault is latched the controller rejects ServoJ/ServoMoveEnd with
it, regardless of the commanded motion. These tests pin the recovery behavior:
detect the latched fault, auto-clear it, keep the stream alive, and stop only
when the fault survives repeated clears.
"""
import threading
import time
import types
import unittest

import numpy as np

from xrobotoolkit_teleop.hardware.interface import fr3c as fr3c_module
from xrobotoolkit_teleop.hardware.fr3c_control_utils import AbsoluteDeadlinePacer
from xrobotoolkit_teleop.hardware.interface.fr3c import (
    MAX_CONSECUTIVE_SERVO_ERRORS,
    MAX_FAULT_RECOVERY_ATTEMPTS,
    Fr3cController,
)


class FakeSdkRobot:
    """Mimics the subset of the Fairino RPC object the servo path uses."""

    def __init__(self, servo_errors=0, fault=(0, 0), fault_clearable=True):
        self.servo_calls = 0
        self.servo_cmd_types = []
        self.servo_errors = servo_errors  # return this error code for the first N ServoJ calls
        self.fault = fault
        self.fault_clearable = fault_clearable
        self.reset_error_calls = 0
        self.move_end_calls = 0
        self.move_end_error = 0
        self.robot_state_pkg = None  # set to a SimpleNamespace to test pkg reads

    def gripper_active(self, index=1):
        pkg = self.robot_state_pkg
        return pkg is not None and int(pkg.gripper_active) == 1

    def ServoJ(
        self,
        joint_pos,
        axisPos,
        acc=0.0,
        vel=0.0,
        cmdT=0.008,
        filterT=0.0,
        gain=0.0,
        id=0,
        cmdType=0,
    ):
        self.servo_calls += 1
        self.servo_cmd_types.append(cmdType)
        if self.servo_errors > 0:
            self.servo_errors -= 1
            return 14
        return 0

    def GetRobotErrorCode(self):
        return 0, list(self.fault)

    def ResetAllError(self):
        self.reset_error_calls += 1
        if self.fault_clearable:
            self.fault = (0, 0)
        return 0

    def ServoMoveEnd(self, cmdType=0):
        self.move_end_calls += 1
        if self.fault != (0, 0):
            return 14
        return self.move_end_error


class ServoFaultRecoveryTests(unittest.TestCase):
    def _controller(self, sdk_robot):
        controller = object.__new__(Fr3cController)
        controller.robot = sdk_robot
        controller._rpc_lock = threading.Lock()
        controller._cmd_t = 0.01
        controller._pacer = AbsoluteDeadlinePacer(0.0001)
        controller._last_send_late_s = 0.0
        controller._servo_active = False
        controller._servo_ready = threading.Event()
        controller._consecutive_errors = 0
        controller._last_fault_check_s = 0.0
        controller._fault_recovery_attempts = 0
        return controller

    def setUp(self):
        # Poll the (simulated) fault state on every rejected command.
        self._original_interval = fr3c_module.SERVO_FAULT_POLL_INTERVAL_S
        fr3c_module.SERVO_FAULT_POLL_INTERVAL_S = 0.0

    def tearDown(self):
        fr3c_module.SERVO_FAULT_POLL_INTERVAL_S = self._original_interval

    def test_err14_with_latched_fault_clears_it_and_resumes(self):
        robot = FakeSdkRobot(servo_errors=3, fault=(8, 1))
        controller = self._controller(robot)

        for _ in range(6):
            controller.servo_joints(np.zeros(6))

        self.assertEqual(robot.reset_error_calls, 1)
        self.assertEqual(controller._consecutive_errors, 0)
        self.assertEqual(controller._fault_recovery_attempts, 0)
        self.assertFalse(controller.servo_stream_dead)

    def test_err14_with_latched_fault_does_not_trip_consecutive_limit(self):
        # A persistent fault must survive far longer than the 50-tick
        # consecutive-error window (which passes in 0.5 s at cmd_t=10 ms);
        # the recovery-attempt budget governs death instead.
        robot = FakeSdkRobot(servo_errors=10_000, fault=(8, 1))
        controller = self._controller(robot)

        for _ in range(3 * MAX_CONSECUTIVE_SERVO_ERRORS):
            if controller.servo_stream_dead:
                break
            controller.servo_joints(np.zeros(6))

        self.assertGreater(robot.reset_error_calls, 0)
        self.assertTrue(controller.servo_stream_dead)
        self.assertGreater(controller._fault_recovery_attempts, 0)

    def test_err14_without_fault_still_slows_the_stream(self):
        robot = FakeSdkRobot(servo_errors=5)
        controller = self._controller(robot)

        controller.servo_joints(np.zeros(6))

        self.assertAlmostEqual(controller._cmd_t, 0.015)
        self.assertAlmostEqual(controller._pacer.period_s, 0.015)

    def test_success_resets_all_counters(self):
        robot = FakeSdkRobot()
        controller = self._controller(robot)
        controller._consecutive_errors = 7
        controller._fault_recovery_attempts = 3

        controller.servo_joints(np.zeros(6))

        self.assertEqual(controller._consecutive_errors, 0)
        self.assertEqual(controller._fault_recovery_attempts, 0)
        self.assertTrue(controller.servo_ready)

    def test_udp_transport_passes_cmd_type_one(self):
        robot = FakeSdkRobot()
        controller = self._controller(robot)
        controller._servo_cmd_type = 1

        controller.servo_joints(np.zeros(6))

        self.assertEqual(robot.servo_cmd_types, [1])

    def test_udp_servoj_does_not_wait_for_xmlrpc_lock(self):
        robot = FakeSdkRobot()
        controller = self._controller(robot)
        controller._servo_cmd_type = 1

        controller._rpc_lock.acquire()
        try:
            controller.servo_joints(np.zeros(6))
        finally:
            controller._rpc_lock.release()

        self.assertEqual(robot.servo_calls, 1)

    def test_gripper_active_reads_state_pkg(self):
        robot = FakeSdkRobot()
        controller = self._controller(robot)
        self.assertFalse(controller.robot.gripper_active())

        robot.robot_state_pkg = types.SimpleNamespace(gripper_active=1)
        self.assertTrue(controller.robot.gripper_active())

    def test_gripper_fault_reads_state_pkg(self):
        robot = FakeSdkRobot()
        controller = self._controller(robot)
        self.assertEqual(controller.gripper_fault_code(), -1)

        robot.robot_state_pkg = types.SimpleNamespace(gripper_fault=1)
        self.assertEqual(controller.gripper_fault_code(), 1)

    def test_gripper_recovery_request_is_serviced_by_servo_path(self):
        robot = FakeSdkRobot()
        controller = self._controller(robot)
        controller._servo_active = True
        controller._gripper_recovery_requested = threading.Event()
        controller.request_gripper_recovery()

        controller.servo_joints(np.zeros(6))

        self.assertEqual(robot.reset_error_calls, 1)
        self.assertFalse(controller._gripper_recovery_requested.is_set())

    def test_stop_servo_clears_fault_and_retries_move_end(self):
        robot = FakeSdkRobot(fault=(8, 1))
        controller = self._controller(robot)
        controller._servo_active = True

        controller.stop_servo()

        self.assertEqual(robot.reset_error_calls, 1)
        self.assertEqual(robot.move_end_calls, 2)
        self.assertFalse(controller._servo_active)


class EnsureFaultFreeTests(unittest.TestCase):
    def _controller(self, sdk_robot):
        controller = object.__new__(Fr3cController)
        controller.robot = sdk_robot
        controller._rpc_lock = threading.Lock()
        return controller

    def test_clears_resettable_fault(self):
        robot = FakeSdkRobot(fault=(8, 1))
        controller = self._controller(robot)

        self.assertTrue(controller.ensure_fault_free(max_attempts=3))
        self.assertEqual(robot.fault, (0, 0))

    def test_persistent_fault_reports_not_ready(self):
        robot = FakeSdkRobot(fault=(8, 1), fault_clearable=False)
        controller = self._controller(robot)

        self.assertFalse(controller.ensure_fault_free(max_attempts=3))
        self.assertEqual(robot.reset_error_calls, 3)

    def test_healthy_robot_is_ready_without_resets(self):
        robot = FakeSdkRobot()
        controller = self._controller(robot)

        self.assertTrue(controller.ensure_fault_free(max_attempts=3))
        self.assertEqual(robot.reset_error_calls, 0)


if __name__ == "__main__":
    unittest.main()
