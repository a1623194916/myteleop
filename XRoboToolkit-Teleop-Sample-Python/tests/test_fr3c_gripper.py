import threading
import unittest

from xrobotoolkit_teleop.hardware.fr3c_gripper import (
    Fr3cVrGripperController,
    TriggerGripperMapper,
)
from xrobotoolkit_teleop.hardware.interface.fr3c import Fr3cController


class TriggerGripperMapperTests(unittest.TestCase):
    def setUp(self):
        self.mapper = TriggerGripperMapper(
            trigger_threshold=0.1,
            open_position_percent=100.0,
            closed_position_percent=0.0,
            max_width_mm=90.0,
        )

    def test_trigger_at_or_below_threshold_is_fully_open(self):
        for trigger in (0.0, 0.05, 0.1):
            target = self.mapper.map(trigger)
            self.assertEqual(target.position_percent, 100.0)
            self.assertEqual(target.width_mm, 90.0)
            self.assertEqual(target.closure, 0.0)

    def test_trigger_above_threshold_maps_continuously_to_width(self):
        target = self.mapper.map(0.55)

        self.assertAlmostEqual(target.closure, 0.5)
        self.assertAlmostEqual(target.position_percent, 50.0)
        self.assertAlmostEqual(target.width_mm, 45.0)

    def test_full_trigger_is_fully_closed(self):
        target = self.mapper.map(1.0)

        self.assertEqual(target.position_percent, 0.0)
        self.assertEqual(target.width_mm, 0.0)
        self.assertEqual(target.closure, 1.0)

    def test_trigger_is_clamped_to_sdk_range(self):
        self.assertEqual(self.mapper.map(-0.2).position_percent, 100.0)
        self.assertEqual(self.mapper.map(1.2).position_percent, 0.0)


class FakeXrClient:
    def __init__(self, trigger_value=0.0):
        self.trigger_value = trigger_value
        self.button_value = False
        self.requested_keys = []

    def get_key_value_by_name(self, name):
        self.requested_keys.append(name)
        return self.trigger_value

    def get_button_state_by_name(self, name):
        return self.button_value


class FakeFr3cRobot:
    error_code = (0, 0)

    def __init__(self):
        self.positions = []
        self.activations = 0
        self.active = True
        self.servo_active = False
        self.reset_calls = 0
        self._gripper_expected_active = False
        self._gripper_needs_activation = False

    @property
    def gripper_needs_activation(self):
        return self._gripper_needs_activation

    def latched_fault_code(self):
        return self.error_code

    def reset_all_errors(self):
        self.reset_calls += 1
        self.error_code = (0, 0)
        if self._gripper_expected_active:
            self._gripper_needs_activation = True

    def gripper_active(self, index=1):
        return self.active

    def activate_gripper(self, **kwargs):
        self.activations += 1
        self.active = True
        self._gripper_expected_active = True
        self._gripper_needs_activation = False

    def move_gripper(self, position_percent, **kwargs):
        self.positions.append(position_percent)
        return 0

    def gripper_fault_code(self):
        return 0


class Fr3cVrGripperControllerTests(unittest.TestCase):
    def test_update_reads_selected_side_and_sends_mapped_position(self):
        xr_client = FakeXrClient(trigger_value=0.55)
        robot = FakeFr3cRobot()
        controller = Fr3cVrGripperController(
            xr_client=xr_client,
            robot=robot,
            controller_side="left",
            trigger_threshold=0.1,
            trigger_toggle=False,
        )

        target = controller.update()

        self.assertEqual(xr_client.requested_keys, ["left_trigger"])
        self.assertAlmostEqual(target.width_mm, 45.0)
        self.assertEqual(len(robot.positions), 1)
        self.assertAlmostEqual(robot.positions[0], 50.0)

    def test_update_suppresses_small_repeated_commands(self):
        xr_client = FakeXrClient(trigger_value=0.0)
        robot = FakeFr3cRobot()
        controller = Fr3cVrGripperController(
            xr_client=xr_client,
            robot=robot,
            controller_side="right",
            min_position_change_percent=2.0,
            trigger_toggle=False,
        )

        controller.update()
        xr_client.trigger_value = 0.01
        controller.update()

        self.assertEqual(robot.positions, [100.0])

    def test_trigger_is_edge_toggle_and_holds_until_next_press(self):
        xr_client = FakeXrClient(trigger_value=0.0)
        robot = FakeFr3cRobot()
        controller = Fr3cVrGripperController(
            xr_client=xr_client,
            robot=robot,
            controller_side="right",
            trigger_threshold=0.1,
            open_position_percent=0.0,
            closed_position_percent=97.0,
            min_command_interval_s=0.0,
            toggle_debounce_s=0.0,
        )

        controller.update()  # establish the open baseline
        xr_client.trigger_value = 1.0
        controller.update()  # rising edge: close
        controller.update()  # held: still closed, no new command
        xr_client.trigger_value = 0.0
        controller.update()  # release
        xr_client.trigger_value = 1.0
        target = controller.update()  # rising edge: open

        self.assertAlmostEqual(target.position_percent, 0.0)
        self.assertEqual(robot.positions, [97.0, 0.0])

    def test_trigger_and_button_edges_do_not_toggle_twice(self):
        xr_client = FakeXrClient(trigger_value=0.0)
        robot = FakeFr3cRobot()
        controller = Fr3cVrGripperController(
            xr_client=xr_client,
            robot=robot,
            controller_side="right",
            min_command_interval_s=0.0,
            toggle_button="right_axis_click",
        )

        controller.update()
        xr_client.trigger_value = 1.0
        xr_client.button_value = True
        controller.update()
        self.assertEqual(robot.positions, [0.0])

    def test_timeout_is_not_retried_until_next_toggle(self):
        xr_client = FakeXrClient(trigger_value=0.0)
        robot = FakeFr3cRobot()
        attempts = []

        def move_with_first_timeout(position_percent, **kwargs):
            attempts.append(position_percent)
            if len(attempts) == 1:
                raise TimeoutError("timed out")
            robot.positions.append(position_percent)
            return 0

        robot.move_gripper = move_with_first_timeout
        controller = Fr3cVrGripperController(
            xr_client=xr_client,
            robot=robot,
            controller_side="right",
            trigger_threshold=0.1,
            open_position_percent=0.0,
            closed_position_percent=90.0,
            min_command_interval_s=0.0,
            toggle_debounce_s=0.0,
        )
        controller.initialize_idle()
        controller.update()

        xr_client.trigger_value = 1.0
        controller.update()
        controller.update()
        controller.update()
        self.assertEqual(attempts, [90.0])

        xr_client.trigger_value = 0.0
        controller.update()
        xr_client.trigger_value = 1.0
        controller.update()

        self.assertEqual(attempts, [90.0, 0.0])
        self.assertEqual(robot.positions, [0.0])

    def test_activation_is_explicit(self):
        robot = FakeFr3cRobot()
        controller = Fr3cVrGripperController(
            xr_client=FakeXrClient(),
            robot=robot,
            controller_side="right",
        )

        self.assertEqual(robot.activations, 0)
        controller.activate()
        self.assertEqual(robot.activations, 1)

    def test_initialize_open_sends_open_target_with_release_force(self):
        robot = FakeFr3cRobot()
        controller = Fr3cVrGripperController(
            xr_client=FakeXrClient(),
            robot=robot,
            controller_side="right",
            open_position_percent=0.0,
            closed_position_percent=97.0,
            force=3,
            open_force=100,
        )

        controller.initialize_open()

        self.assertEqual(robot.positions, [0.0])
        self.assertFalse(controller._toggle_closed)

    def test_initialize_open_reactivates_after_fault_clear(self):
        robot = FakeFr3cRobot()
        robot._gripper_needs_activation = True
        controller = Fr3cVrGripperController(
            xr_client=FakeXrClient(),
            robot=robot,
            controller_side="right",
        )

        controller.initialize_open()

        self.assertEqual(robot.activations, 1)
        self.assertEqual(robot.positions, [100.0])

    def test_initialize_idle_reactivates_without_motion(self):
        robot = FakeFr3cRobot()
        robot._gripper_needs_activation = True
        controller = Fr3cVrGripperController(
            xr_client=FakeXrClient(),
            robot=robot,
            controller_side="right",
        )

        controller.initialize_idle()

        self.assertEqual(robot.activations, 1)
        self.assertEqual(robot.positions, [])
        self.assertEqual(controller._last_command_percent, 100.0)

    def test_servo_thread_owns_fault_recovery(self):
        xr_client = FakeXrClient(trigger_value=0.0)
        robot = FakeFr3cRobot()
        robot.error_code = (8, 1)
        robot.servo_active = True
        controller = Fr3cVrGripperController(
            xr_client=xr_client,
            robot=robot,
            controller_side="right",
            trigger_toggle=False,
        )

        controller.update()

        self.assertEqual(robot.reset_calls, 0)
        self.assertEqual(robot.positions, [])

    def test_gripper_reactivates_after_servo_fault_clear(self):
        xr_client = FakeXrClient(trigger_value=0.0)
        robot = FakeFr3cRobot()
        robot._gripper_expected_active = True
        robot._gripper_needs_activation = True
        controller = Fr3cVrGripperController(
            xr_client=xr_client,
            robot=robot,
            controller_side="right",
            trigger_toggle=False,
            min_command_interval_s=0.0,
        )

        controller.update()

        self.assertEqual(robot.activations, 1)
        self.assertEqual(robot.positions, [])
        self.assertFalse(robot.gripper_needs_activation)

    def test_gripper_timeout_requests_recovery_from_servo_thread(self):
        robot = FakeFr3cRobot()
        robot.servo_active = True
        robot.gripper_fault = 1  # Fairino: tool 485 timeout
        robot.recovery_requests = 0
        robot.gripper_fault_code = lambda: robot.gripper_fault
        robot.request_gripper_recovery = lambda: setattr(
            robot, "recovery_requests", robot.recovery_requests + 1
        )
        controller = Fr3cVrGripperController(
            xr_client=FakeXrClient(),
            robot=robot,
            controller_side="right",
        )

        controller.update()

        self.assertEqual(robot.recovery_requests, 1)
        self.assertEqual(robot.positions, [])

    def test_toggle_survives_timeout_clear_and_reactivation(self):
        xr_client = FakeXrClient(trigger_value=0.0)
        robot = FakeFr3cRobot()
        robot.servo_active = True
        robot.gripper_fault = 0
        robot.recovery_requests = 0
        robot.gripper_fault_code = lambda: robot.gripper_fault
        robot.request_gripper_recovery = lambda: setattr(
            robot, "recovery_requests", robot.recovery_requests + 1
        )
        controller = Fr3cVrGripperController(
            xr_client=xr_client,
            robot=robot,
            controller_side="right",
            trigger_threshold=0.1,
            open_position_percent=0.0,
            closed_position_percent=97.0,
            min_command_interval_s=0.0,
            toggle_debounce_s=0.0,
        )
        controller.initialize_idle()
        controller.update()  # establish released input state

        robot.gripper_fault = 1
        xr_client.trigger_value = 1.0
        controller.update()  # latch close target and request fault recovery
        self.assertEqual(robot.recovery_requests, 1)
        self.assertEqual(robot.positions, [])

        # Simulate ResetAllError on the servo thread. The CNDE fault remains
        # stale until after activation on the affected firmware.
        robot._gripper_needs_activation = True
        controller._next_retry_at_s = 0.0
        controller.update()
        self.assertEqual(robot.activations, 1)

        controller._next_retry_at_s = 0.0
        controller.update()
        controller.update()  # held input must not resend the close target

        self.assertEqual(robot.positions, [97.0])


class FakeSdkRobot:
    def __init__(self):
        self.activation_calls = []
        self.move_calls = []
        self.reset_error_calls = 0

    def ActGripper(self, index, action):
        self.activation_calls.append((index, action))
        return 0

    def MoveGripper(self, *args):
        self.move_calls.append(args)
        return 0

    def ResetAllError(self):
        self.reset_error_calls += 1
        return 0


class Fr3cInterfaceGripperTests(unittest.TestCase):
    def setUp(self):
        self.sdk_robot = FakeSdkRobot()
        self.controller = object.__new__(Fr3cController)
        self.controller.robot = self.sdk_robot
        self.controller._rpc_lock = threading.Lock()
        self.controller._gripper_rpc_lock = threading.Lock()

    def test_activate_gripper_resets_then_activates(self):
        self.controller.activate_gripper(reset_delay=0.0, activation_delay=0.0)

        self.assertEqual(self.sdk_robot.activation_calls, [(1, 0), (1, 1)])

    def test_clear_gripper_warning_resets_alarm_and_gripper(self):
        self.controller._gripper_expected_active = True
        self.controller._gripper_needs_activation = True

        self.controller.clear_gripper_warning(
            error_settle_delay=0.0,
            reset_delay=0.0,
        )

        self.assertEqual(self.sdk_robot.reset_error_calls, 1)
        self.assertEqual(self.sdk_robot.activation_calls, [(1, 0)])
        self.assertFalse(self.controller._gripper_expected_active)
        self.assertFalse(self.controller._gripper_needs_activation)

    def test_move_gripper_uses_nonblocking_parallel_gripper_command(self):
        self.controller.move_gripper(
            37.0,
            index=1,
            velocity=40,
            force=30,
            max_time_ms=1000,
        )

        self.assertEqual(
            self.sdk_robot.move_calls,
            [(1, 37, 40, 30, 1000, 1, 0, 0.0, 0, 0)],
        )

    def test_move_gripper_can_drop_when_rpc_lock_is_busy(self):
        self.controller._gripper_rpc_lock.acquire()
        try:
            result = self.controller.move_gripper(
                37.0, index=1, velocity=40, force=30, max_time_ms=1000,
                lock_timeout_s=0.0,
            )
        finally:
            self.controller._gripper_rpc_lock.release()
        self.assertEqual(result, 1)
        self.assertEqual(self.sdk_robot.move_calls, [])

class FaultyGripperRobot(FakeFr3cRobot):
    """MoveGripper fails while a fault is latched; motion re-latches it.

    Mirrors the live rig: every completed gripper motion trips servo drive
    fault 8-1 (the tool-485 feedback channel never reports position), and a
    latched fault rejects the next MoveGripper with 73.
    """

    def __init__(self):
        super().__init__()
        self.error_code = (8, 1)
        self.reset_calls = 0

    def reset_all_errors(self):
        self.reset_calls += 1
        self.error_code = (0, 0)

    def move_gripper(self, position_percent, **kwargs):
        if self.error_code != (0, 0):
            raise RuntimeError("MoveGripper failed, error code 73")
        self.positions.append(position_percent)
        self.error_code = (8, 1)  # motion completion trips the fault again
        return 0


class Fr3cGripperRecoveryTests(unittest.TestCase):
    def _controller(self, robot):
        return Fr3cVrGripperController(
            xr_client=FakeXrClient(trigger_value=0.0),
            robot=robot,
            controller_side="right",
            min_position_change_percent=5.0,
            trigger_toggle=False,
        )

    def test_latched_fault_is_cleared_before_sending(self):
        robot = FaultyGripperRobot()
        controller = self._controller(robot)

        controller.xr_client.trigger_value = 0.3
        controller.update()

        self.assertEqual(robot.reset_calls, 1)  # 8-1 cleared reactively
        self.assertAlmostEqual(robot.positions[0], 73.68421052631578, places=6)

    def test_no_reset_when_healthy(self):
        robot = FaultyGripperRobot()
        robot.error_code = (0, 0)
        controller = self._controller(robot)

        controller.xr_client.trigger_value = 0.3
        controller._last_command_at_s = None  # bypass min interval
        controller.update()

        self.assertEqual(robot.reset_calls, 0)
        self.assertEqual(len(robot.positions), 1)

    def test_every_motion_retrips_and_is_cleared_again(self):
        # Slider following: consecutive targets keep flowing despite the
        # per-motion fault retrips, each cleared reactively.
        robot = FaultyGripperRobot()
        controller = self._controller(robot)
        controller.xr_client.trigger_value = 0.2
        controller.update()
        controller._last_command_at_s = None
        controller.xr_client.trigger_value = 0.5
        controller.update()
        controller._last_command_at_s = None
        controller.xr_client.trigger_value = 0.8
        controller.update()

        self.assertEqual(len(robot.positions), 3)
        self.assertEqual(robot.reset_calls, 3)


if __name__ == "__main__":
    unittest.main()
