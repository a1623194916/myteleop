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
        self.requested_keys = []

    def get_key_value_by_name(self, name):
        self.requested_keys.append(name)
        return self.trigger_value


class FakeFr3cRobot:
    def __init__(self):
        self.positions = []
        self.activations = 0

    def activate_gripper(self, **kwargs):
        self.activations += 1

    def move_gripper(self, position_percent, **kwargs):
        self.positions.append(position_percent)
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
        )

        controller.update()
        xr_client.trigger_value = 0.01
        controller.update()

        self.assertEqual(robot.positions, [100.0])

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


class FakeSdkRobot:
    def __init__(self):
        self.activation_calls = []
        self.move_calls = []

    def ActGripper(self, index, action):
        self.activation_calls.append((index, action))
        return 0

    def MoveGripper(self, *args):
        self.move_calls.append(args)
        return 0


class Fr3cInterfaceGripperTests(unittest.TestCase):
    def setUp(self):
        self.sdk_robot = FakeSdkRobot()
        self.controller = object.__new__(Fr3cController)
        self.controller.robot = self.sdk_robot

    def test_activate_gripper_resets_then_activates(self):
        self.controller.activate_gripper(reset_delay=0.0, activation_delay=0.0)

        self.assertEqual(self.sdk_robot.activation_calls, [(1, 0), (1, 1)])

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


if __name__ == "__main__":
    unittest.main()
