import unittest

from xrobotoolkit_teleop.hardware.fr3c_suction import Fr3cSuctionController


class FakeXrClient:
    def __init__(self, trigger_value=0.0):
        self.trigger_value = trigger_value
        self.requested_keys = []

    def get_key_value_by_name(self, name):
        self.requested_keys.append(name)
        return self.trigger_value


class FakeRobot:
    def __init__(self):
        self.do_writes = []

    def set_tool_do(self, index, status, smooth=0, block=1):
        self.do_writes.append((index, status, smooth, block))
        return 0


class Fr3cSuctionControllerTests(unittest.TestCase):
    def setUp(self):
        self.xr_client = FakeXrClient(trigger_value=0.0)
        self.robot = FakeRobot()
        self.controller = Fr3cSuctionController(
            xr_client=self.xr_client,
            robot=self.robot,
            controller_side="left",
            tool_do_index=0,
            trigger_threshold=0.5,
        )

    def test_toggle_on_each_press(self):
        # Released start: no rising edge, no write, still OFF.
        self.assertFalse(self.controller.update())
        self.assertEqual(self.robot.do_writes, [])

        # First press (rising edge) toggles ON.
        self.xr_client.trigger_value = 0.8
        self.assertTrue(self.controller.update())
        self.assertEqual(self.robot.do_writes, [(0, True, 0, 1)])

        # Holding the trigger does not re-trigger.
        self.xr_client.trigger_value = 1.0
        self.assertTrue(self.controller.update())
        self.assertEqual(len(self.robot.do_writes), 1)

        # Releasing does not toggle (only press edges act).
        self.xr_client.trigger_value = 0.1
        self.assertTrue(self.controller.update())
        self.assertEqual(len(self.robot.do_writes), 1)

        # Second press toggles OFF.
        self.xr_client.trigger_value = 0.9
        self.assertFalse(self.controller.update())
        self.assertEqual(len(self.robot.do_writes), 2)
        self.assertEqual(self.robot.do_writes[-1], (0, False, 0, 1))

    def test_release_forces_vacuum_off(self):
        self.xr_client.trigger_value = 1.0
        self.controller.update()
        self.assertEqual(self.robot.do_writes, [(0, True, 0, 1)])

        self.controller.release()
        self.assertEqual(self.robot.do_writes[-1], (0, False, 0, 1))
        # A second release does not duplicate the write.
        self.controller.release()
        self.assertEqual(len(self.robot.do_writes), 2)

    def test_reads_selected_side_trigger(self):
        self.controller.update()
        self.assertEqual(self.xr_client.requested_keys, ["left_trigger"])

    def test_rejects_invalid_config(self):
        with self.assertRaises(ValueError):
            Fr3cSuctionController(self.xr_client, self.robot, controller_side="up")
        with self.assertRaises(ValueError):
            Fr3cSuctionController(
                self.xr_client, self.robot, controller_side="right", tool_do_index=2
            )
        with self.assertRaises(ValueError):
            Fr3cSuctionController(
                self.xr_client, self.robot, controller_side="right", trigger_threshold=1.5
            )


if __name__ == "__main__":
    unittest.main()
