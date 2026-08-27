import unittest

import numpy as np

from xrobotoolkit_teleop.common.fake_xr_client import FakeXrClient


class FakeXrClientTest(unittest.TestCase):
    def test_pose_and_analog_inputs_can_be_updated_independently(self):
        client = FakeXrClient()
        left_pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])

        client.set_pose("left_controller", left_pose)
        client.set_key_value("left_grip", 1.0)
        client.set_key_value("left_trigger", 0.4)

        np.testing.assert_allclose(client.get_pose_by_name("left_controller"), left_pose)
        self.assertEqual(client.get_key_value_by_name("left_grip"), 1.0)
        self.assertEqual(client.get_key_value_by_name("left_trigger"), 0.4)
        self.assertEqual(client.get_key_value_by_name("right_grip"), 0.0)

    def test_unknown_input_names_are_rejected(self):
        client = FakeXrClient()

        with self.assertRaises(ValueError):
            client.set_pose("tracker", np.zeros(7))
        with self.assertRaises(ValueError):
            client.set_key_value("A", 1.0)


if __name__ == "__main__":
    unittest.main()
