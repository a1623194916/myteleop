import unittest
from pathlib import Path

from xrobotoolkit_teleop.common.fake_xr_client import FakeXrClient
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController


REPO_ROOT = Path(__file__).resolve().parents[1]
NG01_ROOT = REPO_ROOT.parent / "NG01_v4"
TELEOP_MJCF = NG01_ROOT / "NG01_mjcf" / "NG01_teleop.xml"
TELEOP_URDF = NG01_ROOT / "urdf" / "NG01_teleop.urdf"


class XrClientInjectionTest(unittest.TestCase):
    def test_mujoco_controller_uses_supplied_xr_client_without_initializing_sdk(self):
        fake_client = FakeXrClient()
        controller = MujocoTeleopController(
            xml_path=str(TELEOP_MJCF),
            robot_urdf_path=str(TELEOP_URDF),
            manipulator_config={},
            xr_client=fake_client,
        )

        self.assertIs(controller.xr_client, fake_client)


if __name__ == "__main__":
    unittest.main()
