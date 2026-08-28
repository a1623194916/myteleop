import unittest
from pathlib import Path

import mujoco
import numpy as np
import placo

from xrobotoolkit_teleop.utils.mujoco_utils import (
    calc_mujoco_qpos_from_placo_q,
    calc_placo_q_from_mujoco_qpos,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
NG01_ROOT = REPO_ROOT.parent / "NG01_v4"
TELEOP_MJCF = NG01_ROOT / "NG01_mjcf" / "NG01_teleop.xml"
TELEOP_URDF = NG01_ROOT / "urdf" / "NG01_teleop.urdf"


class Ng01KinematicsMappingTest(unittest.TestCase):
    def setUp(self):
        self.mujoco_model = mujoco.MjModel.from_xml_path(str(TELEOP_MJCF))
        self.placo_robot = placo.RobotWrapper(str(TELEOP_URDF))

    def test_joint_configuration_round_trip_preserves_all_joints(self):
        home_qpos = self.mujoco_model.key("home").qpos.copy()

        placo_q = calc_placo_q_from_mujoco_qpos(
            self.mujoco_model,
            self.placo_robot,
            home_qpos,
            floating_base=False,
        )
        round_trip_qpos = calc_mujoco_qpos_from_placo_q(
            self.mujoco_model,
            self.placo_robot,
            placo_q,
            floating_base=False,
        )

        np.testing.assert_allclose(round_trip_qpos, home_qpos, atol=1e-9)

    def test_tcp_positions_match_between_urdf_and_mjcf_at_home(self):
        home_qpos = self.mujoco_model.key("home").qpos.copy()
        placo_q = calc_placo_q_from_mujoco_qpos(
            self.mujoco_model,
            self.placo_robot,
            home_qpos,
            floating_base=False,
        )
        self.placo_robot.state.q = placo_q
        self.placo_robot.update_kinematics()

        data = mujoco.MjData(self.mujoco_model)
        data.qpos[:] = home_qpos
        mujoco.mj_forward(self.mujoco_model, data)

        for tcp_name in ("ltcp_link", "rtcp_link"):
            body_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_BODY, tcp_name)
            mujoco_position = data.xpos[body_id]
            placo_position = self.placo_robot.get_T_world_frame(tcp_name)[:3, 3]
            np.testing.assert_allclose(mujoco_position, placo_position, atol=1e-4)


if __name__ == "__main__":
    unittest.main()
