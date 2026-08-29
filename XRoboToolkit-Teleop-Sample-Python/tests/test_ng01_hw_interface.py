import unittest
from pathlib import Path

import mujoco
import numpy as np

from scripts.hardware.ng01_hw import (
    GRIPPER_FINGER_MAX_RAD,
    GRIPPER_MAX_WIDTH_MM,
    Ng01MockInterface,
    Ng01HwState,
    gripper_mm_to_rad,
    state_to_qpos,
)
from scripts.hardware.teleop_ng01_hardware import MJCF


class Ng01GripperMappingTest(unittest.TestCase):
    def test_width_to_rad_endpoints_and_clamp(self):
        self.assertEqual(gripper_mm_to_rad(0.0), 0.0)
        self.assertAlmostEqual(gripper_mm_to_rad(GRIPPER_MAX_WIDTH_MM), GRIPPER_FINGER_MAX_RAD)
        # out-of-range values clamp, never extrapolate
        self.assertEqual(gripper_mm_to_rad(-5.0), 0.0)
        self.assertAlmostEqual(
            gripper_mm_to_rad(GRIPPER_MAX_WIDTH_MM + 20.0), GRIPPER_FINGER_MAX_RAD
        )


class Ng01MockInterfaceTest(unittest.TestCase):
    def test_mock_state_shapes_and_ranges(self):
        hw = Ng01MockInterface()
        state = hw.read_state()
        self.assertIsInstance(state, Ng01HwState)
        self.assertEqual(state.left_joints_deg.shape, (7,))
        self.assertEqual(state.right_joints_deg.shape, (7,))
        self.assertTrue(0.0 <= state.lift_m <= 0.33)
        for width in (state.left_gripper_mm, state.right_gripper_mm):
            self.assertTrue(0.0 <= width <= GRIPPER_MAX_WIDTH_MM)


class Ng01StateToQposTest(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(MJCF))
        self.qpos = np.zeros(self.model.nq)

    def test_known_state_maps_to_expected_qpos(self):
        state = Ng01HwState(
            left_joints_deg=np.array([10.0, -20.0, 30.0, -40.0, 50.0, -60.0, 70.0]),
            right_joints_deg=np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]),
            lift_m=0.12,
            left_gripper_mm=77.9,
            right_gripper_mm=38.95,
        )
        qpos = state_to_qpos(state, self.model, self.qpos)

        self.assertAlmostEqual(qpos[self.model.joint("up_down_joint").qposadr[0]], 0.12)
        for j in range(7):
            self.assertAlmostEqual(
                qpos[self.model.joint(f"ljoint{j + 1}").qposadr[0]],
                np.radians([10.0, -20.0, 30.0, -40.0, 50.0, -60.0, 70.0][j]),
            )
            self.assertAlmostEqual(
                qpos[self.model.joint(f"rjoint{j + 1}").qposadr[0]],
                np.radians(j + 1.0),
            )
        self.assertAlmostEqual(
            qpos[self.model.joint("lgripper_finger_joint").qposadr[0]],
            GRIPPER_FINGER_MAX_RAD,
        )
        self.assertAlmostEqual(
            qpos[self.model.joint("rgripper_finger_joint").qposadr[0]],
            GRIPPER_FINGER_MAX_RAD / 2,
        )

    def test_mapped_qpos_is_forward_kinematics_safe(self):
        # NOTE: the zero state (all joints 0deg, lift at top) is NOT the home
        # keyframe (home bends ljoint2/rjoint2 to -0.6rad and j4 to +0.3rad);
        # it is a valid straight-arm pose of its own.
        state = Ng01HwState(
            left_joints_deg=np.zeros(7),
            right_joints_deg=np.zeros(7),
            lift_m=0.0,
            left_gripper_mm=0.0,
            right_gripper_mm=0.0,
        )
        qpos = state_to_qpos(state, self.model, self.qpos)
        np.testing.assert_allclose(qpos, 0.0, atol=1e-12)
        data = mujoco.MjData(self.model)
        data.qpos[:] = qpos
        mujoco.mj_forward(self.model, data)
        self.assertTrue(np.isfinite(data.qpos).all())


if __name__ == "__main__":
    unittest.main()
