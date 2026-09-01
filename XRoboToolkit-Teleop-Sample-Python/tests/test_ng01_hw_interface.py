import sys
import unittest
from pathlib import Path

import mujoco
import numpy as np

from scripts.hardware.ng01_hw import (
    GRIPPER_FINGER_MAX_RAD,
    GRIPPER_MAX_WIDTH_MM,
    NG01_ROOT,
    Ng01HwInterface,
    Ng01MockInterface,
    Ng01HwState,
    gripper_width_to_finger_rad,
    state_to_qpos,
    trigger_to_width_mm,
)
from scripts.hardware.teleop_ng01_hardware import MJCF
from scripts.hardware.teleop_ng01_vr_hardware import limit_joint_step


class Ng01GripperMappingTest(unittest.TestCase):
    def test_width_to_rad_endpoints_and_clamp(self):
        # MJCF finger: 0 = open, 1.0472 = closed; vendor width: 0 = closed,
        # 77.9 = max open -> the map is inverted
        self.assertAlmostEqual(gripper_width_to_finger_rad(GRIPPER_MAX_WIDTH_MM), 0.0)
        self.assertAlmostEqual(gripper_width_to_finger_rad(0.0), GRIPPER_FINGER_MAX_RAD)
        # out-of-range values clamp, never extrapolate
        self.assertAlmostEqual(gripper_width_to_finger_rad(-5.0), GRIPPER_FINGER_MAX_RAD)
        self.assertEqual(gripper_width_to_finger_rad(GRIPPER_MAX_WIDTH_MM + 20.0), 0.0)

    def test_trigger_to_width_mm_squeeze_closes(self):
        self.assertAlmostEqual(trigger_to_width_mm(0.0, max_width_mm=60.0), 60.0)
        self.assertAlmostEqual(trigger_to_width_mm(1.0, max_width_mm=60.0), 0.0)
        self.assertAlmostEqual(trigger_to_width_mm(0.5, max_width_mm=60.0), 30.0)
        # clamps beyond [0, 1] and caps at the hardware maximum
        self.assertAlmostEqual(trigger_to_width_mm(-0.5, max_width_mm=60.0), 60.0)
        self.assertAlmostEqual(trigger_to_width_mm(2.0, max_width_mm=500.0),
                               0.0)

    def test_mock_set_gripper_updates_state(self):
        hw = Ng01MockInterface()
        hw.set_gripper("left", 30.0)
        state = hw.read_state()
        self.assertAlmostEqual(state.left_gripper_mm, 30.0)
        self.assertEqual(hw.gripper_commands, [("left", 30.0)])


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

    def test_mock_requires_enable_and_tracks_arm_commands(self):
        hw = Ng01MockInterface()
        with self.assertRaises(RuntimeError):
            hw.send_joint_targets(left=np.ones(7))

        hw.enable_arms()
        self.assertTrue(hw.send_joint_targets(left=np.arange(7.0)))
        np.testing.assert_allclose(hw.read_state().left_joints_deg, np.arange(7.0))
        self.assertEqual(len(hw.arm_commands), 1)
        hw.stop_arms()
        self.assertFalse(hw.arms_enabled)

    def test_joint_step_limiter_clamps_each_axis(self):
        limited = limit_joint_step(
            np.array([2.0, -2.0, 0.25, -0.25, 1.0, -1.0, 0.0]),
            np.zeros(7),
            max_step_deg=0.5,
        )
        np.testing.assert_allclose(limited, [0.5, -0.5, 0.25, -0.25, 0.5, -0.5, 0.0])


class Ng01RealInterfaceCommandTest(unittest.TestCase):
    def test_partial_enable_failure_rolls_back_first_arm(self):
        class Robot:
            def __init__(self, enable_ok):
                self.enable_ok = enable_ok
                self.disabled = False

            def get_protect_status(self, **kwargs):
                return True

            def enable(self, **kwargs):
                return self.enable_ok

            def disable(self, **kwargs):
                self.disabled = True

        hw = Ng01HwInterface.__new__(Ng01HwInterface)
        hw._robots = {1: Robot(True), 2: Robot(False)}
        with self.assertRaises(RuntimeError):
            hw.enable_arms()
        self.assertTrue(hw._robots[1].disabled)

    def test_dual_target_uses_batch_command(self):
        class Robot:
            def __init__(self):
                self.params = None

            def move_joints_multi(self, params):
                self.params = params
                return True

        hw = Ng01HwInterface.__new__(Ng01HwInterface)
        hw._robots = {1: Robot(), 2: Robot()}
        self.assertTrue(hw.send_joint_targets(left=np.zeros(7), right=np.ones(7)))
        self.assertEqual([item["robot_id"] for item in hw._robots[1].params], [1, 2])
        self.assertTrue(hw._robots[1].params[0]["interpolation"])
        self.assertEqual(hw._robots[1].params[0]["specify_global_speed"], 0.0)

    def test_vendor_batch_wrapper_forwards_interpolation(self):
        if str(NG01_ROOT) not in sys.path:
            sys.path.insert(0, str(NG01_ROOT))
        from hc_robot import HCRobot

        class Param:
            pass

        class MultiParam:
            pass

        class Xoip:
            RobotActionParam = Param
            MultiRobotActionParam = MultiParam

        class RobotManager:
            captured = None

            @classmethod
            def moveJointsMulti2(cls, params):
                cls.captured = params
                return True

        robot = HCRobot.__new__(HCRobot)
        robot._xoip = Xoip
        robot._rm = RobotManager
        robot._robot_id = 1
        self.assertTrue(robot.move_joints_multi([
            {"robot_id": 1, "joints": [0.0] * 7, "interpolation": True}
        ]))
        self.assertTrue(RobotManager.captured[0].param.interpolation_en)


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
        # width 77.9mm (open) -> finger 0; width 38.95mm (half) -> finger half-closed
        self.assertAlmostEqual(
            qpos[self.model.joint("lgripper_finger_joint").qposadr[0]], 0.0
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
            # fully open -> finger joints at 0, so the whole qpos is zero
            left_gripper_mm=GRIPPER_MAX_WIDTH_MM,
            right_gripper_mm=GRIPPER_MAX_WIDTH_MM,
        )
        qpos = state_to_qpos(state, self.model, self.qpos)
        np.testing.assert_allclose(qpos, 0.0, atol=1e-12)
        data = mujoco.MjData(self.model)
        data.qpos[:] = qpos
        mujoco.mj_forward(self.model, data)
        self.assertTrue(np.isfinite(data.qpos).all())


if __name__ == "__main__":
    unittest.main()
