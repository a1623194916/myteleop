"""Hold-B homing logic in Fr3cTeleopController (offline, fake XR + real Placo)."""
import threading
import time
import types
import unittest

import numpy as np
import placo

from xrobotoolkit_teleop.hardware.fr3c_control_utils import PoseDeltaFilter
from xrobotoolkit_teleop.hardware.fr3c_teleop_controller import (
    IK_LOOP_PERIOD,
    Fr3cTeleopController,
)

URDF = "/home/u22/kyz/pico_software/fr3c_assets/fr3c_teleop.urdf"


class FakeXrClient:
    def __init__(self, b_pressed=False, grip_value=0.0):
        self.b_pressed = b_pressed
        self.grip_value = grip_value

    def get_button_state_by_name(self, name):
        return self.b_pressed

    def get_key_value_by_name(self, name):
        return self.grip_value

    def get_pose_by_name(self, name):
        return np.zeros(7)  # [x, y, z, qx, qy, qz, qw]

    def get_timestamp_ns(self):
        return time.monotonic_ns()


class HomeLogicTests(unittest.TestCase):
    def _controller(self, b_pressed):
        ctrl = object.__new__(Fr3cTeleopController)
        ctrl.xr_client = FakeXrClient(b_pressed=b_pressed)
        ctrl.placo_robot = placo.RobotWrapper(URDF)
        nq = ctrl.placo_robot.state.q.shape[0]
        ctrl.q_lo, ctrl.q_hi = nq - 6, nq
        ctrl.placo_robot.state.q[ctrl.q_lo : ctrl.q_hi] = np.linspace(0.1, 0.9, 6)
        ctrl.placo_robot.update_kinematics()
        ctrl.target_q = ctrl.placo_robot.state.q[ctrl.q_lo : ctrl.q_hi].copy()
        ctrl.command_q = ctrl.target_q.copy()
        ctrl._state_lock = threading.Lock()
        ctrl.home_button = "B"
        ctrl.home_joint_speed_dps = 60.0
        ctrl.deadzone = 0.1
        # Normal IK path after home-exit: a solver stub that raises the
        # expected RuntimeError keeps that path a graceful no-op here.
        ctrl.solver = types.SimpleNamespace(
            solve=lambda *a: (_ for _ in ()).throw(RuntimeError("stub"))
        )
        ctrl.visualize_placo = False
        ctrl.home_q = np.deg2rad(np.array([10.0, -20.0, 30.0, -40.0, 50.0, -60.0]))
        ctrl._homing = False
        ctrl._last_home_step_t = None
        link_name = ctrl.placo_robot.frame_names()
        ctrl.manipulator_config = {"arm": {"link_name": link_name[-1],
                                           "pose_source": "right_controller",
                                           "control_trigger": "right_grip"}}
        ctrl.control_trigger_name = "right_grip"
        ctrl.scale_factor = 1.0
        ctrl.R_headset_world = np.eye(3)
        ctrl.effector_task = {"arm": types.SimpleNamespace(T_world_frame=None)}
        ctrl.init_ee_xyz = {"arm": None}
        ctrl.init_ee_quat = {"arm": None}
        ctrl.init_controller_xyz = {"arm": None}
        ctrl.init_controller_quat = {"arm": None}
        ctrl.input_pose_filter = {
            "arm": PoseDeltaFilter(min_cutoff_hz=2.0, beta=0.02,
                                   position_deadband_m=0.0, rotation_deadband_rad=0.0)
        }
        return ctrl

    def test_hold_b_glides_toward_home_at_bounded_speed(self):
        ctrl = self._controller(b_pressed=True)

        ctrl.calc_target_joint_position()  # enter + first step
        self.assertTrue(ctrl._homing)

        start = ctrl.target_q.copy()
        # Simulate a held button over ~1 s of IK ticks.
        for _ in range(int(1.0 / IK_LOOP_PERIOD)):
            ctrl.calc_target_joint_position()
            time.sleep(0.001)  # real elapsed < period; rate uses min-clamped dt

        dist_start = np.abs(ctrl.home_q - start).max()
        dist_now = np.abs(ctrl.home_q - ctrl.target_q).max()
        self.assertLess(dist_now, dist_start)
        # Speed bound: at most home_joint_speed_dps + smoothing over ~1 s.
        moved_deg = np.rad2deg(np.abs(start - ctrl.target_q).max())
        self.assertLessEqual(moved_deg, ctrl.home_joint_speed_dps * 1.1)

    def test_home_reaches_target_and_holds(self):
        ctrl = self._controller(b_pressed=True)
        deadline = time.monotonic() + 10.0  # ~112 deg max travel at 60 dps ≈ 2 s
        while (np.abs(ctrl.home_q - ctrl.target_q).max() > 1e-6
               and time.monotonic() < deadline):
            ctrl.calc_target_joint_position()
            time.sleep(0.002)
        self.assertLess(np.abs(ctrl.home_q - ctrl.target_q).max(), 1e-6)

    def test_enter_home_cancels_live_grip_session(self):
        ctrl = self._controller(b_pressed=True)
        ctrl.init_ee_xyz["arm"] = np.zeros(3)
        ctrl.init_ee_quat["arm"] = np.zeros(4)

        ctrl.calc_target_joint_position()

        self.assertIsNone(ctrl.init_ee_xyz["arm"])
        self.assertIsNone(ctrl.init_controller_xyz["arm"])

    def test_release_b_exits_home_and_reanchors_task(self):
        ctrl = self._controller(b_pressed=True)
        ctrl.calc_target_joint_position()
        self.assertTrue(ctrl._homing)

        ctrl.xr_client.b_pressed = False
        ctrl.calc_target_joint_position()  # exit branch runs, then normal IK path

        self.assertFalse(ctrl._homing)
        self.assertIsNotNone(ctrl.effector_task["arm"].T_world_frame)

    def test_disabled_button_skips_homing(self):
        ctrl = self._controller(b_pressed=True)
        ctrl.home_button = ""

        ctrl.calc_target_joint_position()

        self.assertFalse(ctrl._homing)

    def test_grip_held_suppresses_homing(self):
        # Homing must never yank an arm while it is being teleoperated.
        ctrl = self._controller(b_pressed=True)
        ctrl.xr_client.grip_value = 1.0

        ctrl.calc_target_joint_position()

        self.assertFalse(ctrl._homing)
        self.assertTrue(  # target untouched: grip branch owns the pose
            np.allclose(ctrl.target_q, ctrl.placo_robot.state.q[ctrl.q_lo : ctrl.q_hi])
        )


if __name__ == "__main__":
    unittest.main()
