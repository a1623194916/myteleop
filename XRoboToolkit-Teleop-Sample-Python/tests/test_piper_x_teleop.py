"""Piper-X model, deadman and CAN adapter checks without robot hardware."""

import time
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from xrobotoolkit_teleop.hardware.piper_x import JointFollower, PiperXCanArm
from xrobotoolkit_teleop.simulation.piper_x_model import (
    DEFAULT_GRIPPER_WIDTH, GRIPPER_JOINTS, JOINTS, build_scene,
)


class FakeArm:
    OPTIONS = SimpleNamespace(EFFECTOR=SimpleNamespace(AGX_GRIPPER="gripper"))

    def __init__(self):
        self.commands = []
        self.connected = False
        self.timestamp = time.time()
        self.gripper = SimpleNamespace(move_gripper_m=lambda **kwargs: self.commands.append(kwargs))

    def init_effector(self, kind):
        return self.gripper

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def enable(self):
        return True

    def set_speed_percent(self, speed):
        self.speed = speed

    def get_joint_angles(self):
        return SimpleNamespace(msg=[0.0] * 6, timestamp=self.timestamp)

    def is_connected(self):
        return self.connected

    def is_ok(self):
        return True

    def move_j(self, joints):
        self.commands.append(joints)


class PiperXTeleopTests(unittest.TestCase):
    def test_official_model_has_matching_ik_joints_and_position_actuators(self):
        urdf, scene = build_scene()
        self.assertTrue(urdf.exists())
        model = mujoco.MjModel.from_xml_path(str(scene))
        self.assertEqual(model.nu, 8)
        for name in (*JOINTS, *GRIPPER_JOINTS):
            self.assertGreaterEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name), 0)
        self.assertEqual(model.key("home").qpos.shape, (8,))
        self.assertAlmostEqual(model.key("home").qpos[6], DEFAULT_GRIPPER_WIDTH / 2)
        self.assertAlmostEqual(model.key("home").qpos[7], -DEFAULT_GRIPPER_WIDTH / 2)
        for name in ("gripper_base.stl", "gripper_link1.stl", "gripper_link2.stl"):
            self.assertTrue((urdf.parent / name).exists(), name)
        for name in ("gripper_link1", "gripper_link2"):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertGreaterEqual(body_id, 0)
            self.assertTrue(any(
                model.geom_bodyid[i] == body_id and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
                for i in range(model.ngeom)
            ))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        data.ctrl[:6] = data.qpos[:6]
        data.ctrl[6:] = 0
        for _ in range(400):
            mujoco.mj_step(model, data)
        np.testing.assert_allclose(data.qpos[6:], [0, 0], atol=0.002)
        data.ctrl[6:] = [DEFAULT_GRIPPER_WIDTH / 2, -DEFAULT_GRIPPER_WIDTH / 2]
        for _ in range(400):
            mujoco.mj_step(model, data)
        np.testing.assert_allclose(
            data.qpos[6:], [DEFAULT_GRIPPER_WIDTH / 2, -DEFAULT_GRIPPER_WIDTH / 2], atol=0.002,
        )

    def test_trigger_maps_to_official_finger_joint_signs(self):
        from xrobotoolkit_teleop.common.base_teleop_controller import BaseTeleopController

        grip = {
            "type": "parallel", "gripper_trigger": "right_trigger",
            "joint_names": list(GRIPPER_JOINTS),
            "open_pos": [DEFAULT_GRIPPER_WIDTH / 2, -DEFAULT_GRIPPER_WIDTH / 2],
            "close_pos": [0, 0],
        }
        owner = SimpleNamespace(
            manipulator_config={"hand": {"gripper_config": grip}},
            gripper_pos_target={"hand": {}},
            xr_client=SimpleNamespace(get_key_value_by_name=lambda _: 0.0),
        )
        BaseTeleopController._update_gripper_target(owner)
        np.testing.assert_allclose(list(owner.gripper_pos_target["hand"].values()), grip["open_pos"])
        owner.xr_client.get_key_value_by_name = lambda _: 1.0
        BaseTeleopController._update_gripper_target(owner)
        np.testing.assert_allclose(list(owner.gripper_pos_target["hand"].values()), grip["close_pos"])

    def test_actual_tcp_follows_moving_target_ball(self):
        from scripts.simulation.teleop_fr3c_mujoco import _install_fake_xrt

        _install_fake_xrt()
        import xrobotoolkit_sdk as sdk

        from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController
        from xrobotoolkit_teleop.simulation.piper_x_model import HOME, TCP_LINK

        hand_x = [0.0]
        sdk.get_right_controller_pose = lambda: np.array([hand_x[0], 0, 0, 0, 0, 0, 1.0])
        urdf, scene = build_scene()
        controller = MujocoTeleopController(
            str(scene), str(urdf),
            {"hand": {
                "link_name": TCP_LINK, "pose_source": "right_controller",
                "control_trigger": "right_grip", "vis_target": "target",
            }},
            scale_factor=0.5,
        )
        home = controller.solver.add_joints_task()
        home.set_joints(dict(zip(JOINTS, HOME)))
        home.configure("home", "soft", 1e-4)
        model, data = controller.mj_model, controller.mj_data
        tcp_id = model.body(TCP_LINK).id
        for step in range(421):
            if step == 100:
                hand_x[0] = 0.16  # 8 cm world-space target step after scaling.
            controller._update_ik()
            controller._update_mocap_target()
            controller._send_command()
            mujoco.mj_step(model, data)
            if step == 100:
                self.assertGreater(np.linalg.norm(data.xpos[tcp_id] - data.mocap_pos[0]), 0.06)

        self.assertLess(np.linalg.norm(data.xpos[tcp_id] - data.mocap_pos[0]), 0.015)
        self.assertEqual(data.ncon, 0)

    def test_deadman_limits_each_increment_and_reanchors_on_release(self):
        follower = JointFollower([-1] * 6, [1] * 6, max_step_rad=0.01)
        origin = np.zeros(6)
        self.assertIsNone(follower.next(origin, np.ones(6), False))
        np.testing.assert_allclose(follower.next(origin, np.ones(6), True), 0.01)
        self.assertIsNone(follower.next(origin, np.ones(6), False))
        np.testing.assert_allclose(follower.next(origin, np.ones(6), True), 0.01)
        with self.assertRaisesRegex(ValueError, "Non-finite"):
            follower.next(origin, [float("nan")] * 6, True)
        with self.assertRaisesRegex(RuntimeError, "tracking"):
            follower.next(np.ones(6), origin, True)

    def test_can_adapter_fails_closed_on_stale_feedback_and_does_not_disable(self):
        fake = FakeArm()
        arm = PiperXCanArm(driver=fake)
        self.assertEqual(fake.speed, 10)
        np.testing.assert_array_equal(arm.read_joints(), np.zeros(6))
        arm.move_joints([0.01] * 6)
        arm.set_gripper(0.5)
        self.assertAlmostEqual(fake.commands[-1]["value"], 0.035)
        fake.timestamp = time.time() - 1
        with self.assertRaisesRegex(RuntimeError, "stale"):
            arm.read_joints()
        arm.close()
        self.assertFalse(fake.connected)

    def test_read_only_does_not_enable_or_initialize_gripper(self):
        fake = FakeArm()
        def forbidden(*args, **kwargs):
            self.fail("Read-only session attempted to enable/configure hardware")
        fake.enable = forbidden
        fake.init_effector = forbidden
        fake.set_speed_percent = forbidden
        fake.is_ok = lambda: False  # Disabled motors still allow feedback reads.
        arm = PiperXCanArm(driver=fake, read_only=True)
        try:
            np.testing.assert_array_equal(arm.read_joints(), np.zeros(6))
            with self.assertRaisesRegex(RuntimeError, "Read-only"):
                arm.move_joints([0] * 6)
            with self.assertRaisesRegex(RuntimeError, "Read-only"):
                arm.set_gripper(1)
        finally:
            arm.close()


if __name__ == "__main__":
    unittest.main()
