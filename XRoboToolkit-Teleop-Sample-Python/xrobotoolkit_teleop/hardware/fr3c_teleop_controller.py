"""Single FR3C arm teleoperation controller (Fairino SDK, ServoJ streaming).

Follows the official DualArmURController architecture: a dedicated servo
thread streams the last IK solution to the robot with ServoJ while the main
thread runs Placo IK at ~100 Hz, re-anchoring on the robot's ACTUAL joint
angles every cycle.  Activation/deactivation and delta-pose handling are
identical to the MuJoCo controller, so the sim and hardware feel the same.

Extend to dual arms by adding a second arm entry to manipulator_config, a
second Fr3cController + servo thread, and the dual-arm URDF q slice.
"""
import threading
import time

import meshcat.transformations as tf
import numpy as np
import placo
from placo_utils.visualization import robot_frame_viz, robot_viz

from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)

IK_LOOP_PERIOD = 0.01  # s, IK thread target period (solver.dt follows cmd_t)


class Fr3cTeleopController:
    def __init__(
        self,
        xr_client: XrClient,
        robot_urdf_path: str,
        robot_ip: str,
        initial_joint_deg: np.ndarray,
        scale_factor: float = 1.0,
        cmd_t: float = 0.01,
        tool: int = 1,
        user: int = 0,
        ee_link_name: str = "wrist3_Link",
        q_slice: tuple = (7, 13),
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        visualize_placo: bool = False,
    ):
        from xrobotoolkit_teleop.hardware.interface.fr3c import Fr3cController

        self.xr_client = xr_client
        self.robot_urdf_path = robot_urdf_path
        self.R_headset_world = R_headset_world
        self.scale_factor = scale_factor
        self.visualize_placo = visualize_placo
        self.initial_joint_rad = np.deg2rad(np.asarray(initial_joint_deg, dtype=float))
        self.q_lo, self.q_hi = q_slice

        self.arm_name = "right_arm"
        self.manipulator_config = {
            self.arm_name: {
                "link_name": ee_link_name,
                "pose_source": "right_controller",
                "control_trigger": "right_grip",
            },
        }
        self.deadzone = 0.1  # grip value above 1 - deadzone counts as active

        self.robot = Fr3cController(robot_ip=robot_ip, tool=tool, user=user, cmd_t=cmd_t)

        # Placo setup (same task layout as DualArmURController)
        self.placo_robot = placo.RobotWrapper(self.robot_urdf_path)
        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = cmd_t
        self.solver.mask_fbase(True)
        self.solver.add_kinetic_energy_regularization_task(1e-6)

        self.effector_task = {}
        self.init_ee_xyz = {}
        self.init_ee_quat = {}
        self.init_controller_xyz = {}
        self.init_controller_quat = {}
        for name, config in self.manipulator_config.items():
            initial_pose = np.eye(4)
            self.effector_task[name] = self.solver.add_frame_task(config["link_name"], initial_pose)
            self.effector_task[name].configure(f"{name}_frame", "soft", 1.0)
            manipulability = self.solver.add_manipulability_task(config["link_name"], "both", 1.0)
            manipulability.configure(f"{name}_manipulability", "soft", 5e-2)
            self.init_ee_xyz[name] = None
            self.init_ee_quat[name] = None
            self.init_controller_xyz[name] = None
            self.init_controller_quat[name] = None

        # Anchor the IK state on the ACTUAL robot joints.
        actual_q = self.robot.get_current_joint_positions()
        self.placo_robot.state.q[self.q_lo : self.q_hi] = actual_q
        self.placo_robot.update_kinematics()
        self.target_q = actual_q.copy()

        # Hold the current pose until the first grip activation (a frame task
        # left at identity would slowly drag the arm toward the origin).
        for name, config in self.manipulator_config.items():
            T_world_ee = self.placo_robot.get_T_world_frame(config["link_name"])
            self.effector_task[name].T_world_frame = T_world_ee

        if self.visualize_placo:
            self.placo_vis = robot_viz(self.placo_robot)
            self.placo_vis.display(self.placo_robot.state.q)
            for name, config in self.manipulator_config.items():
                robot_frame_viz(self.placo_robot, config["link_name"])

    def _process_xr_pose(self, xr_pose, arm_name: str):
        controller_xyz = np.array([xr_pose[0], xr_pose[1], xr_pose[2]])
        controller_quat = np.array([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]])

        controller_xyz = self.R_headset_world @ controller_xyz

        R_transform = np.eye(4)
        R_transform[:3, :3] = self.R_headset_world
        R_quat = tf.quaternion_from_matrix(R_transform)
        controller_quat = tf.quaternion_multiply(
            tf.quaternion_multiply(R_quat, controller_quat),
            tf.quaternion_conjugate(R_quat),
        )

        if self.init_controller_xyz[arm_name] is None:
            self.init_controller_xyz[arm_name] = controller_xyz.copy()
            self.init_controller_quat[arm_name] = controller_quat.copy()
            return np.zeros(3), np.zeros(3)

        delta_xyz = (controller_xyz - self.init_controller_xyz[arm_name]) * self.scale_factor
        delta_rot = quat_diff_as_angle_axis(self.init_controller_quat[arm_name], controller_quat)
        return delta_xyz, delta_rot

    def calc_target_joint_position(self):
        current_q = self.robot.get_current_joint_positions()
        self.placo_robot.state.q[self.q_lo : self.q_hi] = current_q
        self.placo_robot.update_kinematics()

        for arm_name, config in self.manipulator_config.items():
            grip_val = self.xr_client.get_key_value_by_name(config["control_trigger"])
            active = grip_val > (1.0 - self.deadzone)

            if active:
                if self.init_ee_xyz[arm_name] is None:
                    T_world_ee = self.placo_robot.get_T_world_frame(config["link_name"])
                    self.init_ee_xyz[arm_name] = T_world_ee[:3, 3].copy()
                    self.init_ee_quat[arm_name] = tf.quaternion_from_matrix(T_world_ee)
                    print(f"{arm_name} activated.")
                xr_pose = self.xr_client.get_pose_by_name(config["pose_source"])
                delta_xyz, delta_rot = self._process_xr_pose(xr_pose, arm_name)
                target_xyz, target_quat = apply_delta_pose(
                    self.init_ee_xyz[arm_name],
                    self.init_ee_quat[arm_name],
                    delta_xyz,
                    delta_rot,
                )
                target_transform = tf.quaternion_matrix(target_quat)
                target_transform[:3, 3] = target_xyz
                self.effector_task[arm_name].T_world_frame = target_transform
            else:
                if self.init_ee_xyz[arm_name] is not None:
                    print(f"{arm_name} deactivated.")
                    self.init_ee_xyz[arm_name] = None
                    self.init_ee_quat[arm_name] = None
                    self.init_controller_xyz[arm_name] = None
                    self.init_controller_quat[arm_name] = None
                    T_world_ee = self.placo_robot.get_T_world_frame(config["link_name"])
                    self.effector_task[arm_name].T_world_frame = T_world_ee

        try:
            self.solver.solve(True)
            self.target_q = self.placo_robot.state.q[self.q_lo : self.q_hi].copy()
        except RuntimeError as e:
            print(f"IK solver failed: {e}. Keeping last target.")

        if self.visualize_placo and hasattr(self, "placo_vis"):
            self.placo_vis.display(self.placo_robot.state.q)
            for name, config in self.manipulator_config.items():
                robot_frame_viz(self.placo_robot, config["link_name"])

    def reset(self):
        self.robot.reset(self.initial_joint_rad)
        self.target_q = self.robot.get_current_joint_positions().copy()
        self.placo_robot.state.q[self.q_lo : self.q_hi] = self.target_q
        self.placo_robot.update_kinematics()
        for name, config in self.manipulator_config.items():
            self.effector_task[name].T_world_frame = self.placo_robot.get_T_world_frame(
                config["link_name"]
            )

    def run_arm_thread(self, stop_event: threading.Event):
        print("Starting arm servo thread...")
        self.robot.start_servo()
        while not stop_event.is_set():
            self.robot.servo_joints(self.target_q)
            if self.robot.servo_stream_dead:
                print("Servo stream dead (persistent errors). Stopping arm thread.")
                break
        self.robot.stop_servo()

    def run_ik_thread(self, stop_event: threading.Event):
        print("Starting IK thread...")
        while not stop_event.is_set():
            start = time.monotonic()
            self.calc_target_joint_position()
            elapsed = time.monotonic() - start
            time.sleep(max(0.0, IK_LOOP_PERIOD - elapsed))

    def close(self):
        self.robot.close()
