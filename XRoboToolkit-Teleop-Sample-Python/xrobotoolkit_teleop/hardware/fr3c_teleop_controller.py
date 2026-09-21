"""Single FR3C arm teleoperation controller (Fairino SDK, ServoJ streaming).

Follows the official DualArmURController architecture: a dedicated servo
thread streams the last IK solution to the robot with ServoJ while another
thread runs Placo IK at ~100 Hz. The IK state advances from the last command
instead of delayed measured state, keeping the ServoJ trajectory continuous.

Extend to dual arms by adding a second arm entry to manipulator_config, a
second Fr3cController + servo thread, and the dual-arm URDF q slice.
"""
import os
import sys
import threading
import time

import meshcat.transformations as tf
import numpy as np
import placo
from placo_utils.visualization import robot_frame_viz, robot_viz

from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.hardware.fr3c_control_utils import (
    DebugEventLogger,
    JointCommandTrajectory,
    PoseDeltaFilter,
    resolve_controller_side,
)
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
        servo_transport: str = "udp",
        tool: int = 1,
        user: int = 0,
        ee_link_name: str = "wrist3_Link",
        q_slice: tuple = (7, 13),
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        visualize_placo: bool = False,
        smooth_tau_s: float = 0.06,
        max_joint_step_deg: float = 0.30,
        controller_side: str = "auto",
        input_min_cutoff_hz: float = 1.5,
        input_beta: float = 0.08,
        position_deadband_mm: float = 1.5,
        rotation_deadband_deg: float = 0.5,
        home_button: str = "B",
        home_joint_speed_dps: float = 60.0,
        home_q_deg: list[float] | None = None,
        debug_csv_path: str | None = None,
        grip_activate_threshold: float = 0.90,
        grip_release_threshold: float = 0.75,
        grip_debounce_ms: float = 80.0,
    ):
        from xrobotoolkit_teleop.hardware.interface.fr3c import Fr3cController

        # Let the servo thread preempt the IK/render threads faster than the
        # 5 ms default GIL switch interval.
        sys.setswitchinterval(0.001)

        self.xr_client = xr_client
        self.debug_logger = DebugEventLogger(debug_csv_path) if debug_csv_path else None
        self.robot_urdf_path = robot_urdf_path
        self.R_headset_world = R_headset_world
        self.scale_factor = scale_factor
        self.visualize_placo = visualize_placo
        self.initial_joint_rad = np.deg2rad(np.asarray(initial_joint_deg, dtype=float))
        self.q_lo, self.q_hi = q_slice
        self.controller_side = resolve_controller_side(robot_ip, controller_side)
        self.arm_name = f"{self.controller_side}_arm"
        self.manipulator_config = {
            self.arm_name: {
                "link_name": ee_link_name,
                "pose_source": f"{self.controller_side}_controller",
                "control_trigger": f"{self.controller_side}_grip",
            },
        }
        print(
            f"Controller side: {self.controller_side} "
            f"(robot {robot_ip}, requested {controller_side})"
        )
        self.deadzone = 0.1  # grip value above 1 - deadzone counts as active
        if not 0.0 <= grip_release_threshold < grip_activate_threshold <= 1.0:
            raise ValueError("grip thresholds must satisfy 0 <= release < activate <= 1")
        self.grip_activate_threshold = float(grip_activate_threshold)
        self.grip_release_threshold = float(grip_release_threshold)
        if grip_debounce_ms < 0.0:
            raise ValueError("grip_debounce_ms must be non-negative")
        self.grip_debounce_s = float(grip_debounce_ms) / 1000.0
        self._grip_active = False
        self._grip_candidate: bool | None = None
        self._grip_candidate_since = 0.0

        self.robot = Fr3cController(
            robot_ip=robot_ip,
            tool=tool,
            user=user,
            cmd_t=cmd_t,
            sdk_namespace=self.controller_side,
            servo_transport=servo_transport,
        )

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
        self.input_pose_filter = {}
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
            self.input_pose_filter[name] = PoseDeltaFilter(
                min_cutoff_hz=input_min_cutoff_hz,
                beta=input_beta,
                position_deadband_m=position_deadband_mm / 1000.0,
                rotation_deadband_rad=np.deg2rad(rotation_deadband_deg),
                default_dt_s=cmd_t,
            )

        self.control_trigger_name = next(iter(self.manipulator_config.values()))[
            "control_trigger"
        ]

        print(
            "XR input filter: "
            f"one-euro min_cutoff={input_min_cutoff_hz} Hz, beta={input_beta}, "
            f"position_deadband={position_deadband_mm} mm, "
            f"rotation_deadband={rotation_deadband_deg} deg"
        )

        # Measured joints initialize the command once. During ServoJ streaming,
        # Placo advances from the last command to avoid delayed feedback.
        actual_q = self.robot.get_current_joint_positions()
        self.placo_robot.state.q[self.q_lo : self.q_hi] = actual_q
        self.placo_robot.update_kinematics()
        self.target_q = actual_q.copy()
        self.command_q = actual_q.copy()
        self._state_lock = threading.Lock()
        # Hold-button homing: while the button is down the IK target glides
        # toward the fixed home pose (per-arm, captured on 2026-09-18 and
        # overridable via --home-q-left/right-deg) at a bounded joint speed.
        # Suppressed while the arm's grip is held, so homing can never yank
        # an arm mid-teleoperation.
        self.home_button = home_button
        self.home_joint_speed_dps = float(home_joint_speed_dps)
        if home_q_deg is not None:
            if len(home_q_deg) != 6:
                raise ValueError("home_q_deg must list 6 joint angles in degrees")
            self.home_q = np.deg2rad(np.asarray(home_q_deg, dtype=float))
        else:
            self.home_q = actual_q.copy()
        print(f"Home pose ({self.controller_side} arm, deg): "
              f"{[round(float(v), 3) for v in np.rad2deg(self.home_q)]}")
        self._homing = False
        self._last_home_step_t: float | None = None
        self.command_trajectory = JointCommandTrajectory(
            tau_s=smooth_tau_s,
            max_step_rad=np.deg2rad(max_joint_step_deg),
            default_dt_s=cmd_t,
        )
        self.command_trajectory.reset(actual_q)

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
            self.input_pose_filter[arm_name].reset()
            return np.zeros(3), np.zeros(3)

        delta_xyz = controller_xyz - self.init_controller_xyz[arm_name]
        delta_rot = quat_diff_as_angle_axis(self.init_controller_quat[arm_name], controller_quat)
        delta_xyz, delta_rot = self.input_pose_filter[arm_name].update(
            delta_xyz,
            delta_rot,
            timestamp_ns=self.xr_client.get_timestamp_ns(),
        )
        if self.debug_logger:
            self.debug_logger.row(
                "xr_delta", side=self.controller_side,
                xyz=delta_xyz.tolist(), rotation=delta_rot.tolist(),
            )
        return delta_xyz * self.scale_factor, delta_rot

    def _cancel_grip_sessions(self):
        """Drop any live grip takeover so the next grip re-anchors cleanly."""
        for arm_name, config in self.manipulator_config.items():
            if self.init_ee_xyz[arm_name] is not None:
                print(f"{arm_name} deactivated.")
                self.init_ee_xyz[arm_name] = None
                self.init_ee_quat[arm_name] = None
                self.init_controller_xyz[arm_name] = None
                self.init_controller_quat[arm_name] = None
                self.input_pose_filter[arm_name].reset()
                T_world_ee = self.placo_robot.get_T_world_frame(config["link_name"])
                self.effector_task[arm_name].T_world_frame = T_world_ee

    def _enter_home(self):
        self._homing = True
        self._last_home_step_t = None
        self._cancel_grip_sessions()
        print(f"Homing: hold {self.home_button} to keep moving to the initial pose.")

    def _exit_home(self):
        self._homing = False
        with self._state_lock:
            q = self.target_q.copy()
        self.placo_robot.state.q[self.q_lo : self.q_hi] = q
        self.placo_robot.update_kinematics()
        for name, config in self.manipulator_config.items():
            self.effector_task[name].T_world_frame = self.placo_robot.get_T_world_frame(
                config["link_name"]
            )
        print("Home released; holding pose (re-grip to take over).")

    def _update_grip_state(self, value: float) -> bool:
        """Apply hysteresis plus a short time debounce to the VR grip input."""
        value = float(value)
        # A few unit/integration fakes construct the controller without the
        # full initializer; keep this helper robust for those callers.
        active = bool(getattr(self, "_grip_active", False))
        candidate = getattr(self, "_grip_candidate", None)
        activate_threshold = float(getattr(self, "grip_activate_threshold", 0.90))
        release_threshold = float(getattr(self, "grip_release_threshold", 0.75))
        requested = (
            value >= release_threshold
            if active
            else value >= activate_threshold
        )
        # Clearly held/released values do not need an 80 ms confirmation.
        strong_transition = (
            (not active and value >= activate_threshold + 0.05)
            or (active and value <= release_threshold - 0.05)
        )
        if requested == active:
            self._grip_candidate = None
            self._grip_active = active
            return active
        now = time.monotonic()
        if strong_transition or getattr(self, "grip_debounce_s", 0.0) <= 0.0:
            active = requested
            self._grip_candidate = None
            self._grip_active = active
            return active
        if candidate != requested:
            self._grip_candidate = requested
            self._grip_candidate_since = now
        elif now - getattr(self, "_grip_candidate_since", now) >= self.grip_debounce_s:
            active = requested
            self._grip_active = active
            self._grip_candidate = None
        return active

    def _advance_home_target(self):
        """Move the IK target toward the initial pose at a bounded joint speed.

        Time-based (real elapsed since the previous tick), so GIL jitter
        changes the phase but not the homing speed, matching the rest of the
        command-chain filters.
        """
        now = time.monotonic()
        if self._last_home_step_t is None:
            dt = IK_LOOP_PERIOD
        else:
            dt = min(0.1, max(0.0, now - self._last_home_step_t))
        self._last_home_step_t = now
        with self._state_lock:
            target = self.target_q.copy()
        max_step = np.deg2rad(self.home_joint_speed_dps) * dt
        homed = target + np.clip(self.home_q - target, -max_step, max_step)
        self.placo_robot.state.q[self.q_lo : self.q_hi] = homed
        self.placo_robot.update_kinematics()
        with self._state_lock:
            self.target_q = homed

    def calc_target_joint_position(self):
        grip_value = self.xr_client.get_key_value_by_name(self.control_trigger_name)
        grip_held = self._update_grip_state(grip_value)
        if (
            self.home_button
            and not grip_held
            and self.xr_client.get_button_state_by_name(self.home_button)
        ):
            if not self._homing:
                self._enter_home()
            self._advance_home_target()
            return
        if self._homing:
            self._exit_home()

        with self._state_lock:
            command_q = self.command_q.copy()
        self.placo_robot.state.q[self.q_lo : self.q_hi] = command_q
        self.placo_robot.update_kinematics()

        for arm_name, config in self.manipulator_config.items():
            # ``control_trigger_name`` is the same key for the single-arm
            # controller. Reuse the sample read above so one IK tick cannot
            # observe two different grip values and so the XR SDK is not
            # polled twice at 100 Hz.
            if config["control_trigger"] == self.control_trigger_name:
                grip_val = grip_value
                active = grip_held
            else:
                grip_val = self.xr_client.get_key_value_by_name(config["control_trigger"])
                active = self._update_grip_state(grip_val)

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
                    self.input_pose_filter[arm_name].reset()
                    T_world_ee = self.placo_robot.get_T_world_frame(config["link_name"])
                    self.effector_task[arm_name].T_world_frame = T_world_ee

        # A released grip means this arm is intentionally holding its current
        # pose.  Solving Placo against a soft frame task while inactive can
        # still move the redundant joints toward the URDF's preferred
        # configuration (observed as a several-degree drift on the left arm),
        # even though the controller is not being teleoperated.  That drift
        # eventually trips Fairino drive fault 8-1.  Keep the last ServoJ
        # command exactly until the operator takes over again.
        if not any(self.init_ee_xyz[name] is not None for name in self.effector_task):
            with self._state_lock:
                held_q = self.command_q.copy()
                self.target_q = held_q.copy()
            self.placo_robot.state.q[self.q_lo : self.q_hi] = held_q
            self.placo_robot.update_kinematics()
            if getattr(self, "debug_logger", None):
                self.debug_logger.row(
                    "ik_target", side=self.controller_side,
                    target_q_rad=held_q.tolist(), grip=float(grip_val), held=True,
                )
            return

        try:
            self.solver.solve(True)
            target_q = self.placo_robot.state.q[self.q_lo : self.q_hi].copy()
            with self._state_lock:
                self.target_q = target_q
            if self.debug_logger:
                self.debug_logger.row(
                    "ik_target", side=self.controller_side,
                    target_q_rad=target_q.tolist(),
                    grip=float(grip_val),
                )
        except RuntimeError as e:
            print(f"IK solver failed: {e}. Keeping last target.")

        if self.visualize_placo and hasattr(self, "placo_vis"):
            self.placo_vis.display(self.placo_robot.state.q)
            for name, config in self.manipulator_config.items():
                robot_frame_viz(self.placo_robot, config["link_name"])

    def reset(self):
        self.robot.reset(self.initial_joint_rad)
        reset_q = self.robot.get_current_joint_positions().copy()
        with self._state_lock:
            self.target_q = reset_q.copy()
            self.command_q = reset_q.copy()
        self.command_trajectory.reset(reset_q)
        self.placo_robot.state.q[self.q_lo : self.q_hi] = reset_q
        self.placo_robot.update_kinematics()
        for name, config in self.manipulator_config.items():
            self.effector_task[name].T_world_frame = self.placo_robot.get_T_world_frame(
                config["link_name"]
            )

    @staticmethod
    def _raise_thread_priority():
        """Best-effort SCHED_FIFO for the calling thread. Needs CAP_SYS_NICE
        (e.g. sudo); silently ignored when unprivileged."""
        try:
            param = os.sched_param(os.sched_get_priority_min(os.SCHED_FIFO) + 10)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            print("Servo thread running with SCHED_FIFO priority.")
        except (AttributeError, OSError):
            pass

    def run_arm_thread(
        self,
        stop_event: threading.Event,
        initial_delay_s: float = 0.0,
    ):
        print("Starting arm servo thread...")
        self._raise_thread_priority()
        try:
            if initial_delay_s > 0.0 and stop_event.wait(initial_delay_s):
                return
            self.robot.start_servo()
            while not stop_event.is_set():
                with self._state_lock:
                    target_q = self.target_q.copy()
                q_cmd = self.command_trajectory.advance(target_q)
                self.robot.servo_joints(q_cmd)
                if self.debug_logger:
                    measured = self.robot.get_realtime_joint_positions()
                    self.debug_logger.row(
                        "servoj", side=self.controller_side,
                        command_q_rad=q_cmd.tolist(),
                        measured_q_rad=None if measured is None else measured.tolist(),
                        send_late_ms=self.robot.last_send_late_s * 1000.0,
                        rpc_ms=self.robot.last_servo_rpc_ms,
                        servo_error=self.robot.last_servo_error,
                        stream_dead=self.robot.servo_stream_dead,
                        fault_code=list(self.robot.realtime_fault_code),
                        collision_state=self.robot.realtime_collision_state,
                        collision_level=self.robot.realtime_collision_level,
                    )
                with self._state_lock:
                    self.command_q = q_cmd
                if self.robot.servo_stream_dead:
                    print("Servo stream dead (persistent errors). Stopping arm thread.")
                    stop_event.set()
                    break
        except Exception as e:
            print(f"Arm servo thread failed: {e}")
            stop_event.set()
        finally:
            self.robot.stop_servo()

    def run_ik_thread(
        self,
        stop_event: threading.Event,
        initial_delay_s: float = 0.0,
    ):
        print("Starting IK thread...")
        try:
            if initial_delay_s > 0.0 and stop_event.wait(initial_delay_s):
                return
            while not stop_event.is_set():
                start = time.monotonic()
                self.calc_target_joint_position()
                elapsed = time.monotonic() - start
                # A released arm is held exactly at command_q and does not
                # need a 100 Hz Placo/XR poll.  Reducing this idle path to
                # 20 Hz prevents two inactive IK threads from competing with
                # the dual ServoJ senders for the GIL.  Grip takeover returns
                # to the full IK rate immediately on the next iteration.
                active = self._homing or any(
                    value is not None for value in self.init_ee_xyz.values()
                )
                period = IK_LOOP_PERIOD if active else 0.05
                stop_event.wait(max(0.0, period - elapsed))
        except Exception as e:
            print(f"IK thread failed: {e}")
            stop_event.set()

    def close(self):
        self.robot.close()
        if self.debug_logger:
            self.debug_logger.close()
