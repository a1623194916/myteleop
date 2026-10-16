"""Piper IK with the same pose stabilization primitives used by UR teleop."""

import math
import sys
from pathlib import Path

import numpy as np
import mujoco
from meshcat import transformations as tf

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController
from xrobotoolkit_teleop.utils.geometry import apply_delta_pose
from xrobotoolkit_teleop.hardware.piper_x_ik import ContinuousPoseIK

# Share the tested math with the local UR package; retain Piper's base-frame mapping.
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from ur_vr_teleop.vr_teleop_math import (
    CartesianDeadband, CartesianRateLimiter, ControllerJumpGuard, OneEuroPoseFilter,
)


ROTATION_LIMIT_RAD = math.pi


def scale_rotvec(rotvec, scale, maximum=ROTATION_LIMIT_RAD):
    """Scale an angle-axis delta and saturate it at the representation limit.

    ``apply_delta_pose`` hands the angle straight to ``quaternion_about_axis``,
    which does not fold it back into (-pi, pi]. A scaled norm above pi would
    therefore be read as the reflex angle and the arm would swing the other way
    at the wrap point. Saturating keeps the motion monotone: the wrist stops at
    the limit and picks up again as the hand comes back.

    Returns the scaled vector plus whether it hit the limit.
    """
    scaled = np.asarray(rotvec, dtype=float) * float(scale)
    norm = float(np.linalg.norm(scaled))
    if norm <= maximum or norm < 1e-12:
        return scaled, False
    return scaled * (maximum / norm), True


class PiperTeleopController(MujocoTeleopController):
    def __init__(self, *args, pose_filter=True, position_deadband=0.001,
                 rotation_deadband_deg=0.3, max_linear_speed=0.5,
                 max_angular_speed=1.5, oneeuro_min_cutoff=3.0, oneeuro_beta=40.0,
                 oneeuro_derivative_cutoff=10.0, oneeuro_rotation_min_cutoff=3.0,
                 oneeuro_rotation_beta=3.0, rotation_scale=1.0,
                 unlimited_motion=False, **kwargs):
        if not 0 < rotation_scale <= 3:
            raise ValueError("rotation_scale must be in (0, 3]")
        self.rotation_scale = float(rotation_scale)
        self.rotation_saturated = False
        self.unlimited_motion = unlimited_motion
        self.input_position_gap_m = 0.0
        self.pose_filter = OneEuroPoseFilter(
            min_cutoff=oneeuro_min_cutoff, beta=oneeuro_beta,
            derivative_cutoff=oneeuro_derivative_cutoff,
            rotation_min_cutoff=oneeuro_rotation_min_cutoff,
            rotation_beta=oneeuro_rotation_beta,
        ) if pose_filter else None
        self.deadband = CartesianDeadband(position_deadband, np.deg2rad(rotation_deadband_deg))
        if max_linear_speed <= 0 or max_angular_speed <= 0:
            raise ValueError("Cartesian speed limits must be positive")
        self.limiter = CartesianRateLimiter(max_linear_speed, max_angular_speed)
        self.jump_guard = ControllerJumpGuard()
        super().__init__(*args, **kwargs)
        self.pose_ik = ContinuousPoseIK(self.mj_model)
        self.last_ik_result = None

    def solve_from_feedback(self, measured, active):
        """Sample clutch once; solve only a live, engaged hand target."""
        self.mj_data.qpos[self.pose_ik.qadr] = measured
        mujoco.mj_forward(self.mj_model, self.mj_data)
        name, config = next(iter(self.manipulator_config.items()))
        self.active[name] = bool(active)
        if not active:
            if self.ref_ee_xyz[name] is not None:
                print(f"{name} is deactivated.")
            self.ref_ee_xyz[name] = self.ref_controller_xyz[name] = None
            self.pose_ik.previous = None
            self.last_ik_result = None
            return np.asarray(measured).copy()
        if self.ref_ee_xyz[name] is None:
            print(f"{name} is activated.")
            self.ref_ee_xyz[name], self.ref_ee_quat[name] = self._get_link_pose(config["link_name"])
            self.pose_ik.reset(measured)
        pose = self.xr_client.get_pose_by_name(config["pose_source"])
        xyz, rotvec = self._process_xr_pose(pose, name)
        position, quaternion = apply_delta_pose(self.ref_ee_xyz[name], self.ref_ee_quat[name], xyz, rotvec)
        target = tf.quaternion_matrix(quaternion)
        target[:3, 3] = position
        self.effector_task[name].T_world_frame = target
        self.last_ik_result = self.pose_ik.solve(target, measured)
        self.mj_data.ctrl[:6] = self.last_ik_result.joints
        return self.last_ik_result.joints.copy()

    def _process_xr_pose(self, pose, name):
        if self.ref_controller_xyz[name] is None:
            self.jump_guard.reset(pose)
            if self.pose_filter is not None:
                self.pose_filter.reset(np.zeros(6))
            self.deadband.reset(np.zeros(6))
            self.limiter.reset(np.zeros(6))
        guarded = self.jump_guard.filter(pose)
        if guarded is None:
            # Hold after a teleport; re-gripping reanchors the hand and filters.
            output = self.limiter.previous.copy()
        else:
            xyz, rotvec = super()._process_xr_pose(guarded, name)
            target = np.concatenate([xyz, rotvec])
            if self.pose_filter is not None:
                target = self.pose_filter.advance(target, self.dt)
            target = self.deadband.advance(target)
            # Scale the rotation last. The filter and the deadband are hand-side
            # conditioning, so they stay in hand units and keep the same noise
            # and jitter behaviour no matter how the hand maps onto the arm.
            target[3:], self.rotation_saturated = scale_rotvec(
                target[3:], self.rotation_scale
            )
            if self.unlimited_motion:
                output = target.copy()
                self.limiter.reset(output)  # Jump rejection still holds the last target.
            else:
                output = self.limiter.advance(target, self.dt)
            self.input_position_gap_m = float(np.linalg.norm(xyz - output[:3]))
        return output[:3], output[3:]
