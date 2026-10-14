"""Piper IK with the same pose stabilization primitives used by UR teleop."""

import sys
from pathlib import Path

import numpy as np

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController

# Share the tested math with the local UR package; retain Piper's base-frame mapping.
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from ur_vr_teleop.vr_teleop_math import (
    CartesianDeadband, CartesianRateLimiter, ControllerJumpGuard, OneEuroPoseFilter,
)


class PiperTeleopController(MujocoTeleopController):
    def __init__(self, *args, pose_filter=True, position_deadband=0.001,
                 rotation_deadband_deg=0.3, max_linear_speed=0.5,
                 max_angular_speed=1.5, oneeuro_min_cutoff=4.0, oneeuro_beta=2.0, **kwargs):
        self.pose_filter = OneEuroPoseFilter(min_cutoff=oneeuro_min_cutoff, beta=oneeuro_beta) if pose_filter else None
        self.deadband = CartesianDeadband(position_deadband, np.deg2rad(rotation_deadband_deg))
        if max_linear_speed <= 0 or max_angular_speed <= 0:
            raise ValueError("Cartesian speed limits must be positive")
        self.limiter = CartesianRateLimiter(max_linear_speed, max_angular_speed)
        self.jump_guard = ControllerJumpGuard()
        super().__init__(*args, **kwargs)

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
            output = self.limiter.advance(self.deadband.advance(target), self.dt)
        return output[:3], output[3:]
