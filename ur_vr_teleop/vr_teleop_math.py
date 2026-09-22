"""Dependency-light pose mapping and safety limiting for VR teleoperation."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# Translation uses the operator's reported horizontal correction.
# Keep orientation independent until its directions are verified.
T_XR_TO_ROBOT = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)

R_XR_TO_ROBOT = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)


def _rotation_from_quaternion_xyzw(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("controller quaternion must contain four finite values")
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        raise ValueError("controller quaternion has zero length")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _rotation_from_rotvec(rotvec):
    vector = np.asarray(rotvec, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _rotvec_from_rotation(rotation):
    matrix = np.asarray(rotation, dtype=float)
    cos_angle = float(np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-5:
        diagonal = np.maximum((np.diag(matrix) + 1.0) * 0.5, 0.0)
        axis = np.sqrt(diagonal)
        axis[0] = math.copysign(axis[0], matrix[2, 1] - matrix[1, 2])
        axis[1] = math.copysign(axis[1], matrix[0, 2] - matrix[2, 0])
        if np.linalg.norm(axis) < 1e-8:
            axis = np.array([1.0, 0.0, 0.0])
        return axis / np.linalg.norm(axis) * angle
    axis = np.array(
        [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]]
    ) / (2.0 * math.sin(angle))
    return axis * angle


def _limit_norm(vector, maximum):
    norm = float(np.linalg.norm(vector))
    if norm <= maximum or norm < 1e-12:
        return vector
    return vector * (maximum / norm)


def _smoothing_alpha(cutoff, dt):
    if cutoff <= 0 or dt <= 0:
        raise ValueError("cutoff and dt must be positive")
    tau = 1.0 / (2.0 * math.pi * cutoff)
    return 1.0 / (1.0 + tau / dt)


class _OneEuroScalar:
    def __init__(self, min_cutoff, beta, derivative_cutoff):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.derivative_cutoff = float(derivative_cutoff)
        self.raw = None
        self.filtered = None
        self.filtered_derivative = 0.0

    def reset(self, value):
        self.raw = float(value)
        self.filtered = float(value)
        self.filtered_derivative = 0.0

    def advance(self, value, dt):
        value = float(value)
        if self.raw is None:
            self.reset(value)
            return value
        derivative = (value - self.raw) / dt
        derivative_alpha = _smoothing_alpha(self.derivative_cutoff, dt)
        self.filtered_derivative += derivative_alpha * (
            derivative - self.filtered_derivative
        )
        cutoff = self.min_cutoff + self.beta * abs(self.filtered_derivative)
        alpha = _smoothing_alpha(cutoff, dt)
        self.filtered += alpha * (value - self.filtered)
        self.raw = value
        return self.filtered


class OneEuroPoseFilter:
    """Adaptive pose smoothing with SO(3) interpolation for orientation."""

    def __init__(self, min_cutoff=1.5, beta=0.25, derivative_cutoff=1.0):
        if min_cutoff <= 0 or beta < 0 or derivative_cutoff <= 0:
            raise ValueError("One-Euro parameters are invalid")
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.derivative_cutoff = float(derivative_cutoff)
        self.position_filters = [
            _OneEuroScalar(min_cutoff, beta, derivative_cutoff) for _ in range(3)
        ]
        self.filtered_rotation = None
        self.raw_rotation = None
        self.filtered_angular_speed = 0.0
        self.previous = None

    def reset(self, pose):
        value = np.asarray(pose, dtype=float)
        if value.shape != (6,) or not np.all(np.isfinite(value)):
            raise ValueError("pose must contain 6 finite values")
        for pose_value, position_filter in zip(value[:3], self.position_filters):
            position_filter.reset(pose_value)
        self.filtered_rotation = _rotation_from_rotvec(value[3:])
        self.raw_rotation = self.filtered_rotation.copy()
        self.filtered_angular_speed = 0.0
        self.previous = value.copy()

    def advance(self, pose, dt):
        value = np.asarray(pose, dtype=float)
        if value.shape != (6,) or not np.all(np.isfinite(value)):
            raise ValueError("pose must contain 6 finite values")
        if dt <= 0:
            raise ValueError("dt must be positive")
        if self.filtered_rotation is None:
            self.reset(value)
            return value.copy()

        position = np.array(
            [filt.advance(component, dt) for filt, component in zip(self.position_filters, value[:3])]
        )
        target_rotation = _rotation_from_rotvec(value[3:])
        raw_delta = _rotvec_from_rotation(target_rotation @ self.raw_rotation.T)
        angular_speed = float(np.linalg.norm(raw_delta)) / dt
        derivative_alpha = _smoothing_alpha(self.derivative_cutoff, dt)
        self.filtered_angular_speed += derivative_alpha * (
            angular_speed - self.filtered_angular_speed
        )
        cutoff = self.min_cutoff + self.beta * self.filtered_angular_speed
        alpha = _smoothing_alpha(cutoff, dt)
        filtered_delta = _rotvec_from_rotation(target_rotation @ self.filtered_rotation.T)
        self.filtered_rotation = (
            _rotation_from_rotvec(filtered_delta * alpha) @ self.filtered_rotation
        )
        self.raw_rotation = target_rotation
        self.previous = np.concatenate(
            [position, _rotvec_from_rotation(self.filtered_rotation)]
        )
        return self.previous.copy()


class ControllerJumpGuard:
    """Reject tracker teleports and clamp unusually large per-frame translations."""

    def __init__(self, jump_threshold=0.12, teleport_threshold=0.30):
        if jump_threshold <= 0 or teleport_threshold <= jump_threshold:
            raise ValueError("jump thresholds are invalid")
        self.jump_threshold = float(jump_threshold)
        self.teleport_threshold = float(teleport_threshold)
        self.last_valid_position = None

    def reset(self, controller_pose):
        pose = np.asarray(controller_pose, dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError("controller pose must contain 7 finite values")
        self.last_valid_position = pose[:3].copy()

    def filter(self, controller_pose):
        pose = np.asarray(controller_pose, dtype=float).copy()
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError("controller pose must contain 7 finite values")
        if self.last_valid_position is None:
            self.reset(pose)
            return pose
        delta = pose[:3] - self.last_valid_position
        distance = float(np.linalg.norm(delta))
        if distance > self.teleport_threshold:
            return None
        if distance > self.jump_threshold:
            pose[:3] = self.last_valid_position + delta * (self.jump_threshold / distance)
        self.last_valid_position = pose[:3].copy()
        return pose


class RelativePoseMapper:
    """Map controller motion relative to the GRIP press into a UR TCP target."""

    def __init__(self, scale=1.0, max_translation=0.30, max_rotation=math.pi):
        if scale <= 0 or max_translation <= 0 or max_rotation <= 0:
            raise ValueError("pose mapper limits and scale must be positive")
        self.scale = float(scale)
        self.max_translation = float(max_translation)
        self.max_rotation = float(max_rotation)
        self._anchor_controller_position = None
        self._anchor_controller_rotation = None
        self._anchor_tcp_position = None
        self._anchor_tcp_rotation = None

    @property
    def engaged(self):
        return self._anchor_tcp_position is not None

    def engage(self, robot_tcp, controller_pose):
        tcp = np.asarray(robot_tcp, dtype=float)
        pose = np.asarray(controller_pose, dtype=float)
        if (
            tcp.shape != (6,)
            or pose.shape != (7,)
            or not np.all(np.isfinite(tcp))
            or not np.all(np.isfinite(pose))
        ):
            raise ValueError("robot TCP must have 6 values and controller pose must have 7")
        self._anchor_tcp_position = tcp[:3].copy()
        self._anchor_tcp_rotation = _rotation_from_rotvec(tcp[3:])
        self._anchor_controller_position = pose[:3].copy()
        self._anchor_controller_rotation = _rotation_from_quaternion_xyzw(pose[3:])

    def release(self):
        self._anchor_tcp_position = None
        self._anchor_tcp_rotation = None
        self._anchor_controller_position = None
        self._anchor_controller_rotation = None

    def target(self, controller_pose):
        if not self.engaged:
            raise RuntimeError("pose mapper is not engaged")
        pose = np.asarray(controller_pose, dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError("controller pose must contain 7 finite values")

        delta_position = T_XR_TO_ROBOT @ (pose[:3] - self._anchor_controller_position)
        delta_position = _limit_norm(delta_position * self.scale, self.max_translation)

        current_rotation = _rotation_from_quaternion_xyzw(pose[3:])
        delta_xr = current_rotation @ self._anchor_controller_rotation.T
        delta_robot = R_XR_TO_ROBOT @ delta_xr @ R_XR_TO_ROBOT.T
        delta_rotvec = _limit_norm(_rotvec_from_rotation(delta_robot), self.max_rotation)
        target_rotation = _rotation_from_rotvec(delta_rotvec) @ self._anchor_tcp_rotation

        return np.concatenate(
            [self._anchor_tcp_position + delta_position, _rotvec_from_rotation(target_rotation)]
        )


@dataclass
class CartesianRateLimiter:
    max_linear_speed: float = 0.25
    max_angular_speed: float = 1.0
    previous: np.ndarray | None = None

    def reset(self, pose):
        value = np.asarray(pose, dtype=float)
        if value.shape != (6,):
            raise ValueError("pose must contain 6 values")
        self.previous = value.copy()

    def advance(self, target, dt):
        target = np.asarray(target, dtype=float)
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            raise ValueError("target must contain 6 finite values")
        if dt <= 0:
            raise ValueError("dt must be positive")
        if self.previous is None:
            self.reset(target)
            return target.copy()

        position_delta = _limit_norm(target[:3] - self.previous[:3], self.max_linear_speed * dt)
        previous_rotation = _rotation_from_rotvec(self.previous[3:])
        target_rotation = _rotation_from_rotvec(target[3:])
        rotation_delta = _rotvec_from_rotation(target_rotation @ previous_rotation.T)
        rotation_delta = _limit_norm(rotation_delta, self.max_angular_speed * dt)
        next_rotation = _rotation_from_rotvec(rotation_delta) @ previous_rotation
        self.previous = np.concatenate(
            [self.previous[:3] + position_delta, _rotvec_from_rotation(next_rotation)]
        )
        return self.previous.copy()


def trigger_to_gripper(trigger, open_rad=1.71, closed_rad=0.0):
    value = float(np.clip(trigger, 0.0, 1.0))
    return float(open_rad + value * (closed_rad - open_rad))
