"""Dependency-light helpers for FR3C hardware teleoperation."""

import numpy as np


ROBOT_IP_CONTROLLER_SIDES = {
    "192.168.5.22": "left",
    "192.168.5.23": "right",
}


def resolve_controller_side(robot_ip: str, requested_side: str) -> str:
    """Resolve ``auto`` using the deployed dual-arm IP convention."""
    side = requested_side.lower()
    if side == "auto":
        return ROBOT_IP_CONTROLLER_SIDES.get(robot_ip, "right")
    if side not in {"left", "right"}:
        raise ValueError("controller_side must be one of: auto, left, right")
    return side


class PoseDeltaFilter:
    """Low-pass XR pose deltas and apply continuous radial deadbands."""

    def __init__(
        self,
        alpha: float,
        position_deadband_m: float,
        rotation_deadband_rad: float,
    ):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        if position_deadband_m < 0.0:
            raise ValueError("position_deadband_m must be non-negative")
        if rotation_deadband_rad < 0.0:
            raise ValueError("rotation_deadband_rad must be non-negative")
        self.alpha = float(alpha)
        self.position_deadband_m = float(position_deadband_m)
        self.rotation_deadband_rad = float(rotation_deadband_rad)
        self.reset()

    def reset(self):
        self._filtered_position = np.zeros(3)
        self._filtered_rotation = np.zeros(3)
        self._output_position = np.zeros(3)
        self._output_rotation = np.zeros(3)
        self._last_timestamp_ns: int | None = None

    @staticmethod
    def _validate_delta(delta: np.ndarray, name: str) -> np.ndarray:
        value = np.asarray(delta, dtype=float)
        if value.shape != (3,):
            raise ValueError(f"{name} must have shape (3,), got {value.shape}")
        return value

    @staticmethod
    def _apply_soft_deadband(value: np.ndarray, radius: float) -> np.ndarray:
        magnitude = np.linalg.norm(value)
        if magnitude <= radius:
            return np.zeros_like(value)
        return value * ((magnitude - radius) / magnitude)

    def update(
        self,
        position_delta: np.ndarray,
        rotation_delta: np.ndarray,
        timestamp_ns: int | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        position = self._validate_delta(position_delta, "position_delta")
        rotation = self._validate_delta(rotation_delta, "rotation_delta")

        has_timestamp = timestamp_ns is not None and timestamp_ns > 0
        if has_timestamp and timestamp_ns == self._last_timestamp_ns:
            return self._output_position.copy(), self._output_rotation.copy()
        if has_timestamp:
            self._last_timestamp_ns = timestamp_ns

        self._filtered_position += self.alpha * (position - self._filtered_position)
        self._filtered_rotation += self.alpha * (rotation - self._filtered_rotation)

        self._output_position = self._apply_soft_deadband(
            self._filtered_position,
            self.position_deadband_m,
        )
        self._output_rotation = self._apply_soft_deadband(
            self._filtered_rotation,
            self.rotation_deadband_rad,
        )

        return self._output_position.copy(), self._output_rotation.copy()


class JointCommandTrajectory:
    """Advance a bounded command trajectory without measured-state feedback."""

    def __init__(self, alpha: float, max_step_rad: float):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        if max_step_rad <= 0.0:
            raise ValueError("max_step_rad must be positive")
        self.alpha = float(alpha)
        self.max_step_rad = float(max_step_rad)
        self._command: np.ndarray | None = None

    def reset(self, joint_positions: np.ndarray) -> np.ndarray:
        self._command = np.asarray(joint_positions, dtype=float).copy()
        return self._command.copy()

    def advance(self, target_positions: np.ndarray) -> np.ndarray:
        if self._command is None:
            raise RuntimeError("trajectory must be reset before advance")
        target = np.asarray(target_positions, dtype=float)
        if target.shape != self._command.shape:
            raise ValueError(
                f"target shape {target.shape} does not match command shape {self._command.shape}"
            )
        step = self.alpha * (target - self._command)
        step = np.clip(step, -self.max_step_rad, self.max_step_rad)
        self._command = self._command + step
        return self._command.copy()
