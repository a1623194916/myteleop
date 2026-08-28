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
