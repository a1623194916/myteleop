import numpy as np


class FakeXrClient:
    """In-memory XR input source for deterministic simulation tests."""

    POSE_NAMES = {"left_controller", "right_controller", "headset"}
    KEY_NAMES = {"left_trigger", "right_trigger", "left_grip", "right_grip"}
    BUTTON_NAMES = {"A", "B", "X", "Y"}

    def __init__(self):
        identity_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self._poses = {name: identity_pose.copy() for name in self.POSE_NAMES}
        self._key_values = {name: 0.0 for name in self.KEY_NAMES}
        self._buttons = {name: False for name in self.BUTTON_NAMES}

    def set_pose(self, name: str, pose: np.ndarray) -> None:
        if name not in self.POSE_NAMES:
            raise ValueError(f"Invalid pose source: {name}")
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (7,):
            raise ValueError("Pose must contain [x, y, z, qx, qy, qz, qw].")
        self._poses[name] = pose.copy()

    def set_key_value(self, name: str, value: float) -> None:
        if name not in self.KEY_NAMES:
            raise ValueError(f"Invalid analog input: {name}")
        if not 0.0 <= value <= 1.0:
            raise ValueError("Analog input must be in [0, 1].")
        self._key_values[name] = float(value)

    def get_pose_by_name(self, name: str) -> np.ndarray:
        if name not in self.POSE_NAMES:
            raise ValueError(f"Invalid pose source: {name}")
        return self._poses[name].copy()

    def get_key_value_by_name(self, name: str) -> float:
        if name not in self.KEY_NAMES:
            raise ValueError(f"Invalid analog input: {name}")
        return self._key_values[name]

    def set_button_state(self, name: str, pressed: bool) -> None:
        if name not in self.BUTTON_NAMES:
            raise ValueError(f"Invalid button: {name}")
        self._buttons[name] = bool(pressed)

    def get_button_state_by_name(self, name: str) -> bool:
        if name not in self.BUTTON_NAMES:
            raise ValueError(f"Invalid button: {name}")
        return self._buttons[name]

    def get_motion_tracker_data(self) -> dict:
        return {}

    def close(self) -> None:
        pass
