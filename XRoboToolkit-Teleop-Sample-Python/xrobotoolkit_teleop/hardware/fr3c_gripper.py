"""Optional VR trigger control for a Fairino parallel gripper.

This module is deliberately not wired into the FR3C arm teleoperation entry
point. Callers must explicitly activate and run the controller.
"""

import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class GripperTarget:
    trigger: float
    closure: float
    position_percent: float
    width_mm: float


class TriggerGripperMapper:
    """Map trigger travel above a threshold to parallel-gripper closure."""

    def __init__(
        self,
        trigger_threshold: float = 0.05,
        open_position_percent: float = 100.0,
        closed_position_percent: float = 0.0,
        max_width_mm: float = 90.0,
    ):
        if not 0.0 <= trigger_threshold < 1.0:
            raise ValueError("trigger_threshold must be in [0, 1)")
        for name, value in (
            ("open_position_percent", open_position_percent),
            ("closed_position_percent", closed_position_percent),
        ):
            if not 0.0 <= value <= 100.0:
                raise ValueError(f"{name} must be in [0, 100]")
        if open_position_percent == closed_position_percent:
            raise ValueError("open and closed positions must differ")
        if max_width_mm <= 0.0:
            raise ValueError("max_width_mm must be positive")

        self.trigger_threshold = float(trigger_threshold)
        self.open_position_percent = float(open_position_percent)
        self.closed_position_percent = float(closed_position_percent)
        self.max_width_mm = float(max_width_mm)

    def map(self, trigger: float) -> GripperTarget:
        trigger = min(1.0, max(0.0, float(trigger)))
        if trigger <= self.trigger_threshold:
            closure = 0.0
        else:
            closure = (trigger - self.trigger_threshold) / (1.0 - self.trigger_threshold)

        position_percent = self.open_position_percent + closure * (
            self.closed_position_percent - self.open_position_percent
        )
        width_mm = self.max_width_mm * (1.0 - closure)
        return GripperTarget(
            trigger=trigger,
            closure=closure,
            position_percent=position_percent,
            width_mm=width_mm,
        )


class Fr3cVrGripperController:
    """Poll one VR trigger and issue rate-limited Fairino gripper targets."""

    def __init__(
        self,
        xr_client,
        robot,
        controller_side: str,
        trigger_threshold: float = 0.05,
        open_position_percent: float = 100.0,
        closed_position_percent: float = 0.0,
        max_width_mm: float = 90.0,
        min_position_change_percent: float = 2.0,
        gripper_index: int = 1,
        velocity: int = 20,
        force: int = 20,
        max_time_ms: int = 1000,
    ):
        side = controller_side.lower()
        if side not in {"left", "right"}:
            raise ValueError("controller_side must be 'left' or 'right'")
        if min_position_change_percent < 0.0:
            raise ValueError("min_position_change_percent must be non-negative")

        self.xr_client = xr_client
        self.robot = robot
        self.trigger_name = f"{side}_trigger"
        self.mapper = TriggerGripperMapper(
            trigger_threshold=trigger_threshold,
            open_position_percent=open_position_percent,
            closed_position_percent=closed_position_percent,
            max_width_mm=max_width_mm,
        )
        self.min_position_change_percent = float(min_position_change_percent)
        self.gripper_index = gripper_index
        self.velocity = velocity
        self.force = force
        self.max_time_ms = max_time_ms
        self._last_command_percent: float | None = None

    def activate(self):
        self.robot.activate_gripper(index=self.gripper_index)

    def update(self) -> GripperTarget:
        target = self.mapper.map(
            self.xr_client.get_key_value_by_name(self.trigger_name)
        )
        should_send = self._last_command_percent is None or abs(
            target.position_percent - self._last_command_percent
        ) >= self.min_position_change_percent
        if should_send:
            self.robot.move_gripper(
                target.position_percent,
                index=self.gripper_index,
                velocity=self.velocity,
                force=self.force,
                max_time_ms=self.max_time_ms,
            )
            self._last_command_percent = target.position_percent
        return target

    def run(self, stop_event: threading.Event, update_hz: float = 20.0):
        if update_hz <= 0.0:
            raise ValueError("update_hz must be positive")
        period = 1.0 / update_hz
        while not stop_event.is_set():
            self.update()
            stop_event.wait(period)
