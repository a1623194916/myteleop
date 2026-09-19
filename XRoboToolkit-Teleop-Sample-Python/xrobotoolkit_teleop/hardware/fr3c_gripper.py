"""Optional VR trigger control for a Fairino parallel gripper.

This module is deliberately not wired into the FR3C arm teleoperation entry
point. Callers must explicitly activate and run the controller.
"""

import threading
import time
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
        min_command_interval_s: float = 0.1,
        motion_done_gate: bool = False,
        gripper_index: int = 1,
        velocity: int = 20,
        force: int = 20,
        max_time_ms: int = 1000,
        recover_on_any_error: bool = False,
    ):
        side = controller_side.lower()
        if side not in {"left", "right"}:
            raise ValueError("controller_side must be 'left' or 'right'")
        if min_position_change_percent < 0.0:
            raise ValueError("min_position_change_percent must be non-negative")
        if min_command_interval_s < 0.0:
            raise ValueError("min_command_interval_s must be non-negative")

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
        self.min_command_interval_s = float(min_command_interval_s)
        self.motion_done_gate = bool(motion_done_gate)
        self._move_in_flight = False
        self.gripper_index = gripper_index
        self.velocity = velocity
        self.force = force
        self.max_time_ms = max_time_ms
        self._last_command_percent: float | None = None
        self._last_command_at_s: float | None = None
        self._retry_backoff_s = 0.5
        self._max_retry_backoff_s = 5.0
        self._next_retry_at_s = 0.0
        self._consecutive_failures = 0
        # Kept for call-site compatibility; the reactive clear in update()
        # replaced the old activation/recovery state machine.
        self._recover_on_any_error = recover_on_any_error

    def activate(self):
        self.robot.activate_gripper(index=self.gripper_index)

    def update(self) -> GripperTarget:
        target = self.mapper.map(
            self.xr_client.get_key_value_by_name(self.trigger_name)
        )
        now = time.monotonic()
        # This rig's gripper motion trips servo drive fault 8-1 ("runaway":
        # the tool-485 feedback channel never reports position, so the
        # controller's motion supervision fails at motion end), and a latched
        # fault rejects MoveGripper with 73. Clear it the moment it appears —
        # via the 8 ms state pkg, no RPC — so the next target goes through.
        main_code, sub_code = self.robot.latched_fault_code()
        if main_code > 0:
            try:
                self.robot.reset_all_errors()
                time.sleep(0.1)  # measured: reset needs ~0.1 s to take effect
            except Exception as e:
                print(f"Gripper fault clear failed ({e}); will retry next tick")
        if now < self._next_retry_at_s:
            return target
        changed = self._last_command_percent is None or abs(
            target.position_percent - self._last_command_percent
        ) >= self.min_position_change_percent
        interval_ok = (
            self._last_command_at_s is None
            or now - self._last_command_at_s >= self.min_command_interval_s
        )
        if should_send := (changed and interval_ok):
            if self.motion_done_gate and self._move_in_flight:
                check = getattr(self.robot, "get_gripper_motion_done", None)
                if check is not None and not check(self.gripper_index):
                    return target  # previous move still running; hold this frame
            try:
                self.robot.move_gripper(
                    target.position_percent,
                    index=self.gripper_index,
                    velocity=self.velocity,
                    force=self.force,
                    max_time_ms=self.max_time_ms,
                )
                self._move_in_flight = True
            except Exception as e:
                self._move_in_flight = False
                self._consecutive_failures += 1
                print(
                    f"Gripper command failed ({e}); "
                    f"backing off {self._retry_backoff_s:.1f}s"
                )
                self._next_retry_at_s = now + self._retry_backoff_s
                self._retry_backoff_s = min(
                    self._retry_backoff_s * 2.0, self._max_retry_backoff_s
                )
                return target
            self._retry_backoff_s = 0.5
            self._consecutive_failures = 0
            self._last_command_percent = target.position_percent
            self._last_command_at_s = now
        return target

    def run(self, stop_event: threading.Event, update_hz: float = 20.0):
        if update_hz <= 0.0:
            raise ValueError("update_hz must be positive")
        period = 1.0 / update_hz
        while not stop_event.is_set():
            try:
                self.update()
            except Exception as e:
                print(f"Gripper thread error (kept alive): {e}")
            stop_event.wait(period)
