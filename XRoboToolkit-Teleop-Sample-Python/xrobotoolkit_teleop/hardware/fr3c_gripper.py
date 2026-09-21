"""Optional VR trigger control for a Fairino parallel gripper.

The FR3C entry points activate and run this controller on a separate thread.
It can also be used standalone by explicitly activating and running it.
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
    """Poll VR inputs and issue edge-triggered Fairino gripper targets.

    The trigger is intentionally a toggle, rather than a continuous position
    command: one press closes and holds the gripper, and the next press opens
    it.  This prevents trigger noise and hand tremor from constantly replacing
    an in-progress gripper command while the arm is being servoed.
    """

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
        velocity: int = 100,
        force: int = 20,
        open_force: int = 100,
        max_time_ms: int = 1000,
        toggle_button: str | None = None,
        toggle_debounce_s: float = 0.18,
        trigger_toggle: bool = True,
        recover_on_any_error: bool = False,
        skip_when_servo_busy: bool = True,
        debug_logger=None,
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
        self.open_force = int(open_force)
        self.max_time_ms = max_time_ms
        self.toggle_button = toggle_button or None
        self.toggle_debounce_s = max(0.0, float(toggle_debounce_s))
        self.trigger_toggle = bool(trigger_toggle)
        self._toggle_closed = False
        self._trigger_previous = False
        self._toggle_button_previous = False
        self._input_initialized = False
        self._last_toggle_at_s = -float("inf")
        self._last_command_percent: float | None = None
        self._last_command_at_s: float | None = None
        self._retry_backoff_s = 0.5
        self._max_retry_backoff_s = 5.0
        self._next_retry_at_s = 0.0
        self._consecutive_failures = 0
        # Kept for call-site compatibility; the reactive clear in update()
        # replaced the old activation/recovery state machine.
        self._recover_on_any_error = recover_on_any_error
        self.skip_when_servo_busy = bool(skip_when_servo_busy)
        self.skipped_busy_commands = 0
        self.debug_logger = debug_logger
        self._activated_once = False
        self._next_activation_at_s = 0.0
        self._gripper_recovery_pending = False
        self._ignore_gripper_fault_until_s = 0.0

    def activate(self):
        self.robot.activate_gripper(index=self.gripper_index)
        self._activated_once = True
        self._next_activation_at_s = 0.0

    def _gripper_fault_code(self) -> int:
        reader = getattr(self.robot, "gripper_fault_code", None)
        if reader is None:
            return 0
        try:
            return int(reader())
        except (TypeError, ValueError, RuntimeError, AttributeError):
            return -1

    def initialize_idle(self):
        """Prepare trigger state without commanding a startup gripper move."""
        self._toggle_closed = False
        if getattr(self.robot, "gripper_needs_activation", False):
            self.activate()
        now = time.monotonic()
        self._last_command_percent = self.mapper.open_position_percent
        self._last_command_at_s = now
        self._retry_backoff_s = 0.5

    def initialize_open(self):
        """Open once at teleoperation startup and establish the open baseline.

        The startup command is explicit instead of waiting for a trigger frame;
        it also uses the release force, which is independent from the safe
        closing force used when grasping an object.
        """
        self._toggle_closed = False
        # ResetAllError clears the tool activation on the affected Fairino
        # firmware.  The startup barrier may therefore have to activate the
        # gripper again before the first MoveGripper command.
        if self._gripper_fault_code() > 0:
            self.robot.reset_all_errors()
            time.sleep(0.1)
        if getattr(self.robot, "gripper_needs_activation", False):
            self.activate()
        result = self.robot.move_gripper(
            self.mapper.open_position_percent,
            index=self.gripper_index,
            velocity=self.velocity,
            force=self.open_force,
            max_time_ms=self.max_time_ms,
            lock_timeout_s=None,
        )
        if result != 0:
            raise RuntimeError(f"startup open MoveGripper returned {result}")
        now = time.monotonic()
        self._last_command_percent = self.mapper.open_position_percent
        self._last_command_at_s = now
        self._retry_backoff_s = 0.5

    def update(self) -> GripperTarget:
        # Both the trigger and optional axis-click button are edge-driven.  A
        # single sample may contain both rising edges, but it must still flip
        # the persistent state only once.
        trigger_value = float(
            self.xr_client.get_key_value_by_name(self.trigger_name)
        )
        trigger_value = min(1.0, max(0.0, trigger_value))
        now = time.monotonic()
        trigger_pressed = trigger_value > self.mapper.trigger_threshold
        trigger_rising = trigger_pressed and not self._trigger_previous
        self._trigger_previous = trigger_pressed

        button_rising = False
        if self.toggle_button:
            pressed = bool(self.xr_client.get_button_state_by_name(self.toggle_button))
            button_rising = pressed and not self._toggle_button_previous
            self._toggle_button_previous = pressed

        # Do not interpret a non-zero value already present when the program
        # starts as a user press. The operator must release and press again.
        if not self._input_initialized:
            self._input_initialized = True
            trigger_rising = False
            button_rising = False

        if self.trigger_toggle or self.toggle_button:
            if (trigger_rising or button_rising) and (
                now - self._last_toggle_at_s >= self.toggle_debounce_s
            ):
                self._toggle_closed = not self._toggle_closed
                self._last_toggle_at_s = now
                self._next_retry_at_s = 0.0
            target = GripperTarget(
                trigger=trigger_value,
                closure=1.0 if self._toggle_closed else 0.0,
                position_percent=(
                    self.mapper.closed_position_percent
                    if self._toggle_closed
                    else self.mapper.open_position_percent
                ),
                width_mm=self.mapper.max_width_mm
                * (0.0 if self._toggle_closed else 1.0),
            )
        else:
            target = self.mapper.map(trigger_value)
        if self.debug_logger and (trigger_rising or button_rising):
            self.debug_logger.row(
                "gripper_toggle", side=self.trigger_name.split("_")[0],
                trigger=target.trigger, position_percent=target.position_percent,
                closed=int(self._toggle_closed),
            )
        now = time.monotonic()
        if now < self._next_retry_at_s:
            return target

        # ResetAllError deactivates the configured gripper. Reactivate before
        # looking at gripper_fault again: on this firmware the CNDE timeout bit
        # can remain stale until ActGripper succeeds, which otherwise creates
        # a permanent fault -> reset -> wait loop and drops the button command.
        if getattr(self.robot, "gripper_needs_activation", False):
            if now < self._next_activation_at_s:
                return target
            try:
                self.activate()
            except Exception as e:
                self._next_retry_at_s = now + self._retry_backoff_s
                self._retry_backoff_s = min(
                    self._retry_backoff_s * 2.0, self._max_retry_backoff_s
                )
                print(f"Gripper re-activation failed ({e}); will retry next tick")
                return target
            self._move_in_flight = False
            self._retry_backoff_s = 0.5
            self._gripper_recovery_pending = False
            now = time.monotonic()
            self._next_retry_at_s = now + 0.2
            self._ignore_gripper_fault_until_s = now + 1.0
            return target

        # The gripper's 485 timeout is separate from the robot-level
        # main/sub-code pair.  When ServoJ is live, ask its serialized worker
        # to clear the fault; otherwise clear it here and let the next tick
        # re-activate the tool before sending another target.
        gripper_fault = self._gripper_fault_code()
        if gripper_fault <= 0:
            self._gripper_recovery_pending = False
        if gripper_fault > 0 and now >= self._ignore_gripper_fault_until_s:
            if self._gripper_recovery_pending:
                self._next_retry_at_s = now + 0.3
                return target
            self._gripper_recovery_pending = True
            if getattr(self.robot, "servo_active", False):
                request = getattr(self.robot, "request_gripper_recovery", None)
                if request is not None:
                    request()
                self._next_retry_at_s = now + 0.3
                return target
            try:
                self.robot.reset_all_errors()
                time.sleep(0.1)
                self._next_activation_at_s = now + 0.2
                self._next_retry_at_s = self._next_activation_at_s
            except Exception as e:
                print(f"Gripper fault clear failed ({e}); will retry next tick")
            return target

        # A motion timeout may also latch a robot-level 8-1 fault.  The arm
        # servo worker owns recovery while it is running.
        main_code, sub_code = self.robot.latched_fault_code()
        if main_code > 0:
            # The arm servo thread owns fault recovery while ServoJ is live.
            # A concurrent ResetAllError from this follower would contend for
            # the same XML-RPC connection and create a long ServoJ hole.
            if getattr(self.robot, "servo_active", False):
                return target
            try:
                self.robot.reset_all_errors()
                time.sleep(0.1)  # measured: reset needs ~0.1 s to take effect
                self._next_activation_at_s = now + 0.2
            except Exception as e:
                print(f"Gripper fault clear failed ({e}); will retry next tick")
                return target

        # Do not infer activation from ``gripper_active`` here.  On the
        # TG-9801 tool-board path that CNDE field is permanently zero even
        # after a successful ActGripper call.  Re-issuing ActGripper from the
        # 20 Hz follower creates repeated long RPCs and can latch drive fault
        # 8-1, which stalls the arm ServoJ stream.  Activation is performed
        # once during the startup barrier; failed moves are retried without
        # stealing the arm's realtime connection.
        # Establish the initial state without issuing an unsolicited
        # MoveGripper.  Sending an open command on the first poll is a long
        # XML-RPC operation and can contend with ServoJ; button/trigger motion
        # will produce the first real command.
        if (
            self.trigger_toggle
            and self._last_command_percent is None
            and not (trigger_rising or button_rising)
        ):
            self._last_command_percent = target.position_percent
            self._last_command_at_s = now
            return target
        changed = self._last_command_percent is None or abs(
            target.position_percent - self._last_command_percent
        ) >= self.min_position_change_percent
        interval_ok = (
            self._last_command_at_s is None
            or now - self._last_command_at_s >= self.min_command_interval_s
        )
        if changed and interval_ok:
            if self.motion_done_gate and self._move_in_flight:
                check = getattr(self.robot, "get_gripper_motion_done", None)
                if check is not None and not check(self.gripper_index):
                    return target  # previous move still running; hold this frame
            try:
                result = self.robot.move_gripper(
                    target.position_percent,
                    index=self.gripper_index,
                    velocity=self.velocity,
                    force=self.force if target.closure > 0.0 else self.open_force,
                    max_time_ms=self.max_time_ms,
                    lock_timeout_s=0.0 if self.skip_when_servo_busy else None,
                )
                if result != 0:
                    # The interface returns 1 when the shared RPC lock is
                    # busy.  Do not mark this target as sent; it will be
                    # retried after the next trigger sample.
                    self.skipped_busy_commands += 1
                    if self.debug_logger:
                        self.debug_logger.row(
                            "gripper_skipped_busy",
                            side=self.trigger_name.split("_")[0],
                            position_percent=target.position_percent,
                        )
                    return target
                self._move_in_flight = True
            except Exception as e:
                self._move_in_flight = False
                self._consecutive_failures += 1
                timeout_is_ambiguous = isinstance(e, TimeoutError)
                if timeout_is_ambiguous and (
                    self.trigger_toggle or self.toggle_button
                ):
                    # The controller may have received the request even when
                    # its reply times out. One button press must never turn
                    # into repeated MoveGripper requests.
                    self._last_command_percent = target.position_percent
                    self._last_command_at_s = now
                fault = (-1, -1)
                try:
                    fault = tuple(self.robot.latched_fault_code())
                except Exception:
                    pass
                if self._gripper_fault_code() > 0:
                    request = getattr(self.robot, "request_gripper_recovery", None)
                    if request is not None:
                        request()
                servo_active = bool(getattr(self.robot, "servo_active", False))
                print(
                    f"Gripper command failed ({e}); "
                    f"servo_active={servo_active}, fault={fault}; "
                    + (
                        "not retrying until the next toggle"
                        if timeout_is_ambiguous
                        else f"backing off {self._retry_backoff_s:.1f}s"
                    )
                )
                if self.debug_logger:
                    self.debug_logger.row(
                        "gripper_command_error",
                        side=self.trigger_name.split("_")[0],
                        error=str(e),
                        servo_active=int(servo_active),
                        fault_main=int(fault[0]),
                        fault_sub=int(fault[1]),
                    )
                self._next_retry_at_s = (
                    float("inf")
                    if timeout_is_ambiguous and (self.trigger_toggle or self.toggle_button)
                    else now + self._retry_backoff_s
                )
                self._retry_backoff_s = min(
                    self._retry_backoff_s * 2.0, self._max_retry_backoff_s
                )
                return target
            self._retry_backoff_s = 0.5
            self._consecutive_failures = 0
            self._last_command_percent = target.position_percent
            self._last_command_at_s = now
            action = (
                "CLOSE" if self._toggle_closed else "OPEN"
            ) if (self.trigger_toggle or self.toggle_button) else "TARGET"
            print(
                f"{self.trigger_name.split('_')[0]} gripper -> {action} "
                f"({target.position_percent:.0f}%)"
            )
            if self.debug_logger:
                self.debug_logger.row(
                    "gripper_command",
                    side=self.trigger_name.split("_")[0],
                    position_percent=target.position_percent,
                    closed=int(self._toggle_closed),
                )
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
