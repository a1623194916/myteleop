"""Dependency-light helpers for FR3C hardware teleoperation.

All smoothing here is TIME-BASED, not per-tick: every filter derives its
blending factor from the elapsed time since the previous update, so a delayed
servo or IK tick (GIL preemption, RPC jitter) changes the phase of the stream,
never the effective velocity. This is the property that keeps the ServoJ
command stream smooth when tick timing wobbles.
"""

import math
import json
import queue
import threading
import time

import numpy as np


class DebugEventLogger:
    """Low-overhead JSONL logger for cross-thread teleop diagnostics."""

    def __init__(self, path: str, max_queue_size: int = 20000):
        self.path = str(path)
        self._queue = queue.Queue(maxsize=max_queue_size)
        self.dropped = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._write_loop, name="fr3c-debug-log", daemon=True)
        self._thread.start()

    def row(self, event: str, **fields):
        payload = {"t_monotonic": time.monotonic(), "event": event, **fields}
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            self.dropped += 1

    def _write_loop(self):
        with open(self.path, "w", encoding="utf-8") as handle:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    payload = self._queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                handle.write(json.dumps(payload, separators=(",", ":")) + "\n")
                if self._queue.qsize() == 0:
                    handle.flush()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


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


def ema_alpha_for_dt(dt_s: float, tau_s: float) -> float:
    """EMA factor that realizes the time constant ``tau_s`` for step ``dt_s``."""
    if dt_s <= 0.0:
        return 0.0
    if tau_s <= 0.0:
        return 1.0
    return 1.0 - math.exp(-dt_s / tau_s)


class OneEuroFilter:
    """Vector One Euro filter (Casiez et al. 2012).

    Adaptive low-pass for noisy human input: cutoff is ``min_cutoff_hz`` at
    rest (strong tremor suppression, the derivative term stays near zero) and
    grows with speed via ``beta`` (wide bandwidth while moving, low lag).
    """

    def __init__(
        self,
        size: int,
        min_cutoff_hz: float,
        beta: float,
        d_cutoff_hz: float = 1.0,
    ):
        if size <= 0:
            raise ValueError("size must be positive")
        if min_cutoff_hz <= 0.0 or d_cutoff_hz <= 0.0:
            raise ValueError("cutoff frequencies must be positive")
        if beta < 0.0:
            raise ValueError("beta must be non-negative")
        self.size = int(size)
        self.min_cutoff_hz = float(min_cutoff_hz)
        self.beta = float(beta)
        self.d_cutoff_hz = float(d_cutoff_hz)
        self._x_hat: np.ndarray | None = None
        self._dx_hat = np.zeros(self.size)
        self._last_t: float | None = None

    @staticmethod
    def _alpha(cutoff_hz: float | np.ndarray, dt_s: float) -> float | np.ndarray:
        tau = 1.0 / (2.0 * math.pi * np.asarray(cutoff_hz))
        return 1.0 / (1.0 + tau / dt_s)

    def reset(self):
        self._x_hat = None
        self._dx_hat = np.zeros(self.size)
        self._last_t = None

    def filter(self, x: np.ndarray, timestamp_s: float) -> np.ndarray:
        """Filter one sample stamped in a monotonic clock (seconds)."""
        x = np.asarray(x, dtype=float)
        if x.shape != (self.size,):
            raise ValueError(f"expected shape ({self.size},), got {x.shape}")
        if self._x_hat is None:
            self._x_hat = x.copy()
            self._dx_hat = np.zeros(self.size)
            self._last_t = timestamp_s
            return self._x_hat.copy()

        dt_s = timestamp_s - self._last_t
        if dt_s <= 0.0:
            return self._x_hat.copy()
        self._last_t = timestamp_s

        dx = (x - self._x_hat) / dt_s
        alpha_d = self._alpha(self.d_cutoff_hz, dt_s)
        self._dx_hat = self._dx_hat + alpha_d * (dx - self._dx_hat)

        cutoff = self.min_cutoff_hz + self.beta * np.abs(self._dx_hat)
        alpha = self._alpha(cutoff, dt_s)
        self._x_hat = self._x_hat + alpha * (x - self._x_hat)
        return self._x_hat.copy()


class PoseDeltaFilter:
    """One-Euro-filter XR pose deltas with continuous radial deadbands.

    Updates are gated on the XR sample timestamp: re-calling with the same
    timestamp (IK loop faster than the headset rate) returns the previous
    output instead of re-filtering a stale sample. Deadbands are radial and
    continuous (output magnitude = excess over the radius), so crossing the
    threshold causes no velocity jump.
    """

    def __init__(
        self,
        min_cutoff_hz: float,
        beta: float,
        position_deadband_m: float,
        rotation_deadband_rad: float,
        d_cutoff_hz: float = 1.0,
        position_jump_limit_m: float = 0.12,
        position_teleport_limit_m: float = 0.30,
        default_dt_s: float = 0.01,
    ):
        self.position_filter = OneEuroFilter(3, min_cutoff_hz, beta, d_cutoff_hz)
        self.rotation_filter = OneEuroFilter(3, min_cutoff_hz, beta, d_cutoff_hz)
        if position_deadband_m < 0.0:
            raise ValueError("position_deadband_m must be non-negative")
        if rotation_deadband_rad < 0.0:
            raise ValueError("rotation_deadband_rad must be non-negative")
        self.position_deadband_m = float(position_deadband_m)
        self.rotation_deadband_rad = float(rotation_deadband_rad)
        if position_jump_limit_m <= 0.0 or position_teleport_limit_m < position_jump_limit_m:
            raise ValueError("position jump limits must satisfy 0 < jump <= teleport")
        if default_dt_s <= 0.0:
            raise ValueError("default_dt_s must be positive")
        self.position_jump_limit_m = float(position_jump_limit_m)
        self.position_teleport_limit_m = float(position_teleport_limit_m)
        self.default_dt_s = float(default_dt_s)
        self._last_valid_position: np.ndarray | None = None
        self._rejected_positions = 0
        self.reset()

    def reset(self):
        self.position_filter.reset()
        self.rotation_filter.reset()
        self._output_position = np.zeros(3)
        self._output_rotation = np.zeros(3)
        self._last_timestamp_ns: int | None = None
        self._last_wall_s: float | None = None
        # OneEuroFilter only needs a monotonic time base. Keep a local
        # accumulated clock so an SDK timestamp of zero, a timestamp reset,
        # or a timestamp from a different clock cannot freeze the filter.
        self._filter_clock_s = 0.0
        self._last_valid_position = None
        self._rejected_positions = 0

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
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            # Tracking can briefly publish an invalid frame while the headset
            # relocalises. Hold the last valid command instead of poisoning
            # the filter state or terminating the IK thread.
            return self._output_position.copy(), self._output_rotation.copy()

        has_timestamp = timestamp_ns is not None and int(timestamp_ns) > 0
        timestamp_ns = int(timestamp_ns) if has_timestamp else None
        if has_timestamp and timestamp_ns == self._last_timestamp_ns:
            return self._output_position.copy(), self._output_rotation.copy()

        # Prefer the XR clock only while it is consecutive and increasing.
        # The SDK's fake/headless source returns zero, and some runtime paths
        # reset its epoch; both cases fall back to elapsed local time.
        now_wall_s = time.monotonic()
        if (
            has_timestamp
            and self._last_timestamp_ns is not None
            and timestamp_ns > self._last_timestamp_ns
        ):
            dt_s = (timestamp_ns - self._last_timestamp_ns) * 1e-9
        elif self._last_wall_s is not None:
            dt_s = now_wall_s - self._last_wall_s
        else:
            dt_s = self.default_dt_s
        # Reject unit mistakes (microsecond/millisecond values passed through
        # an API named ``*_ns``) that would otherwise make the One-Euro alpha
        # effectively zero and leave the arm looking stuck.
        if not math.isfinite(dt_s) or dt_s < 1e-4:
            wall_dt_s = (
                now_wall_s - self._last_wall_s
                if self._last_wall_s is not None
                else self.default_dt_s
            )
            dt_s = (
                wall_dt_s
                if math.isfinite(wall_dt_s) and wall_dt_s >= 1e-4
                else self.default_dt_s
            )
        self._filter_clock_s += dt_s
        self._last_wall_s = now_wall_s
        self._last_timestamp_ns = timestamp_ns if has_timestamp else None

        # Base-station relocalisation can produce a single large position
        # spike.  Drop impossible teleports and limit merely large samples;
        # the One-Euro filter should see a physically plausible trajectory.
        if self._last_valid_position is not None:
            delta = position - self._last_valid_position
            distance = float(np.linalg.norm(delta))
            if distance > self.position_teleport_limit_m:
                self._rejected_positions += 1
                if self._rejected_positions <= 3:
                    return self._output_position.copy(), self._output_rotation.copy()
            elif distance > self.position_jump_limit_m:
                position = self._last_valid_position + delta * (self.position_jump_limit_m / distance)
                self._rejected_positions = 0
            else:
                self._rejected_positions = 0
        self._last_valid_position = position.copy()

        filtered_position = self.position_filter.filter(position, self._filter_clock_s)
        filtered_rotation = self.rotation_filter.filter(rotation, self._filter_clock_s)

        self._output_position = self._apply_soft_deadband(
            filtered_position,
            self.position_deadband_m,
        )
        self._output_rotation = self._apply_soft_deadband(
            filtered_rotation,
            self.rotation_deadband_rad,
        )

        return self._output_position.copy(), self._output_rotation.copy()


class JointCommandTrajectory:
    """Bounded command trajectory with time-constant-based approach.

    The command approaches the latest IK target with blending factor
    ``1 - exp(-dt / tau)`` where ``dt`` is the real time since the previous
    ``advance``. Regular ticks reproduce the classic per-tick EMA with
    ``alpha = 1 - exp(-cmd_t / tau)``; irregular ticks change only the phase,
    not the velocity profile, which is what keeps the ServoJ stream smooth.
    """

    def __init__(self, tau_s: float, max_step_rad: float, default_dt_s: float = 0.01):
        if tau_s <= 0.0:
            raise ValueError("tau_s must be positive")
        if max_step_rad <= 0.0:
            raise ValueError("max_step_rad must be positive")
        if default_dt_s <= 0.0:
            raise ValueError("default_dt_s must be positive")
        self.tau_s = float(tau_s)
        self.max_step_rad = float(max_step_rad)
        self.default_dt_s = float(default_dt_s)
        self._command: np.ndarray | None = None
        self._last_now: float | None = None

    def reset(self, joint_positions: np.ndarray, now_s: float | None = None) -> np.ndarray:
        self._command = np.asarray(joint_positions, dtype=float).copy()
        # None means "clock unknown": the first advance falls back to
        # default_dt_s instead of measuring against an unrelated instant.
        self._last_now = now_s
        return self._command.copy()

    def advance(self, target_positions: np.ndarray, now_s: float | None = None) -> np.ndarray:
        if self._command is None:
            raise RuntimeError("trajectory must be reset before advance")
        target = np.asarray(target_positions, dtype=float)
        if target.shape != self._command.shape:
            raise ValueError(
                f"target shape {target.shape} does not match command shape {self._command.shape}"
            )
        now = time.monotonic() if now_s is None else now_s
        dt_s = self.default_dt_s if self._last_now is None else max(0.0, now - self._last_now)
        self._last_now = now

        step = ema_alpha_for_dt(dt_s, self.tau_s) * (target - self._command)
        step = np.clip(step, -self.max_step_rad, self.max_step_rad)
        self._command = self._command + step
        return self._command.copy()


class AbsoluteDeadlinePacer:
    """Metronomic pacing for the ServoJ stream.

    Each ``tick`` sleeps until the next absolute deadline instead of
    ``period - elapsed`` after the work: the send schedule does not accumulate
    drift. If a tick overruns (GIL stall, GC, slow RPC), the next deadline is
    restarted one full period after the actual wake-up so stale points are
    never emitted in a catch-up burst.
    """

    def __init__(self, period_s: float):
        if period_s <= 0.0:
            raise ValueError("period_s must be positive")
        self.period_s = float(period_s)
        self._deadline: float | None = None

    def reset(self, now_s: float | None = None):
        self._deadline = time.monotonic() if now_s is None else now_s

    @property
    def next_deadline_s(self) -> float | None:
        """The absolute deadline the next send is scheduled on."""
        return self._deadline

    def tick(self, now_s: float | None = None) -> float:
        """Sleep until the current deadline; return send lateness in seconds.

        Lateness is how far the wake-up happened past the deadline (0 when
        early). After a late wake-up, restart one full period after the actual
        wake-up so the loop never emits a catch-up burst.
        """
        now = time.monotonic() if now_s is None else now_s
        if self._deadline is None:
            self._deadline = now
        late_s = max(0.0, now - self._deadline)
        if now_s is None:
            remaining = self._deadline - now
            if remaining > 0.0:
                time.sleep(remaining)
                # ``sleep`` may wake late under scheduler/GIL pressure. Use
                # the actual wake time both for diagnostics and to avoid
                # scheduling a short catch-up interval after an oversleep.
                now = time.monotonic()
                late_s = max(0.0, now - self._deadline)
        if late_s > 0.0:
            self._deadline = now + self.period_s
        else:
            self._deadline += self.period_s
        return late_s

    def anchor_after_send(self, now_s: float | None = None):
        """Schedule the next tick relative to the send start."""
        now = time.monotonic() if now_s is None else now_s
        self._deadline = now + self.period_s
