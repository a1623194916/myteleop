"""VR trigger control for a Fairino tool-side suction cup (vacuum gripper).

The suction cup is driven by a single tool digital output: DO high (vacuum on)
to grip, DO low to release. A VR trigger above a threshold produces
edge-triggered ``SetToolDO`` writes, so the XML-RPC call fires only when the
state changes, never as a continuous command stream.

This module is independent of the arm GRIP takeover: the trigger actuates the
end effector whether or not the arm is currently being teleoperated.
"""

import threading
import time


class Fr3cSuctionController:
    """Poll one VR trigger and toggle a tool DO on each trigger press."""

    def __init__(
        self,
        xr_client,
        robot,
        controller_side: str,
        tool_do_index: int = 0,
        trigger_threshold: float = 0.5,
        active_high: bool = True,
    ):
        side = controller_side.lower()
        if side not in {"left", "right"}:
            raise ValueError("controller_side must be 'left' or 'right'")
        if not 0 <= tool_do_index <= 1:
            raise ValueError("tool_do_index must be in [0, 1]")
        if not 0.0 <= trigger_threshold < 1.0:
            raise ValueError("trigger_threshold must be in [0, 1)")

        self.xr_client = xr_client
        self.robot = robot
        self.trigger_name = f"{side}_trigger"
        self.tool_do_index = int(tool_do_index)
        self.trigger_threshold = float(trigger_threshold)
        self.active_high = bool(active_high)
        self._active: bool = False
        self._prev_pressed: bool = False
        self._last_written: bool | None = None

    def _write(self, active: bool):
        # active_high=True: DO=1 means "grip" (vacuum on).
        self.robot.set_tool_do(self.tool_do_index, active if self.active_high else not active)
        self._last_written = active

    def update(self) -> bool:
        """Poll the trigger once; toggle suction on each rising press edge.

        Returns the current (active) state. A transient RPC error is logged
        so the thread stays alive; the next press re-attempts the toggle.
        """
        trigger = min(1.0, max(0.0, float(self.xr_client.get_key_value_by_name(self.trigger_name))))
        pressed = trigger > self.trigger_threshold
        if pressed and not self._prev_pressed:
            next_active = not self._active
            try:
                self._write(next_active)
            except Exception as e:
                print(f"Suction cup write failed ({e})")
            else:
                self._active = next_active
                state = "ON" if self._active else "OFF"
                print(f"Suction cup ({self.trigger_name}) DO{self.tool_do_index} -> {state}")
        self._prev_pressed = pressed
        return self._active

    def release(self):
        """Force the vacuum off (startup + shutdown). Idempotent, never raises."""
        if not self._active and self._last_written is not None:
            return
        try:
            self._write(False)
        except Exception as e:
            print(f"Suction cup release failed: {e}")
        else:
            self._active = False

    def run(self, stop_event: threading.Event, update_hz: float = 20.0):
        if update_hz <= 0.0:
            raise ValueError("update_hz must be positive")
        period = 1.0 / update_hz
        while not stop_event.is_set():
            try:
                self.update()
            except Exception as e:
                print(f"Suction thread error (kept alive): {e}")
            stop_event.wait(period)
