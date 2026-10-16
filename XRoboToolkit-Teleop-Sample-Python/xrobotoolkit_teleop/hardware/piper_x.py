"""Piper-X CAN boundary and bounded joint following for PICO teleoperation."""

import time

import numpy as np


class JointTrackingError(RuntimeError):
    """Recoverable tracking fault; requires a fresh clutch engagement."""


class JointFollower:
    """Limit each commanded increment and require fresh, plausible feedback."""

    def __init__(self, lower, upper, max_step_rad: float = 0.006, max_error_rad: float = 0.2,
                 feedback_limit_margin_rad: float = 0.05, tracking_timeout: float = 0.0,
                 hard_error_rad: float = 0.6):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        if self.lower.shape != (6,) or self.upper.shape != (6,) or np.any(self.lower >= self.upper):
            raise ValueError("Expected six valid Piper-X joint limits")
        if max_step_rad <= 0 or max_error_rad <= 0:
            raise ValueError("Joint step and tracking tolerance must be positive")
        self.max_step_rad = float(max_step_rad)
        self.max_error_rad = float(max_error_rad)
        self.feedback_limit_margin_rad = float(feedback_limit_margin_rad)
        self.last_command = None
        self.tracking_timeout = tracking_timeout
        self.hard_error_rad = hard_error_rad
        self.error_started = None

    def tracking_error(self, measured):
        """Largest joint deviation from the last accepted command, in radians.

        Returns 0.0 before the first accepted command on a fresh clutch.
        """
        if self.last_command is None:
            return 0.0
        return float(np.max(np.abs(np.asarray(measured, dtype=float) - self.last_command)))

    def error_within_tolerance(self, measured, limit_rad: float | None = None) -> bool:
        """True when feedback is back inside the configured tracking tolerance."""
        limit = self.max_error_rad if limit_rad is None else float(limit_rad)
        return self.tracking_error(measured) <= limit

    def reanchor(self, measured):
        """Restart the next command from fresh feedback after a tracking hold."""
        self.last_command = np.clip(np.asarray(measured, dtype=float), self.lower, self.upper)
        self.error_started = None

    def next(self, measured, target, active: bool):
        """Return a bounded command, or None while the deadman is released."""
        measured = np.asarray(measured, dtype=float)
        target = np.asarray(target, dtype=float)
        if measured.shape != (6,) or target.shape != (6,):
            raise ValueError("Piper-X needs exactly six joint angles")
        if not np.all(np.isfinite(measured)) or not np.all(np.isfinite(target)):
            raise ValueError("Non-finite joint feedback or IK target")
        if (np.any(measured < self.lower - self.feedback_limit_margin_rad)
                or np.any(measured > self.upper + self.feedback_limit_margin_rad)):
            raise RuntimeError("Piper-X joint feedback exceeds model limits")
        if not active:
            self.last_command = None
            self.error_started = None
            return None
        error = (float(np.max(np.abs(measured - self.last_command)))
                 if self.last_command is not None else 0.0)
        if self.last_command is None or error <= self.max_error_rad:
            self.error_started = None
        else:
            now = time.monotonic()
            if self.error_started is None:
                self.error_started = now
            if error > self.hard_error_rad or now - self.error_started >= self.tracking_timeout:
                raise JointTrackingError(f"Piper-X tracking error {np.rad2deg(error):.1f} deg; "
                                         f"above {np.rad2deg(self.max_error_rad):.1f} deg for "
                                         f"{now - self.error_started:.2f} s")
        origin = measured if self.last_command is None else self.last_command
        command = np.clip(
            origin + np.clip(target - origin, -self.max_step_rad, self.max_step_rad),
            self.lower,
            self.upper,
        )
        if self.tracking_timeout > 0 and np.max(np.abs(command - measured)) > self.hard_error_rad:
            raise JointTrackingError(f"Piper-X tracking target exceeds {self.hard_error_rad:g} rad "
                                     "from feedback; re-clutch required")
        self.last_command = command.copy()
        return command


class PiperXCanArm:
    """Use pyAgxArm's radian/metre API; never disable motors on disconnect."""

    def __init__(
        self,
        channel: str = "can0",
        firmware: str | None = None,
        speed_percent: int = 10,
        gripper_max_width: float = 0.07,
        driver=None,
        read_only: bool = False,
        motion_mode: str = "j",
        mit_kp: float = 10.0,
        mit_kd: float = 0.8,
    ):
        if motion_mode not in {"j", "js", "mit"}:
            raise ValueError("Piper motion mode must be j, js or mit")
        if not np.isfinite(mit_kp) or not np.isfinite(mit_kd) or not 0 < mit_kp <= 500 or not 0 < mit_kd <= 5:
            raise ValueError("MIT requires 0 < kp <= 500 and 0 < kd <= 5")
        self.mit_kp, self.mit_kd = mit_kp, mit_kd
        self.motion_mode = motion_mode
        self.channel = channel
        self.gripper_max_width = gripper_max_width
        self.gripper = None
        if driver is None:
            from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile

            def config(profile):
                return create_agx_arm_config(
                    robot="piper_x", firmeware_version=profile,
                    interface="socketcan", channel=channel, bitrate=1000000,
                )

            if firmware is None:
                probe = AgxArmFactory.create_arm(config("default"))
                try:
                    probe.connect()
                    deadline = time.monotonic() + 5
                    version = None
                    while not version and time.monotonic() < deadline:
                        version = (probe.get_firmware() or {}).get("software_version")
                        if not version:
                            time.sleep(0.1)
                    if not version:
                        raise RuntimeError("Piper-X firmware version unavailable; check CAN or set --firmware")
                    firmware = resolve_firmware_profile("piper_x", version)
                finally:
                    probe.disconnect()
            driver = AgxArmFactory.create_arm(config(firmware))
        self.driver = driver
        try:
            if motion_mode == "mit" and not callable(getattr(driver, "move_mit", None)):
                raise RuntimeError("Selected Piper-X SDK/firmware driver lacks move_mit")
            if not read_only:
                self.gripper = driver.init_effector(driver.OPTIONS.EFFECTOR.AGX_GRIPPER)
            driver.connect()
            if not read_only:
                deadline = time.monotonic() + 5
                while not driver.enable():
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Piper-X motors did not enable within 5 seconds")
                    time.sleep(0.01)
                driver.set_speed_percent(speed_percent)
            deadline = time.monotonic() + 2
            while driver.get_joint_angles() is None:
                if time.monotonic() >= deadline:
                    raise RuntimeError("Piper-X did not provide joint feedback")
                time.sleep(0.01)
        except BaseException:
            driver.disconnect()
            raise
        self.read_only = read_only

    def read_joints(self):
        if not self.driver.is_connected() or (not self.read_only and not self.driver.is_ok()):
            raise RuntimeError("Piper-X CAN connection or motor status is unhealthy")
        reading = self.driver.get_joint_angles()
        if reading is None:
            raise RuntimeError("Piper-X CAN joint feedback was lost")
        if time.time() - reading.timestamp > 0.3:
            raise RuntimeError("Piper-X joint feedback is stale (> 300 ms)")
        joints = np.asarray(reading.msg, dtype=float)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise RuntimeError("Invalid Piper-X joint feedback")
        return joints

    def move_joints(self, joints, motion_mode=None):
        if self.read_only:
            raise RuntimeError("Read-only CAN session")
        values = np.asarray(joints, dtype=float)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError("Six finite joints required before sending any motor target")
        command = values.tolist()
        selected_mode = self.motion_mode if motion_mode is None else motion_mode
        if selected_mode not in {"j", "js", "mit"}:
            raise ValueError("Piper motion mode must be j, js or mit")
        if selected_mode == "mit":
            # Use the model/firmware-specific SDK implementation, including
            # Piper-X v188 wrist sign corrections. No guessed torque feedforward.
            for index, position in enumerate(command, 1):
                self.driver.move_mit(index, p_des=position, v_des=0.0,
                                     kp=self.mit_kp, kd=self.mit_kd, t_ff=0.0)
        elif selected_mode == "js":
            self.driver.move_js(command)
        else:
            self.driver.move_j(command)

    def open_gripper(self, force: float = 5.0, timeout: float = 5.0):
        """Open to the configured stroke and verify fresh feedback before teleop."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.read_joints()  # Keep checking CAN health while waiting for the gripper.
            self.set_gripper(1.0, force=force)
            reading = self.gripper.get_gripper_status()
            if reading is not None and time.time() - reading.timestamp <= 0.3:
                msg = reading.msg
                if msg.status_code & 0x3f:
                    raise RuntimeError(f"{self.channel} gripper fault: 0x{msg.status_code:02x}")
                if (msg.mode == "width" and msg.foc_status.driver_enable_status
                        and abs(msg.value - self.gripper_max_width) <= 0.002):
                    print(f"{self.channel} gripper open confirmed: {msg.value * 1000:.1f} mm "
                          f"(target {self.gripper_max_width * 1000:.1f} mm)", flush=True)
                    return
            time.sleep(0.1)
        raise RuntimeError(f"{self.channel} gripper did not open to {self.gripper_max_width * 1000:g} mm; "
                           "check actual stroke, feedback and obstruction")

    def set_gripper(self, opening: float, force: float = 1.0):
        if self.read_only:
            raise RuntimeError("Read-only CAN session")
        self.gripper.move_gripper_m(
            value=float(np.clip(opening, 0, 1)) * self.gripper_max_width,
            force=force,
        )

    def read_gripper_width(self, max_age: float = 0.3):
        """Return (width_m, status_code, mode) or None when feedback is stale."""
        reading = self.gripper.get_gripper_status() if self.gripper is not None else None
        if reading is None or time.time() - reading.timestamp > max_age:
            return None
        msg = reading.msg
        return float(msg.value), int(msg.status_code), str(msg.mode)

    def verify_gripper_stroke(self, probe_mm: float = 8.0, settle_s: float = 2.0):
        """Prove the jaws physically move, then restore the starting width.

        ``open_gripper`` only checks that the width *reads back* as the full
        stroke. That also passes for an effector whose position feedback is
        frozen (never zeroed / not homing), which is how a dead gripper can
        silently make TRIGGER look unmapped. This probe commands a small close
        and requires the reading to follow it.

        Returns ``(moved, width_before_m, width_after_m)``.
        """
        start = self.read_gripper_width()
        if start is None:
            return False, None, None
        width_before = start[0]
        if self.gripper_max_width <= 0:
            return False, width_before, None
        target = max(0.0, width_before - probe_mm / 1000.0)
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            self.set_gripper(target / self.gripper_max_width, force=1.0)
            time.sleep(0.1)
        end = self.read_gripper_width()
        width_after = None if end is None else end[0]
        # Best-effort restore; never verify here, the caller decides what to do.
        self.set_gripper(width_before / self.gripper_max_width, force=1.0)
        moved = width_after is not None and abs(width_after - width_before) >= probe_mm * 0.25 / 1000.0
        return moved, width_before, width_after

    def close(self):
        # disable/reset cut power and can drop a raised arm.
        self.driver.disconnect()
