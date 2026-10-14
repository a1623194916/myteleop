"""Piper-X CAN boundary and bounded joint following for PICO teleoperation."""

import time

import numpy as np


class JointFollower:
    """Limit each commanded increment and require fresh, plausible feedback."""

    def __init__(self, lower, upper, max_step_rad: float = 0.006, max_error_rad: float = 0.2):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        if self.lower.shape != (6,) or self.upper.shape != (6,) or np.any(self.lower >= self.upper):
            raise ValueError("Expected six valid Piper-X joint limits")
        if max_step_rad <= 0 or max_error_rad <= 0:
            raise ValueError("Joint step and tracking tolerance must be positive")
        self.max_step_rad = float(max_step_rad)
        self.max_error_rad = float(max_error_rad)
        self.last_command = None

    def next(self, measured, target, active: bool):
        """Return a bounded command, or None while the deadman is released."""
        measured = np.asarray(measured, dtype=float)
        target = np.asarray(target, dtype=float)
        if measured.shape != (6,) or target.shape != (6,):
            raise ValueError("Piper-X needs exactly six joint angles")
        if not np.all(np.isfinite(measured)) or not np.all(np.isfinite(target)):
            raise ValueError("Non-finite joint feedback or IK target")
        if np.any(measured < self.lower - 0.05) or np.any(measured > self.upper + 0.05):
            raise RuntimeError("Piper-X joint feedback exceeds model limits")
        if not active:
            self.last_command = None
            return None
        if self.last_command is not None and np.max(np.abs(measured - self.last_command)) > self.max_error_rad:
            raise RuntimeError("Piper-X has stopped tracking the commanded joints")
        origin = measured if self.last_command is None else self.last_command
        command = np.clip(
            origin + np.clip(target - origin, -self.max_step_rad, self.max_step_rad),
            self.lower,
            self.upper,
        )
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
    ):
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
                    version = (probe.get_firmware() or {}).get("software_version")
                    if not version:
                        raise RuntimeError("Piper-X firmware version unavailable; check CAN or set --firmware")
                    firmware = resolve_firmware_profile("piper_x", version)
                finally:
                    probe.disconnect()
            driver = AgxArmFactory.create_arm(config(firmware))
        self.driver = driver
        try:
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

    def move_joints(self, joints):
        if self.read_only:
            raise RuntimeError("Read-only CAN session")
        self.driver.move_j([float(q) for q in joints])

    def set_gripper(self, opening: float, force: float = 1.0):
        if self.read_only:
            raise RuntimeError("Read-only CAN session")
        self.gripper.move_gripper_m(
            value=float(np.clip(opening, 0, 1)) * self.gripper_max_width,
            force=force,
        )

    def close(self):
        # disable/reset cut power and can drop a raised arm.
        self.driver.disconnect()
