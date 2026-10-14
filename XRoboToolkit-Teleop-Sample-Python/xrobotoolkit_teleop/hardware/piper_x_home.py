"""Saved per-arm joint poses and bounded, nonblocking return-home control."""

import json
from pathlib import Path

import numpy as np

from .piper_x import JointFollower


def load_home_targets(path, channels):
    config = json.loads(Path(path).read_text())
    if config.get("units") != "radians":
        raise ValueError("Home config must use radians")
    targets = {}
    for side, channel in channels.items():
        saved = config["arms"][side]
        if saved["channel"] != channel:
            raise ValueError(f"{side} home belongs to {saved['channel']}, not {channel}; check home config / cabling")
        target = np.asarray(saved["joint_rad"], dtype=float)
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            raise ValueError(f"Invalid six-axis home for {side}")
        targets[side] = target
    return targets


class JointHomeReturn:
    def __init__(self, side, raw_target, lower, upper, max_step_rad,
                 timeout=30.0, tolerance_rad=np.deg2rad(0.5)):
        self.side = side
        self.raw_target = np.asarray(raw_target, dtype=float)
        if self.raw_target.shape != (6,) or not np.all(np.isfinite(self.raw_target)):
            raise ValueError("Home target must contain six finite angles")
        self.target = np.clip(self.raw_target, lower, upper)
        correction = np.max(np.abs(self.target - self.raw_target))
        if correction > np.deg2rad(5):
            raise ValueError(f"{side} saved home exceeds model limits by more than 5 degrees")
        # Permit the measured boundary offset captured on this particular arm;
        # commanded positions still stay strictly inside nominal joint limits.
        self.feedback_margin = max(0.05, correction + np.deg2rad(0.1))
        self.follower = JointFollower(lower, upper, max_step_rad,
                                      feedback_limit_margin_rad=self.feedback_margin)
        self.timeout, self.tolerance = timeout, tolerance_rad
        self.returning = False
        self.blocked_until_release = False
        self.last_pressed = True  # Held startup button must be released before a request.
        self.started = None
        print(f"{side} saved home (deg): {np.rad2deg(self.raw_target).round(3)}", flush=True)
        if correction > 1e-6:
            print(f"{side} WARNING: saved pose lies outside nominal limits; bounded return target (deg): "
                  f"{np.rad2deg(self.target).round(3)}", flush=True)

    def step(self, measured, pressed, grip, now):
        edge = bool(pressed) and not self.last_pressed
        self.last_pressed = bool(pressed)
        if edge:
            if self.returning:
                self.returning = False
                self.blocked_until_release = True
                self.follower.last_command = None
                print(f"{self.side}: return home cancelled; release GRIP before takeover", flush=True)
                return None, True
            self.returning = True
            self.blocked_until_release = True
            self.started = now
            self.follower.last_command = None
            print(f"{self.side}: returning to saved home (gripper held)", flush=True)
        if self.returning:
            # Run feedback/limit checks even when already near the target.
            command = self.follower.next(measured, self.target, True)
            if np.max(np.abs(np.asarray(measured) - self.target)) <= self.tolerance:
                self.returning = False
                self.follower.last_command = None
                print(f"{self.side}: home reached; release GRIP before takeover", flush=True)
                return None, True
            if now - self.started > self.timeout:
                raise RuntimeError(f"{self.side} return home timed out; check feedback / limits")
            return command, True
        if self.blocked_until_release:
            if grip:
                return None, True
            self.blocked_until_release = False
        return None, False
