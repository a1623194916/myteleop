# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unified configuration for the PiperX teleop data recorder.

Pure logic (no hardware imports): a dataclass tree plus YAML load/save. One
config file holds every tunable (fps, image size, camera serials, CAN/gripper
params, task name, the predefined subtask list, output dir, and the
teleop/gripper button mapping). The resolved config is written next to each
recording session (``save``) for provenance.

Uses PyYAML (available in the picosoftware venv and the local repo venv).
"""

from dataclasses import asdict, dataclass, field, fields, is_dataclass
from typing import Any

import yaml


# --------------------------------------------------------------------------- #
# Nested config groups
# --------------------------------------------------------------------------- #
@dataclass
class CameraConfig:
    """Orbbec stream (served by the Jetson ``stream_server`` over ZMQ)."""

    host: str = "192.168.5.27"
    port: int = 8765
    timeout_ms: int = 300
    width: int = 640
    height: int = 480
    serials: list[str] = field(
        default_factory=lambda: ["CV2R46P00097", "CPC85630003C"]
    )


@dataclass
class CanConfig:
    """PiperX CAN boundary and gripper parameters."""

    channel: str = "can0"
    firmware: Any = None  # None -> auto-probe firmware profile
    speed_percent: int = 10
    gripper_max_width: float = 0.07


@dataclass
class ControlConfig:
    """Teleop control-thread tunables (MuJoCo IK + JointFollower)."""

    rate_hz: float = 20.0
    max_joint_step_deg: float = 0.35
    scale_factor: float = 0.5
    deadman_threshold: float = 0.9  # right_grip value above this engages teleop
    poll_rate_hz: float = 20.0  # button edge-detection poll rate


@dataclass
class ButtonConfig:
    """Controller button / axis mapping (confirmed requirements)."""

    record_toggle: str = "A"  # right A: toggle start/stop episode recording
    subtask_mark: str = "X"  # left X: advance subtask index
    deadman_key: str = "right_grip"  # hold to engage arm teleop
    gripper_key: str = "right_trigger"  # gripper opening = 1 - clip(trigger, 0, 1)


# --------------------------------------------------------------------------- #
# Top-level config
# --------------------------------------------------------------------------- #
@dataclass
class RecorderConfig:
    task: str = "piper_reach"
    subtasks: list[str] = field(
        default_factory=lambda: ["approach", "grasp", "lift", "place"]
    )
    output_dir: str = "/home/u22/kyz/datasets"
    fps: float = 30.0
    picosoftware_root: str = "/home/u22/kyz/pico_software"
    camera: CameraConfig = field(default_factory=CameraConfig)
    can: CanConfig = field(default_factory=CanConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    buttons: ButtonConfig = field(default_factory=ButtonConfig)

    # ---------------- construction ----------------
    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RecorderConfig":
        """Build a config from a (possibly partial) dict, filling defaults.

        Unspecified scalar fields and unspecified nested-group fields fall back
        to their declared defaults; nested groups accept partial sub-dicts.
        """
        data = dict(data or {})
        nested = {
            "camera": CameraConfig,
            "can": CanConfig,
            "control": ControlConfig,
            "buttons": ButtonConfig,
        }
        kwargs: dict[str, Any] = {}
        for f in fields(cls):
            if f.name not in data:
                continue
            if f.name in nested:
                kwargs[f.name] = _build_group(nested[f.name], data[f.name])
            else:
                kwargs[f.name] = data[f.name]
        return cls(**kwargs)

    @classmethod
    def from_yaml(cls, path: str) -> "RecorderConfig":
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        return cls.from_dict(data)

    # ---------------- serialization ----------------
    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def save(self, path: str) -> None:
        """Write the fully-resolved config as YAML (saved with each session)."""
        with open(path, "w", encoding="utf-8") as fh:
            yaml.safe_dump(
                self.to_dict(), fh, allow_unicode=True, sort_keys=False
            )


def _build_group(group_cls, value: Any):
    """Reconstruct a nested dataclass group from a partial dict."""
    if value is None:
        return group_cls()
    if is_dataclass(value):
        return value
    if not isinstance(value, dict):
        raise TypeError(
            f"Expected a mapping for {group_cls.__name__}, got {type(value)!r}"
        )
    allowed = {f.name for f in fields(group_cls)}
    kwargs = {k: v for k, v in value.items() if k in allowed}
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(
            f"Unknown {group_cls.__name__} keys: {sorted(unknown)}"
        )
    return group_cls(**kwargs)
