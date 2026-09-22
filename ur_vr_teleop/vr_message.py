"""VR数据消息类型定义。

用于VR Bridge和Master进程之间的ZeroMQ通信。
使用JSON序列化确保Python 3.8/3.10兼容性。
"""
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional
import json


@dataclass
class VRControllerData:
    """单个VR控制器的完整数据。"""
    timestamp_ns: int
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])  # [qx, qy, qz, qw]
    trigger: float = 0.0
    grip: float = 0.0
    axis: List[float] = field(default_factory=lambda: [0.0, 0.0])

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "VRControllerData":
        return cls(
            timestamp_ns=data.get("timestamp_ns", 0),
            position=data.get("position", [0.0, 0.0, 0.0]),
            orientation=data.get("orientation", [0.0, 0.0, 0.0, 1.0]),
            trigger=data.get("trigger", 0.0),
            grip=data.get("grip", 0.0),
            axis=data.get("axis", [0.0, 0.0]),
        )


@dataclass
class VRDataMessage:
    """完整的VR数据消息，包含头显和两个控制器的数据。"""
    timestamp_ns: int
    headset_pose: List[float] = field(default_factory=lambda: [0.0] * 7)  # [x,y,z,qx,qy,qz,qw]
    left_controller: VRControllerData = field(default_factory=VRControllerData)
    right_controller: VRControllerData = field(default_factory=VRControllerData)
    buttons: Dict[str, bool] = field(default_factory=dict)  # A, B, X, Y等按键状态

    def to_dict(self) -> dict:
        return {
            "timestamp_ns": self.timestamp_ns,
            "headset_pose": self.headset_pose,
            "left_controller": self.left_controller.to_dict(),
            "right_controller": self.right_controller.to_dict(),
            "buttons": self.buttons,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    @classmethod
    def from_dict(cls, data: dict) -> "VRDataMessage":
        return cls(
            timestamp_ns=data.get("timestamp_ns", 0),
            headset_pose=data.get("headset_pose", [0.0] * 7),
            left_controller=VRControllerData.from_dict(data.get("left_controller", {})),
            right_controller=VRControllerData.from_dict(data.get("right_controller", {})),
            buttons=data.get("buttons", {}),
        )

    @classmethod
    def from_json(cls, json_str: str) -> "VRDataMessage":
        return cls.from_dict(json.loads(json_str))


# 按键名称常量
BUTTON_NAMES = ["A", "B", "X", "Y", "left_menu_button", "right_menu_button", "left_axis_click", "right_axis_click"]
