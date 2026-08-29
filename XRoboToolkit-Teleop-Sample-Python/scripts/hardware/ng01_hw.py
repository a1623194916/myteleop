"""NG01 hardware interface: thin wrapper over the vendor HCXSDK (hc_robot.py).

The vendor SDK (xoip) is a CPython 3.10 extension while the teleop stack
(mujoco/placo/curobo) runs on 3.11+.  hc_robot.py already solves this: on
Linux HCRobot defaults to *remote mode*, RPC-ing every call over a Unix
socket to a broker process.  Start the broker ONCE with Python 3.10 (it
loads xoip and owns the controller connection):

    /usr/bin/python3.10 NG01_v4/hc_robot.py --hc-robot-broker \\
        192.168.31.55 192.168.31.88 8001 ""

Any process on this machine (any Python) can then create HCRobot instances.
Do NOT let the 3.11 teleop process auto-spawn the broker: hc_robot uses
sys.executable for that, and a 3.11-spawned broker cannot load xoip -- this
module therefore refuses to construct instances when no broker is alive.

Vendor conventions (hc_robot.py):
    robot_id 1  left arm   get_joints -> 7 angles in degrees
    robot_id 2  right arm  get_joints -> 7 angles in degrees
    robot_id 3  grippers + lift
                J1 = left gripper opening in mm (0..77.9)
                J2 = right gripper opening in mm
                J5 = lift in mm, get_lift_position_m() -> URDF metres
"""
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

PICO_ROOT = Path(__file__).resolve().parents[3]
NG01_ROOT = PICO_ROOT / "NG01_v4"

GRIPPER_MAX_WIDTH_MM = 77.9
GRIPPER_FINGER_MAX_RAD = 1.0472


def gripper_mm_to_rad(width_mm: float) -> float:
    """Gripper opening in mm -> MJCF finger joint radians (0..1.0472)."""
    ratio = min(max(float(width_mm), 0.0), GRIPPER_MAX_WIDTH_MM) / GRIPPER_MAX_WIDTH_MM
    return ratio * GRIPPER_FINGER_MAX_RAD


@dataclass
class Ng01HwState:
    """One snapshot of the real robot (mirrors NG01_teleop MJCF conventions)."""
    left_joints_deg: np.ndarray      # (7,) degrees
    right_joints_deg: np.ndarray     # (7,) degrees
    lift_m: float                    # URDF metres, 0 (top) .. 0.33 (bottom)
    left_gripper_mm: float
    right_gripper_mm: float
    stamp: float = field(default_factory=time.monotonic)


class Ng01MockInterface:
    """Scripted stand-in for Ng01HwInterface: no SDK, no network.

    Used for headless development and unit tests.  Sinusoidal arm sway, a
    slow lift oscillation and alternating gripper openings."""

    def __init__(self, rate_hz: float = 30.0):
        self._t0 = time.monotonic()
        self.rate_hz = rate_hz

    def read_state(self) -> Ng01HwState:
        t = time.monotonic() - self._t0
        sway = [10.0 * np.sin(0.7 * t), 5.0 * np.sin(0.5 * t + 1.0), 0.0, 0.0, 0.0, 0.0, 0.0]
        return Ng01HwState(
            left_joints_deg=np.array(sway),
            right_joints_deg=np.array(sway),
            lift_m=0.165 + 0.165 * np.sin(0.2 * t),
            left_gripper_mm=GRIPPER_MAX_WIDTH_MM if (t % 4.0) < 2.0 else 0.0,
            right_gripper_mm=0.0 if (t % 4.0) < 2.0 else GRIPPER_MAX_WIDTH_MM,
        )

    def read_axis_limits(self) -> dict:
        """Scripted limits deliberately offset by ~0.5deg from the URDF so the
        comparison table demonstrates its discrepancy flag."""
        out = {}
        for side in ("left", "right"):
            rows = []
            for axis in range(7):
                rows.append((-169.5 - axis, 169.5 - axis))
            out[side] = rows
        return out

    def read_arm_worlds(self) -> dict:
        # placeholder: mock has no kinematics; the FK comparison will show a
        # large error in mock mode, which is expected
        return {"left": [300.0, 0.0, 600.0, 0.0, 0.0, 180.0],
                "right": [300.0, 0.0, 600.0, 0.0, 0.0, 180.0]}

    def read_dh_params(self) -> dict:
        return {"left": None, "right": None}

    def close(self):
        pass


class Ng01HwInterface:
    """Read/write access to the real NG01 through the hc_robot broker."""

    def __init__(
        self,
        local_ip: str = "192.168.31.55",
        remote_ip: str = "192.168.31.88",
        port: int = 8001,
    ):
        if str(NG01_ROOT) not in sys.path:
            sys.path.insert(0, str(NG01_ROOT))
        import hc_robot  # vendor wrapper, pure python on this path

        self._hc_robot = hc_robot
        socket_path = hc_robot._broker_paths(remote_ip, port)[0]
        info = hc_robot._broker_info(socket_path)
        if not info or not info.get("alive"):
            raise RuntimeError(
                "hc_robot broker 未运行。先用 Python 3.10 启动（它会加载 xoip）：\n"
                f"  /usr/bin/python3.10 {NG01_ROOT / 'hc_robot.py'} "
                f"--hc-robot-broker {local_ip} {remote_ip} {port} \"\"\n"
                "不要让 Python 3.11 的遥操进程自动拉起 broker（sys.executable "
                "会选错解释器，加载不了 xoip）。"
            )

        self._robots = {
            rid: hc_robot.HCRobot(local_ip=local_ip, remote_ip=remote_ip,
                                  port=port, robot_id=rid)
            for rid in (1, 2, 3)
        }

    def read_state(self) -> Ng01HwState:
        left = self._robots[1].get_joints(feedback=True, robot_id=1)
        right = self._robots[2].get_joints(feedback=True, robot_id=2)
        id3 = self._robots[3].get_joints(feedback=True, robot_id=3)
        if left is None or right is None or id3 is None or len(id3) < 5:
            raise RuntimeError("SDK 读取失败（left/right/id3 返回 None）")
        lift_m = self._robots[3].get_lift_position_m()
        return Ng01HwState(
            left_joints_deg=np.asarray(left[:7], dtype=float),
            right_joints_deg=np.asarray(right[:7], dtype=float),
            lift_m=float(lift_m),
            left_gripper_mm=float(id3[0]),
            right_gripper_mm=float(id3[1]),
        )

    def read_axis_limits(self) -> dict:
        """Real controller axis limits: {side: [(neg_deg, pos_deg) x 7]}."""
        out = {}
        for side, rid in (("left", 1), ("right", 2)):
            rows = []
            for axis in range(7):
                neg = self._robots[rid].get_axis_limit(axis, positive=False, robot_id=rid)
                pos = self._robots[rid].get_axis_limit(axis, positive=True, robot_id=rid)
                rows.append((None if neg is None else float(neg),
                             None if pos is None else float(pos)))
            out[side] = rows
        return out

    def read_arm_worlds(self) -> dict:
        """Controller-side FK TCP per arm: {side: [X,Y,Z (mm), U,V,W (deg)]}."""
        return {
            side: self._robots[rid].get_worlds(feedback=True, robot_id=rid)
            for side, rid in (("left", 1), ("right", 2))
        }

    def read_dh_params(self) -> dict:
        """Controller DH parameters: {side: [{"theta","d","a","alpha"}]}."""
        return {
            side: self._robots[rid].get_dh_params(robot_id=rid)
            for side, rid in (("left", 1), ("right", 2))
        }

    def close(self):
        for robot in self._robots.values():
            try:
                robot.close()
            except Exception:
                pass


def state_to_qpos(state: Ng01HwState, mj_model, out_qpos: np.ndarray) -> np.ndarray:
    """Map an Ng01HwState into NG01_teleop.xml qpos (by joint name)."""
    qpos = out_qpos
    qpos[mj_model.joint("up_down_joint").qposadr[0]] = state.lift_m
    for j in range(7):
        qpos[mj_model.joint(f"ljoint{j + 1}").qposadr[0]] = np.radians(state.left_joints_deg[j])
        qpos[mj_model.joint(f"rjoint{j + 1}").qposadr[0]] = np.radians(state.right_joints_deg[j])
    qpos[mj_model.joint("lgripper_finger_joint").qposadr[0]] = gripper_mm_to_rad(state.left_gripper_mm)
    qpos[mj_model.joint("rgripper_finger_joint").qposadr[0]] = gripper_mm_to_rad(state.right_gripper_mm)
    return qpos
