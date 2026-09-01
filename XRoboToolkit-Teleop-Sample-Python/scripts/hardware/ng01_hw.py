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


def gripper_width_to_finger_rad(width_mm: float) -> float:
    """Gripper opening in mm -> MJCF finger joint radians.

    Direction (vendor + MJCF conventions combined): the controller's J1/J2
    are the opening in mm (0 = closed, 77.9 = max open), while the MJCF
    finger hinge is 0 = open and 1.0472 = closed.  So the map is inverted:
    width 77.9mm -> 0 rad, width 0mm -> 1.0472 rad."""
    ratio = min(max(float(width_mm), 0.0), GRIPPER_MAX_WIDTH_MM) / GRIPPER_MAX_WIDTH_MM
    return (1.0 - ratio) * GRIPPER_FINGER_MAX_RAD


def trigger_to_width_mm(trigger: float, max_width_mm: float = 60.0) -> float:
    """Teleop trigger (0 = released, 1 = squeezed) -> gripper opening in mm.

    Squeezing closes: trigger 0 -> max opening (default 60mm per operator
    spec), trigger 1 -> fully closed (0mm)."""
    trigger = min(max(float(trigger), 0.0), 1.0)
    return (1.0 - trigger) * min(max(max_width_mm, 0.0), GRIPPER_MAX_WIDTH_MM)


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

    Starts with both grippers open at 60mm; `set_gripper` updates the widths
    exactly like the real interface, so command paths can be tested
    end-to-end.  Arm joints sway sinusoidally, the lift oscillates."""

    def __init__(self, rate_hz: float = 30.0):
        self._t0 = time.monotonic()
        self.rate_hz = rate_hz
        self._joints = {"left": np.zeros(7), "right": np.zeros(7)}
        self._widths = {"left": 60.0, "right": 60.0}
        self.arms_enabled = False
        self.arm_commands = []
        self.gripper_commands = []

    def enable_arms(self, speed_percent: float = 5.0) -> None:
        self.arms_enabled = True

    def send_joint_targets(self, left=None, right=None, **motion_params) -> bool:
        if not self.arms_enabled:
            raise RuntimeError("arms are not enabled")
        command = {}
        for side, joints in (("left", left), ("right", right)):
            if joints is None:
                continue
            values = _validate_joint_target(joints, side)
            self._joints[side] = values
            command[side] = values.copy()
        if not command:
            return False
        self.arm_commands.append(command)
        return True

    def stop_arms(self) -> None:
        self.arms_enabled = False

    def set_gripper(self, arm: str, width_mm: float):
        self.gripper_commands.append((arm, float(width_mm)))
        self._widths[arm] = min(max(float(width_mm), 0.0), GRIPPER_MAX_WIDTH_MM)

    def read_state(self) -> Ng01HwState:
        t = time.monotonic() - self._t0
        return Ng01HwState(
            left_joints_deg=self._joints["left"].copy(),
            right_joints_deg=self._joints["right"].copy(),
            lift_m=0.165 + 0.165 * np.sin(0.2 * t),
            left_gripper_mm=self._widths["left"],
            right_gripper_mm=self._widths["right"],
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
        self._gripper_last = {}  # arm -> (last_send_time, last_sent_width)

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

    def enable_arms(self, speed_percent: float = 5.0) -> None:
        """Enable both arms while keeping the controller's protection active."""
        speed = min(max(float(speed_percent), 1.0), 100.0)
        enabled = []
        try:
            for side, rid in (("left", 1), ("right", 2)):
                robot = self._robots[rid]
                if not robot.get_protect_status(robot_id=rid):
                    raise RuntimeError(f"{side} arm is not in protected mode")
                if not robot.enable(robot_id=rid):
                    raise RuntimeError(f"failed to enable {side} arm")
                enabled.append(rid)
            if not self._robots[1].set_speed(speed):
                raise RuntimeError("failed to set global speed")
        except Exception:
            for rid in enabled:
                try:
                    self._robots[rid].disable(robot_id=rid)
                except Exception:
                    pass
            raise

    def send_joint_targets(
        self,
        left=None,
        right=None,
        *,
        acc_time: float = 0.2,
        dec_time: float = 0.2,
        max_line_speed: float = 0.2,
        specify_global_speed: float = 0.0,
    ) -> bool:
        """Send the active arm targets through the protected planning channel."""
        targets = {
            side: _validate_joint_target(joints, side)
            for side, joints in (("left", left), ("right", right))
            if joints is not None
        }
        if not targets:
            return False
        common = dict(
            interpolation=True,
            acc_time=float(acc_time),
            dec_time=float(dec_time),
            max_line_speed=float(max_line_speed),
            specify_global_speed=float(specify_global_speed),
        )
        if len(targets) == 2:
            params = [
                {"robot_id": rid, "joints": targets[side].tolist(), **common}
                for side, rid in (("left", 1), ("right", 2))
            ]
            return bool(self._robots[1].move_joints_multi(params))
        side, joints = next(iter(targets.items()))
        rid = 1 if side == "left" else 2
        result = self._robots[rid].move_joints(
            joints.tolist(), interpolation=True, need_block=False,
            robot_id=rid, **common
        )
        return result != 1

    def stop_arms(self) -> None:
        """Cancel queued paths and remove arm enable on shutdown."""
        for rid in (1, 2):
            try:
                self._robots[rid].clear_route(emergency_stop=False, robot_id=rid)
            except Exception as exc:
                print(f"warning: failed to clear robot {rid} route: {exc}", file=sys.stderr)
            try:
                self._robots[rid].disable(robot_id=rid)
            except Exception as exc:
                print(f"warning: failed to disable robot {rid}: {exc}", file=sys.stderr)

    def set_gripper(self, arm: str, width_mm: float, min_interval_s: float = 0.2,
                    deadband_mm: float = 1.0) -> bool:
        """Command one gripper opening in mm (vendor `move_gripper`).

        Teleop calls this at high rate from the trigger, so it rate-limits to
        one vendor call per `min_interval_s` per arm and skips no-op targets
        within `deadband_mm` (move_gripper does a read-modify-write of ID3)."""
        if arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")
        width = min(max(float(width_mm), 0.0), GRIPPER_MAX_WIDTH_MM)
        now = time.monotonic()
        last_time, last_width = self._gripper_last.get(arm, (0.0, None))
        if last_width is not None and abs(width - last_width) < deadband_mm:
            return False
        if now - last_time < min_interval_s:
            return False
        ok = bool(self._robots[3].move_gripper(width, arm=arm))
        self._gripper_last[arm] = (now, width)
        return ok

    def close(self):
        for robot in self._robots.values():
            try:
                robot.close()
            except Exception:
                pass


def _validate_joint_target(joints, side: str) -> np.ndarray:
    values = np.asarray(joints, dtype=float)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError(f"{side} joint target must contain 7 finite values")
    return values


def state_to_qpos(state: Ng01HwState, mj_model, out_qpos: np.ndarray) -> np.ndarray:
    """Map an Ng01HwState into NG01_teleop.xml qpos (by joint name)."""
    qpos = out_qpos
    qpos[mj_model.joint("up_down_joint").qposadr[0]] = state.lift_m
    for j in range(7):
        qpos[mj_model.joint(f"ljoint{j + 1}").qposadr[0]] = np.radians(state.left_joints_deg[j])
        qpos[mj_model.joint(f"rjoint{j + 1}").qposadr[0]] = np.radians(state.right_joints_deg[j])
    qpos[mj_model.joint("lgripper_finger_joint").qposadr[0]] = gripper_width_to_finger_rad(state.left_gripper_mm)
    qpos[mj_model.joint("rgripper_finger_joint").qposadr[0]] = gripper_width_to_finger_rad(state.right_gripper_mm)
    return qpos
