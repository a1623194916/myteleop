"""Mirror the real NG01 into MuJoCo -- read-only milestone 1 of real-robot teleop.

Polls both arms, the torso lift and both grippers through the hc_robot broker
(see ng01_hw.py for the Python 3.10 broker setup) and renders the measured
state in the NG01 MuJoCo model.  Nothing is ever commanded to the robot in
this mode.

    # 1) start the vendor SDK broker once (Python 3.10, loads xoip):
    /usr/bin/python3.10 NG01_v4/hc_robot.py --hc-robot-broker \\
        192.168.31.55 192.168.31.88 8001 ""
    # 2) mirror the measured state:
    python scripts/hardware/teleop_ng01_hardware.py           # MuJoCo viewer
    python scripts/hardware/teleop_ng01_hardware.py --mock    # scripted, no robot
"""
import time
from pathlib import Path

import mujoco
import numpy as np
import tyro
from mujoco import viewer as mj_viewer

sys_path = Path(__file__).resolve().parents[2]
import sys

if str(sys_path) not in sys.path:
    sys.path.insert(0, str(sys_path))

from scripts.hardware.ng01_hw import (  # noqa: E402
    NG01_ROOT,
    Ng01HwInterface,
    Ng01MockInterface,
    state_to_qpos,
)

MJCF = NG01_ROOT / "NG01_mjcf" / "NG01_teleop.xml"

VIEWER_CAMERA = {
    "azimuth": 90,
    "elevation": -10,
    "distance": 1.8,
    "lookat": [0.0, 0.0, 1.15],
}


def dump_urdf_check_fn(hw, mock: bool):
    """Read-only URDF validation against the live controller (limits, DH, FK)."""
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)

    print("=" * 72)
    print("URDF/MJCF vs 控制器 校验（纯只读）")
    print("=" * 72)

    print("\n[1] 轴限位（度）—— 控制器实测 vs URDF")
    limits = hw.read_axis_limits()
    for side in ("left", "right"):
        print(f"  {side} arm:")
        for axis in range(7):
            jname = f"{'l' if side == 'left' else 'r'}joint{axis + 1}"
            lo, hi = model.jnt_range[model.joint(jname).id]
            lo_d, hi_d = float(np.degrees(lo)), float(np.degrees(hi))
            c_lo, c_hi = limits[side][axis]
            if c_lo is None or c_hi is None:
                flag = "  <-- 控制器读取失败"
            else:
                flag = "" if (abs(c_lo - lo_d) <= 1.0 and abs(c_hi - hi_d) <= 1.0) \
                    else "  <-- 差异 >1deg，需要核对"
            print(f"    J{axis + 1}: 控制器 [{c_lo}, {c_hi}] vs URDF [{lo_d:.1f}, {hi_d:.1f}]{flag}")

    print("\n[2] 控制器 DH 参数（与 dh.png / URDF 连杆长度对照）")
    dh = hw.read_dh_params()
    for side in ("left", "right"):
        rows = dh.get(side)
        if not rows:
            print(f"  {side}: 读取失败或 mock")
            continue
        for k, row in enumerate(rows):
            print(f"    {side} DH{k}: theta={row['theta']:.4f} d={row['d']:.4f} "
                  f"a={row['a']:.4f} alpha={row['alpha']:.4f}")

    print("\n[3] TCP 交叉验证（臂基坐标系，m；假设 get_worlds XYZ 单位 mm）")
    if mock:
        print("  (mock 模式 worlds 为占位值，误差必然很大，仅验证流程)")
    state = hw.read_state()
    state_to_qpos(state, model, data.qpos)
    mujoco.mj_forward(model, data)
    worlds = hw.read_arm_worlds()
    for side in ("left", "right"):
        prefix = side[0]
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}base_link")
        tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}tcp_link")
        base_pos = data.xpos[base_id]
        base_mat = data.xmat[base_id].reshape(3, 3)
        tcp_pos = data.xpos[tcp_id]
        tcp_in_base = base_mat.T @ (tcp_pos - base_pos)
        w = worlds.get(side)
        if not w or len(w) < 3:
            print(f"  {side}: get_worlds 读取失败")
            continue
        ctrl_pos = np.asarray(w[:3], dtype=float) / 1000.0
        err = float(np.linalg.norm(tcp_in_base - ctrl_pos))
        print(f"  {side}: 控制器 TCP {np.round(ctrl_pos, 4).tolist()} | "
              f"URDF TCP {np.round(tcp_in_base, 4).tolist()} | 误差 {err * 1000:.1f}mm"
              + ("  <-- mock 占位值" if mock else ("  <-- 超过 10mm 需要核对零位/方向" if err > 0.01 else "")))

    print("\n说明：TCP 误差包含零位偏差与控制器工作坐标系的影响；"
          "若误差恒定且方向一致，多为基座/零位标定差异，不是 URDF 连杆长度问题。")


def main(
    mock: bool = False,
    local_ip: str = "192.168.31.55",
    remote_ip: str = "192.168.31.88",
    port: int = 8001,
    rate_hz: float = 30.0,
    headless_duration: float = 0.0,
    dump_urdf_check: bool = False,
):
    """
    Mirror the measured NG01 state into MuJoCo (read-only).

    Args:
        mock: scripted state, no SDK / no robot (works anywhere).
        headless_duration: run without a viewer for this many seconds.
        dump_urdf_check: print controller-vs-URDF axis limits, DH params and
            TCP FK comparison, then exit (read-only).
    """
    if mock:
        hw = Ng01MockInterface()
    else:
        hw = Ng01HwInterface(local_ip=local_ip, remote_ip=remote_ip, port=port)

    if dump_urdf_check:
        dump_urdf_check_fn(hw, mock)
        hw.close()
        return

    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)

    period = 1.0 / rate_hz
    state = None
    last_read_error = 0.0
    reads, read_errors = 0, 0
    t_end = time.monotonic() + headless_duration if headless_duration > 0 else None
    next_status = time.monotonic() + 1.0

    def render_loop(with_viewer):
        nonlocal state, last_read_error, reads, read_errors, next_status
        while True:
            loop_t0 = time.monotonic()
            try:
                state = hw.read_state()
                state_to_qpos(state, model, data.qpos)
                mujoco.mj_forward(model, data)
                reads += 1
            except Exception as exc:
                read_errors += 1
                if time.monotonic() - last_read_error > 2.0:
                    last_read_error = time.monotonic()
                    print(f"read_state failed ({exc}); showing last good state")
            if time.monotonic() >= next_status:
                next_status = time.monotonic() + 1.0
                if state is not None:
                    print(
                        f"hw | L {np.round(state.left_joints_deg, 1).tolist()} "
                        f"| lift {state.lift_m:.3f}m "
                        f"| grip L/R {state.left_gripper_mm:.1f}/{state.right_gripper_mm:.1f}mm "
                        f"| reads {reads} err {read_errors}"
                    )
                else:
                    print("hw | waiting for first read ...")
            yield with_viewer
            remain = period - (time.monotonic() - loop_t0)
            if remain > 0:
                time.sleep(remain)
            if t_end is not None and time.monotonic() >= t_end:
                return

    if headless_duration > 0:
        for _ in render_loop(with_viewer=False):
            pass
        print(f"Read-only mirror finished: {reads} reads, {read_errors} errors")
    else:
        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.azimuth = VIEWER_CAMERA["azimuth"]
            viewer.cam.elevation = VIEWER_CAMERA["elevation"]
            viewer.cam.distance = VIEWER_CAMERA["distance"]
            viewer.cam.lookat = VIEWER_CAMERA["lookat"]
            for _ in render_loop(with_viewer=True):
                viewer.sync()
    hw.close()


if __name__ == "__main__":
    tyro.cli(main)
