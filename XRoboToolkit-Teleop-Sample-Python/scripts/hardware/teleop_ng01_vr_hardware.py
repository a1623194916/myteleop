"""Teleoperate both NG01 arms with PICO through the protected HCXSDK path."""
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import tyro

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.hardware.ng01_hw import (  # noqa: E402
    Ng01HwInterface,
    Ng01MockInterface,
    state_to_qpos,
    trigger_to_width_mm,
)
from scripts.simulation.teleop_ng01_dual_mujoco import build_controller  # noqa: E402
from xrobotoolkit_teleop.utils.mujoco_utils import (  # noqa: E402
    calc_mujoco_qpos_from_placo_q,
)

ARM_JOINTS = {
    "left": [f"ljoint{i}" for i in range(1, 8)],
    "right": [f"rjoint{i}" for i in range(1, 8)],
}


def limit_joint_step(target, previous, max_step_deg: float):
    target = np.asarray(target, dtype=float)
    previous = np.asarray(previous, dtype=float)
    if target.shape != (7,) or previous.shape != (7,):
        raise ValueError("joint vectors must contain 7 values")
    if not np.isfinite(target).all() or not np.isfinite(previous).all():
        raise ValueError("joint vectors must be finite")
    step = max(float(max_step_deg), 0.01)
    return previous + np.clip(target - previous, -step, step)


def _arm_targets_deg(controller) -> dict:
    desired = calc_mujoco_qpos_from_placo_q(
        controller.mj_model,
        controller.placo_robot,
        controller.placo_robot.state.q,
        floating_base=False,
    )
    return {
        side: np.degrees([
            desired[controller.mj_model.joint(name).qposadr[0]]
            for name in names
        ])
        for side, names in ARM_JOINTS.items()
    }


def _align_controller_to_state(controller, state) -> None:
    state_to_qpos(state, controller.mj_model, controller.mj_data.qpos)
    mujoco.mj_forward(controller.mj_model, controller.mj_data)
    controller._update_robot_state()
    controller.sync_end_effector_poses_to_placo_tasks()

    current = {}
    for side, names in ARM_JOINTS.items():
        values = state.left_joints_deg if side == "left" else state.right_joints_deg
        current.update({name: np.radians(value) for name, value in zip(names, values)})
    controller.joint_targets.update(current)
    controller._joints_task.set_joints(controller.joint_targets)

    for name, config in controller.manipulator_config.items():
        xyz, quat = controller._get_link_pose(config["link_name"])
        mocap = controller.target_mocap_idx[name]
        controller.mj_data.mocap_pos[mocap] = xyz
        controller.mj_data.mocap_quat[mocap] = quat


def main(
    mock: bool = False,
    input_source: str = "pico",
    confirm_motion: bool = False,
    local_ip: str = "192.168.31.55",
    remote_ip: str = "192.168.31.88",
    port: int = 8001,
    rate_hz: float = 20.0,
    speed_percent: float = 5.0,
    max_joint_step_deg: float = 0.5,
    scale_factor: float = 0.5,
    control_mode: str = "pose",
    max_gripper_width_mm: float = 60.0,
    acc_time: float = 0.2,
    dec_time: float = 0.2,
    max_line_speed: float = 0.2,
    specify_global_speed: float = 0.0,
    headless_duration: float = 0.0,
):
    """Run protected, joint-planned NG01 VR teleoperation.

    `confirm_motion` is deliberately required even though this is a dedicated
    hardware entry point. Start with `--mock --input-source fake` first.
    """
    if not confirm_motion:
        raise RuntimeError(
            "Refusing to enable arms. Re-run with --confirm-motion after "
            "checking the physical E-stop and clearing the workspace."
        )
    if rate_hz <= 0:
        raise ValueError("rate_hz must be positive")

    hw = Ng01MockInterface(rate_hz=rate_hz) if mock else Ng01HwInterface(
        local_ip=local_ip, remote_ip=remote_ip, port=port
    )
    controller = None
    enabled = False
    try:
        state = hw.read_state()
        controller = build_controller(
            input_source=input_source,
            scale_factor=scale_factor,
            control_mode=control_mode,
        )
        _align_controller_to_state(controller, state)
        last_command = {
            "left": state.left_joints_deg.copy(),
            "right": state.right_joints_deg.copy(),
        }

        hw.enable_arms(speed_percent=speed_percent)
        enabled = True
        period = 1.0 / rate_hz
        deadline = time.monotonic()
        end = deadline + headless_duration if headless_duration > 0 else None
        next_status = deadline + 1.0
        failures = 0

        while end is None or time.monotonic() < end:
            state = hw.read_state()
            state_to_qpos(state, controller.mj_model, controller.mj_data.qpos)
            mujoco.mj_forward(controller.mj_model, controller.mj_data)
            controller._update_ik()
            controller._update_gripper_target()
            targets = _arm_targets_deg(controller)

            command = {}
            for side, hand in (("left", "left_hand"), ("right", "right_hand")):
                measured = state.left_joints_deg if side == "left" else state.right_joints_deg
                if controller.active.get(hand, False):
                    command[side] = limit_joint_step(
                        targets[side], last_command[side], max_joint_step_deg
                    )
                    last_command[side] = command[side]
                else:
                    last_command[side] = measured.copy()

            if command:
                ok = hw.send_joint_targets(
                    left=command.get("left"),
                    right=command.get("right"),
                    acc_time=acc_time,
                    dec_time=dec_time,
                    max_line_speed=max_line_speed,
                    specify_global_speed=specify_global_speed,
                )
                failures = 0 if ok else failures + 1
                if failures >= 3:
                    raise RuntimeError("three consecutive arm command failures")

            for side, hand in (("left", "left_hand"), ("right", "right_hand")):
                trigger = controller.xr_client.get_key_value_by_name(
                    controller.manipulator_config[hand]["gripper_config"]["gripper_trigger"]
                )
                hw.set_gripper(side, trigger_to_width_mm(trigger, max_gripper_width_mm))

            now = time.monotonic()
            if now >= next_status:
                next_status = now + 1.0
                active = [side for side, hand in (("left", "left_hand"), ("right", "right_hand"))
                          if controller.active.get(hand, False)]
                print(f"NG01 teleop | active={active or ['none']} | command_failures={failures}")
            deadline += period
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()
    finally:
        try:
            if enabled:
                hw.stop_arms()
        finally:
            hw.close()
            if controller is not None:
                controller.xr_client.close()


if __name__ == "__main__":
    tyro.cli(main)
