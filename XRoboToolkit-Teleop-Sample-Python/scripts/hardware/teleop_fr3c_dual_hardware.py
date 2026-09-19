"""Teleoperate two FR3C arms (real hardware) simultaneously with PICO controllers.

Left controller  -> left arm  (192.168.5.22, parallel gripper)
Right controller -> right arm (192.168.5.23, parallel gripper)

Each arm runs its own Fr3cTeleopController (Placo IK + ServoJ stream), so both
arms can be taken over and driven independently at the same time. Hold a
controller's GRIP to take over that arm; release to freeze it in place.

Hold the left controller's Y button to glide the LEFT arm, and the right
controller's B button to glide the RIGHT arm, back to a FIXED home pose
(DEFAULT_HOME_*_DEG, captured from the arms' actual pose on 2026-09-18;
override with --home-q-left-deg / --home-q-right-deg), at a bounded speed;
release to hold the current pose. Homing is ignored while that arm's grip is
held, so it can never trigger mid-teleoperation.

End-effector control runs in a separate thread per arm and does NOT require
GRIP takeover:
  - left trigger  -> left gripper closure (analog: 0 = open, 1 = closed)
  - right trigger -> right gripper closure (analog)

MoveGripper is a non-blocking call: the RPC returns immediately and the
controller executes the move in the background. Commanded targets are only
sent when the mapped position changes by >= --gripper-min-change %, so the
trigger is effectively smooth and non-blocking (see Fr3cVrGripperController).

Data collection: pressing the right-hand controller's A button sends a light
START/STOP command to the Jetson record server (192.168.5.27:8766 by default),
which records images + both-arm robot state into raw HDF5 on the Jetson itself
(no image transfer over the network during teleoperation). Convert afterwards
with convert_to_lerobot.py.

SAFETY: keep the e-stop within reach and clear the workspace before starting.

Input: XRoboToolkit SDK (requires the PC service running and PICO connected).
"""
from pathlib import Path

import numpy as np
import tyro

PICO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = str(PICO_ROOT / "fr3c_assets/fr3c_teleop.urdf")
DEFAULT_INITIAL_JOINT_DEG = [0.0, -90.0, 51.5708, -51.5708, 270.0, 0.0]

LEFT_ROBOT_IP = "192.168.5.22"
RIGHT_ROBOT_IP = "192.168.5.23"

# Hold-Y/B home pose, captured from the arms' ACTUAL pose on 2026-09-18
# (left arm and right arm respectively, joint angles in degrees j1..j6).
# Override per arm with --home-q-left-deg / --home-q-right-deg.
DEFAULT_HOME_LEFT_DEG = [-63.422, -77.906, -39.449, -134.366, 92.991, -47.235]
DEFAULT_HOME_RIGHT_DEG = [66.248, -91.981, 44.815, -51.790, 271.275, -32.616]


def main(
    left_robot_ip: str = LEFT_ROBOT_IP,
    right_robot_ip: str = RIGHT_ROBOT_IP,
    robot_urdf_path: str = DEFAULT_URDF,
    initial_joints_deg: list[float] = DEFAULT_INITIAL_JOINT_DEG,
    scale_factor: float = 1.0,
    cmd_t: float = 0.01,
    smooth_tau_ms: float = 40.0,
    max_joint_step_deg: float = 1.0,
    input_min_cutoff_hz: float = 2.0,
    input_beta: float = 0.02,
    position_deadband_mm: float = 1.5,
    rotation_deadband_deg: float = 0.5,
    home_button_left: str = "Y",
    home_button_right: str = "B",
    home_joint_speed_dps: float = 60.0,
    home_q_left_deg: list[float] = DEFAULT_HOME_LEFT_DEG,
    home_q_right_deg: list[float] = DEFAULT_HOME_RIGHT_DEG,
    reset: bool = False,
    # Parallel grippers (BOTH arms)
    gripper_index: int = 1,
    gripper_velocity: int = 20,
    gripper_force: int = 20,
    gripper_trigger_threshold: float = 0.05,
    gripper_min_change: float = 1.0,
    gripper_min_interval_s: float = 0.15,
    gripper_motion_gate: bool = False,
    gripper_max_time_ms: int = 5000,
    gripper_closed_percent: int = 97,
    activate_gripper: bool = True,
    # End-effector polling loop
    end_effector_hz: float = 20.0,
    # Remote recording (A button -> controls the Jetson record server)
    record_server_host: str = "192.168.5.27",
    record_server_port: int = 8766,
    record_task: str = "fr3c_dual",
):
    """
    Run dual-arm FR3C teleoperation on real hardware.

    Args:
        left_robot_ip / right_robot_ip: FR3C controller IPs (both with grippers).
        robot_urdf_path: Single-arm URDF used by each arm's Placo solver.
        initial_joints_deg: Joint pose (deg) used by --reset for BOTH arms.
        reset: Move both arms to --initial-joints-deg before starting.
        scale_factor: Controller motion gain (1.0 = 1:1).
        cmd_t: ServoJ command period in seconds (0.008 - 0.016).
        smooth_tau_ms: Command trajectory time constant (ms).
        max_joint_step_deg: Hard cap per servo tick (deg).
        input_min_cutoff_hz / input_beta: One Euro filter tuning for XR deltas.
        position_deadband_mm / rotation_deadband_deg: controller deadbands.
        home_button_left / home_button_right: XR buttons held to glide the
            left/right arm back to its fixed home pose (left controller's Y
            and right controller's B). Empty string disables that arm's homing.
        home_q_left_deg / home_q_right_deg: fixed home joint pose (deg) per
            arm, captured from the arms on 2026-09-18.
        home_joint_speed_dps: Bounded joint speed (deg/s) while homing.
        gripper_index / gripper_velocity / gripper_force / gripper_max_time_ms:
            Fairino gripper command parameters (shared by both arms). The
            trigger is a true ANALOG slider: trigger travel maps linearly to
            closure and short max_time segments make the gripper follow the
            finger continuously.
        gripper_trigger_threshold / gripper_min_change / gripper_closed_percent:
            Trigger->closure mapping (0=open, closed_percent = max close).
        activate_gripper: ActGripper reset+activate after the servo streams
            are up (the servo start latches fault 8-1 once; activating after
            it is cleared is what makes the activation stick).
        end_effector_hz: Polling rate of the gripper control threads.
        record_server_host / record_server_port: ZMQ endpoint of the Jetson
            record server. When host is empty, A-button recording is disabled.
            Otherwise the right-hand A button sends START/STOP commands there;
            images + robot states are recorded on the Jetson (no image stream).
        record_task: Dataset task name (=the subfolder recorded on the Jetson).
    """
    import gc
    import os
    import sys
    import threading
    import time

    if str(PICO_ROOT) not in sys.path:
        sys.path.insert(0, str(PICO_ROOT))

    from pico_recorder.record_client import RecordClient
    from xrobotoolkit_teleop.common.xr_client import XrClient
    from xrobotoolkit_teleop.hardware.fr3c_gripper import Fr3cVrGripperController
    from xrobotoolkit_teleop.hardware.fr3c_teleop_controller import (
        Fr3cTeleopController,
    )

    xr_client = XrClient()

    arm_kwargs = dict(
        xr_client=xr_client,
        robot_urdf_path=robot_urdf_path,
        initial_joint_deg=initial_joints_deg,
        scale_factor=scale_factor,
        cmd_t=cmd_t,
        smooth_tau_s=smooth_tau_ms / 1000.0,
        max_joint_step_deg=max_joint_step_deg,
        input_min_cutoff_hz=input_min_cutoff_hz,
        input_beta=input_beta,
        position_deadband_mm=position_deadband_mm,
        rotation_deadband_deg=rotation_deadband_deg,
        home_joint_speed_dps=home_joint_speed_dps,
    )
    left_controller = Fr3cTeleopController(
        robot_ip=left_robot_ip,
        controller_side="left",
        home_button=home_button_left,
        home_q_deg=home_q_left_deg,
        **arm_kwargs,
    )
    right_controller = Fr3cTeleopController(
        robot_ip=right_robot_ip,
        controller_side="right",
        home_button=home_button_right,
        home_q_deg=home_q_right_deg,
        **arm_kwargs,
    )

    # Move the startup object graph out of the generational GC scans.
    gc.freeze()

    stop_signal = threading.Event()

    if reset:
        print("Reset flag detected. Moving both arms to the initial joint pose...")
        try:
            left_controller.reset()
            right_controller.reset()
        except Exception as e:
            print(f"Reset failed: {e}")
            left_controller.close()
            right_controller.close()
            xr_client.close()
            return
    else:
        print("No reset flag. Teleoperation starts from the CURRENT arm poses.")

    # End effectors (independent of GRIP takeover) -- two grippers.
    gripper_left = Fr3cVrGripperController(
        xr_client=xr_client,
        robot=left_controller.robot,
        controller_side="left",
        trigger_threshold=gripper_trigger_threshold,
        open_position_percent=0.0,
        closed_position_percent=float(gripper_closed_percent),
        min_position_change_percent=gripper_min_change,
        min_command_interval_s=gripper_min_interval_s,
        motion_done_gate=gripper_motion_gate,
        gripper_index=gripper_index,
        velocity=gripper_velocity,
        force=gripper_force,
        max_time_ms=gripper_max_time_ms,
    )
    gripper_right = Fr3cVrGripperController(
        xr_client=xr_client,
        robot=right_controller.robot,
        controller_side="right",
        trigger_threshold=gripper_trigger_threshold,
        open_position_percent=0.0,
        closed_position_percent=float(gripper_closed_percent),
        min_position_change_percent=gripper_min_change,
        min_command_interval_s=gripper_min_interval_s,
        motion_done_gate=gripper_motion_gate,
        gripper_index=gripper_index,
        velocity=gripper_velocity,
        force=gripper_force,
        max_time_ms=gripper_max_time_ms,
    )
    if activate_gripper:
        print("Gripper activation deferred until after the servo streams are up.")

# ---- Remote collection (optional) ----
    # 遥操机只把 A 键转成 START/STOP 指令;  图像/状态都在 Jetson 本机落盘。
    record_client = RecordClient(
        host=record_server_host, port=record_server_port
    ) if record_server_host else None
    if record_client is not None:
        print(f"[record] 采集控制已连接 {record_server_host}:{record_server_port}")
        print("[record] 右手柄 A = 开始/结束采集 (任务名: " + record_task + ")")

    # 遥操转发状态: 遥操端独占每台控制器的 CNDE 状态流, 把关节/夹爪真值推给采集端,
    # 避免采集端再连同一控制器(CNDE 单客户端)读到全 0。转发失败自动降级, 采集端回退本地直读。
    def _read_forward(ctrl):
        """读一台控制器的关节(deg)/夹爪, 优先 CNDE 真值, 兜底 RPC。"""
        try:
            pkg = ctrl.robot.robot_state_pkg
            if pkg is not None and not isinstance(pkg, type) and hasattr(pkg, "jt_cur_pos"):
                deg = [float(v) for v in pkg.jt_cur_pos[:6]]
                if len(deg) == 6 and all(v == v for v in deg):
                    grip = getattr(pkg, "gripper_position", None)
                    return deg, (float(grip) if grip is not None else None)
        except Exception:
            pass
        try:
            rad = ctrl.robot.get_current_joint_positions()
            return [float(x) for x in np.rad2deg(rad)], None
        except Exception:
            return None, None

    def forward_state_loop():
        left_ctrl, right_ctrl = left_controller, right_controller
        rc = record_client
        while not stop_signal.is_set() and rc is not None:
            try:
                l_deg, l_grip = _read_forward(left_ctrl)
                r_deg, r_grip = _read_forward(right_ctrl)
                if l_deg is not None and r_deg is not None:
                    rc.push_state(
                        l_deg, r_deg,
                        l_grip if l_grip is not None else -1.0,
                        r_grip if r_grip is not None else -1.0,
                    )
            except Exception:
                pass
            stop_signal.wait(0.02)

    if record_client is not None:
        threading.Thread(target=forward_state_loop, daemon=True, name="fwd-state").start()

    threads = []
    for ctrl, label in ((left_controller, "left"), (right_controller, "right")):
        threads.append(
            threading.Thread(
                target=ctrl.run_arm_thread, args=(stop_signal,), name=f"{label}-arm-controller"
            )
        )
        threads.append(
            threading.Thread(
                target=ctrl.run_ik_thread, args=(stop_signal,), name=f"{label}-ik"
            )
        )

    for t in threads:
        t.start()

    if activate_gripper:
        # Start the gripper session AFTER the servo streams are up: ServoMoveStart
        # latches drive fault 8-1 once, the stream's recovery clears it (which
        # also DEACTIVATES the grippers as a ResetAllError side effect), so
        # activating afterwards is the only order in which the activation sticks.
        print("Waiting for servo streams to go fault-free before gripper activation...")
        deadline = time.monotonic() + 8.0
        ready = False
        while time.monotonic() < deadline:
            ready = True
            for ctrl in (left_controller, right_controller):
                try:
                    main_code, _sub = ctrl.robot.get_robot_error_code()
                except Exception as e:
                    print(f"GetRobotErrorCode at startup failed: {e}")
                    ready = False
                    break
                if main_code != 0:
                    ready = False
                    break
            if ready:
                break
            time.sleep(0.3)
        if not ready:
            print("Aborting: arms not fault-free after servo start.")
            stop_signal.set()
            for t in threads:
                t.join(timeout=2.0)
            for ctrl in (left_controller, right_controller):
                ctrl.close()
            xr_client.close()
            return

        t_start = time.monotonic()

        def _activate(grip, label):
            try:
                grip.activate()
            except Exception as e:
                print(f"{label} gripper activation failed: {e}")

        act_threads = []
        for grip, ctrl, label in (
            (gripper_left, left_controller, "left"),
            (gripper_right, right_controller, "right"),
        ):
            if ctrl.robot.gripper_active(gripper_index):
                print(f"{label} gripper already active; skipping activation.")
                continue
            act_threads.append(
                threading.Thread(target=_activate, args=(grip, label),
                                 name=f"{label}-gripper-activate")
            )
        for t in act_threads:
            t.start()  # arms are independent controllers: activate in parallel
        for t in act_threads:
            t.join()
        print(f"Gripper startup done in {time.monotonic() - t_start:.1f}s.")
        time.sleep(0.5)  # settle: surface any fault the activation strokes caused
        for ctrl in (left_controller, right_controller):
            # If a stroke tripped 8-1, clear it; the deactivated gripper is
            # brought back by its own control thread (single ownership).
            ctrl.robot.ensure_fault_free()

    for grip, lab_ in (
        (gripper_left, "left-gripper"),
        (gripper_right, "right-gripper"),
    ):
        threads.append(
            threading.Thread(target=grip.run, args=(stop_signal, end_effector_hz), name=lab_)
        )
        threads[-1].start()

    # A 键(右手柄)按下 -> 通知 Jetson 采集服务开始/结束; 无记录时自然循环.
    prev_a = False
    rec_running = False
    try:
        while not stop_signal.is_set():
            a = bool(xr_client.get_button_state_by_name("A")) if record_client else False
            if a and not prev_a and record_client is not None:
                rec_running = not rec_running
                if rec_running:
                    resp = record_client.start(record_task)
                    print(f"A 按下 -> 开始采集 (resp={resp})")
                else:
                    resp = record_client.stop()
                    print(f"A 按下 -> 结束采集 (resp={resp})")
            prev_a = a
            stop_signal.wait(0.05)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Exiting...")
    finally:
        stop_signal.set()
        for t in threads:
            t.join(timeout=1.0)
        # Stop the arm servo sessions first so they cannot interleave with the
        # release/shutdown RPCs below.
        for ctrl in (left_controller, right_controller):
            try:
                ctrl.robot.stop_servo()
            except Exception as e:
                print(f"Servo stop during shutdown failed: {e}")
        if record_client is not None:
            try:
                record_client.close()
            except Exception as e:
                print(f"Record client close failed: {e}")

    for ctrl in (left_controller, right_controller):
        try:
            ctrl.close()
        except Exception as e:
            print(f"Controller close failed: {e}")
    print("Dual FR3C teleoperation stopped.")
    # Shut the XR service down explicitly: leaving its C++ service thread
    # running into interpreter teardown aborts the process with "terminate
    # called without an active exception". os._exit below is the belt to that
    # suspenders: skip teardown entirely once every handle is closed.
    try:
        xr_client.close()
    except Exception as e:
        print(f"XR client close failed: {e}")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    tyro.cli(main)
