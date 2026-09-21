"""Teleoperate a single FR3C arm (real hardware) with a PICO controller.

Streams Placo IK joint targets to the robot via the Fairino SDK ServoJ
interface — the same architecture as the official dual UR5e hardware sample.

SAFETY: the arm will follow the selected controller while its GRIP is held.
Keep the e-stop within reach and clear the workspace before starting.

Input: XRoboToolkit SDK (requires the PC service running and PICO connected).

Runtime settings are loaded from ``configs/fr3c_teleop.yaml`` by default.
"""
from pathlib import Path
import time

import tyro

PICO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = str(PICO_ROOT / "fr3c_assets/fr3c_teleop.urdf")
DEFAULT_MUJOCO_XML = str(PICO_ROOT / "fr3c_assets/scene_fr3c.xml")
DEFAULT_CONFIG = str(Path(__file__).resolve().parents[2] / "configs/fr3c_teleop.yaml")
# Sim home pose (tool pointing straight down) — matches scene_fr3c.xml.
DEFAULT_INITIAL_JOINT_DEG = [0.0, -90.0, 51.5708, -51.5708, 270.0, 0.0]


def main(
    robot_ip: str | None = None,
    robot_urdf_path: str = DEFAULT_URDF,
    mujoco_xml_path: str = DEFAULT_MUJOCO_XML,
    initial_joints_deg: list[float] = DEFAULT_INITIAL_JOINT_DEG,
    scale_factor: float = 1.0,
    cmd_t: float = 0.01,
    servo_transport: str = "udp",
    smooth_tau_ms: float = 60.0,
    max_joint_step_deg: float = 0.30,
    controller_side: str | None = None,
    input_min_cutoff_hz: float = 1.5,
    input_beta: float = 0.08,
    position_deadband_mm: float = 1.5,
    rotation_deadband_deg: float = 0.5,
    gripper_index: int = 1,
    gripper_velocity: int = 100,
    gripper_force: int = 20,
    gripper_open_force: int = 100,
    gripper_force_limit_n: float = 3.0,
    gripper_trigger_threshold: float = 0.05,
    gripper_min_change: float = 1.0,
    gripper_min_interval_s: float = 0.15,
    gripper_max_time_ms: int = 5000,
    gripper_closed_percent: int = 90,
    gripper_open_percent: int = 0,
    gripper_toggle_button: str | None = "right_axis_click",
    gripper_toggle_debounce_ms: float = 180.0,
    activate_gripper: bool = True,
    end_effector_hz: float = 50.0,
    reset: bool = False,
    visualize_mujoco: bool = False,
    visualize_placo: bool = False,
    debug_csv_path: str | None = None,
    config_path: str = DEFAULT_CONFIG,
):
    """
    Run FR3C teleoperation on real hardware.

    Args:
        robot_ip: FR3C controller IP address.
        initial_joints_deg: Joint pose (deg) used by --reset, ideally with the
            tool pointing down like the simulation home pose.
        reset: Move the arm to --initial-joints-degree before starting.
        scale_factor: Controller motion gain (1.0 = 1:1).
        cmd_t: ServoJ command period in seconds (0.008 - 0.016 recommended).
            Raise toward 0.014-0.016 if the timing diag shows a fat tail.
        smooth_tau_ms: Command trajectory time constant (ms). The command
            approaches the IK target with a time-based exponential, so servo
            tick jitter changes the phase, never the velocity. Higher =
            smoother but laggier.
        max_joint_step_deg: Hard joint-step cap per servo tick (deg). Bounds
            joint speed (default 0.30 deg/tick at cmd_t=0.01).
        controller_side: "auto", "left", or "right". Auto maps robot IP
            192.168.5.22 to left and 192.168.5.23 to right.
        input_min_cutoff_hz: One Euro filter cutoff at rest for XR deltas.
            Lower = steadier while holding still (more lag while moving).
        input_beta: One Euro speed-adaptive term for XR deltas. Higher =
            snappier fast motions but more tremor pass-through.
        position_deadband_mm: Accumulated controller translation required
            before updating the IK target, in millimeters.
        rotation_deadband_deg: Accumulated controller rotation required
            before updating the IK target, in degrees.
        gripper_toggle_button: Optional button whose rising edge also toggles
            the gripper; the selected controller trigger is an edge toggle.
        config_path: YAML file containing all runtime settings.
        visualize_mujoco: Show a MuJoCo mirror of measured hardware joints.
            Renders in a SEPARATE process (never in the servo process — GIL
            contention there perturbs ServoJ send timing).
        visualize_placo: Open the MeshCat Placo visualization in a browser.
    """
    import gc

    from xrobotoolkit_teleop.hardware.fr3c_config import load_config

    config = load_config(config_path, "single")
    # An explicitly supplied CLI value identifies the arm for this run;
    # otherwise use the single-arm defaults from YAML.
    robot_ip = robot_ip if robot_ip is not None else config.get("robot_ip", "192.168.58.2")
    controller_side = (
        controller_side
        if controller_side is not None
        else config.get("controller_side", "auto")
    )
    robot_urdf_path = config.get("robot_urdf_path", robot_urdf_path)
    mujoco_xml_path = config.get("mujoco_xml_path", mujoco_xml_path)
    initial_joints_deg = config.get("initial_joints_deg", initial_joints_deg)
    if not Path(robot_urdf_path).is_absolute():
        robot_urdf_path = str(PICO_ROOT / robot_urdf_path)
    scale_factor = config.get("scale_factor", scale_factor)
    cmd_t = config.get("cmd_t", cmd_t)
    servo_transport = config.get("servo_transport", servo_transport)
    smooth_tau_ms = config.get("smooth_tau_ms", smooth_tau_ms)
    max_joint_step_deg = config.get("max_joint_step_deg", max_joint_step_deg)
    input_min_cutoff_hz = config.get("input_min_cutoff_hz", input_min_cutoff_hz)
    input_beta = config.get("input_beta", input_beta)
    position_deadband_mm = config.get("position_deadband_mm", position_deadband_mm)
    rotation_deadband_deg = config.get("rotation_deadband_deg", rotation_deadband_deg)
    gripper_index = config.get("gripper_index", gripper_index)
    gripper_velocity = config.get("gripper_velocity", gripper_velocity)
    gripper_force = config.get("gripper_force_percent", gripper_force)
    gripper_open_force = config.get("gripper_open_force_percent", gripper_open_force)
    gripper_trigger_threshold = config.get("gripper_trigger_threshold", gripper_trigger_threshold)
    gripper_min_change = config.get("gripper_min_change", gripper_min_change)
    gripper_min_interval_s = config.get("gripper_min_interval_s", gripper_min_interval_s)
    gripper_max_time_ms = config.get("gripper_max_time_ms", gripper_max_time_ms)
    gripper_closed_percent = config.get("gripper_closed_percent", gripper_closed_percent)
    gripper_open_percent = config.get("gripper_open_percent", gripper_open_percent)
    gripper_toggle_button = config.get("gripper_toggle_button", gripper_toggle_button)
    if str(gripper_toggle_button).strip().lower() == "auto":
        gripper_toggle_button = f"{controller_side}_axis_click"
    gripper_toggle_debounce_ms = config.get("gripper_toggle_debounce_ms", gripper_toggle_debounce_ms)
    activate_gripper = config.get("activate_gripper", activate_gripper)
    end_effector_hz = config.get("end_effector_hz", end_effector_hz)
    reset = config.get("reset", reset)
    visualize_mujoco = config.get("visualize_mujoco", visualize_mujoco)
    visualize_placo = config.get("visualize_placo", visualize_placo)
    debug_csv_path = config.get("debug_csv_path", debug_csv_path) or debug_csv_path

    from xrobotoolkit_teleop.common.xr_client import XrClient
    from xrobotoolkit_teleop.hardware.fr3c_teleop_controller import (
        Fr3cTeleopController,
    )

    xr_client = XrClient()
    controller = Fr3cTeleopController(
        xr_client=xr_client,
        robot_urdf_path=robot_urdf_path,
        robot_ip=robot_ip,
        initial_joint_deg=initial_joints_deg,
        scale_factor=scale_factor,
        cmd_t=cmd_t,
        servo_transport=servo_transport,
        visualize_placo=visualize_placo,
        smooth_tau_s=smooth_tau_ms / 1000.0,
        max_joint_step_deg=max_joint_step_deg,
        controller_side=controller_side,
        input_min_cutoff_hz=input_min_cutoff_hz,
        input_beta=input_beta,
        position_deadband_mm=position_deadband_mm,
        rotation_deadband_deg=rotation_deadband_deg,
        debug_csv_path=debug_csv_path,
    )
    print(
        f"Single arm routing: {controller.controller_side}_controller -> "
        f"{controller.controller_side} arm; ServoJ={servo_transport}"
    )
    # Move the startup object graph out of the generational GC scans: a full
    # collection mid-stream stalls the GIL (and the ServoJ send).
    gc.freeze()

    import threading
    from xrobotoolkit_teleop.hardware.fr3c_gripper import Fr3cVrGripperController

    stop_signal = threading.Event()

    gripper = Fr3cVrGripperController(
        xr_client=xr_client,
        robot=controller.robot,
        controller_side=controller.controller_side,
        trigger_threshold=gripper_trigger_threshold,
        open_position_percent=float(gripper_open_percent),
        closed_position_percent=float(gripper_closed_percent),
        min_position_change_percent=gripper_min_change,
        min_command_interval_s=gripper_min_interval_s,
        gripper_index=gripper_index,
        velocity=gripper_velocity,
        force=gripper_force,
        open_force=gripper_open_force,
        max_time_ms=gripper_max_time_ms,
        toggle_button=gripper_toggle_button,
        toggle_debounce_s=gripper_toggle_debounce_ms / 1000.0,
        debug_logger=controller.debug_logger,
    )

    if reset:
        print("Reset flag detected. Moving the arm to the initial joint pose...")
        try:
            controller.reset()
        except Exception as e:
            print(f"Reset failed: {e}")
            controller.close()
            return
    else:
        print("No reset flag. Teleoperation starts from the CURRENT arm pose.")

    if activate_gripper:
        print("Clearing stale gripper warnings before teleoperation...")
        try:
            controller.robot.clear_gripper_warning(index=gripper_index)
        except Exception as e:
            print(f"Gripper warning clear failed; aborting startup: {e}")
            controller.close()
            xr_client.close()
            return

    arm_thread = threading.Thread(target=controller.run_arm_thread, args=(stop_signal,))
    ik_thread = threading.Thread(target=controller.run_ik_thread, args=(stop_signal,))
    arm_thread.start()
    ik_thread.start()

    gripper_thread = None
    if activate_gripper:
        print("Waiting for the first ServoJ command before gripper activation...")
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and not controller.robot.servo_ready:
            if stop_signal.wait(0.05):
                break
        try:
            if not controller.robot.servo_ready:
                raise RuntimeError("the first ServoJ command was not sent")
            if not controller.robot.ensure_fault_free():
                raise RuntimeError("robot fault remained after startup clear")
            # The preflight explicitly reset the gripper, so activate it even
            # if the TG-9801 CNDE active bit is stale.
            gripper.activate()
            if not controller.robot.ensure_fault_free():
                raise RuntimeError("gripper activation fault could not be cleared")
            gripper.initialize_idle()
            print("Gripper warning cleared and control ready (startup move skipped).")
            gripper_thread = threading.Thread(
                target=gripper.run, args=(stop_signal, end_effector_hz),
                name="right-gripper", daemon=True,
            )
            gripper_thread.start()
            print("Gripper control started.")
        except Exception as e:
            print(f"Gripper activation failed; arm teleop continues: {e}")

    mirror = None
    try:
        if visualize_mujoco:
            from xrobotoolkit_teleop.hardware.fr3c_mujoco_mirror import MirrorProcess

            mirror = MirrorProcess(mujoco_xml_path)
            mirror.start()
            while not stop_signal.is_set() and mirror.is_running():
                # CNDE state is refreshed by the SDK thread and does not take
                # the ServoJ RPC lock. A 30 Hz GetActualJointPosDegree RPC
                # here would periodically delay the realtime sender and show
                # up as a low-frequency wobble.
                measured = controller.robot.get_realtime_joint_positions()
                if measured is None:
                    with controller._state_lock:
                        measured = controller.command_q.copy()
                mirror.publish(measured)
                stop_signal.wait(1.0 / mirror.refresh_hz)
        else:
            while not stop_signal.is_set():
                stop_signal.wait(0.05)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Exiting...")
    except Exception as e:
        print(f"Hardware visualization/control loop failed: {e}")
    finally:
        stop_signal.set()
        if gripper_thread is not None:
            gripper_thread.join(timeout=1.0)
        if mirror is not None:
            mirror.stop()

    while arm_thread.is_alive() or ik_thread.is_alive():
        try:
            arm_thread.join(timeout=0.1)
            ik_thread.join(timeout=0.1)
        except KeyboardInterrupt:
            print("KeyboardInterrupt detected while stopping...")
            stop_signal.set()
    controller.close()
    try:
        xr_client.close()
    except Exception:
        pass
    print("FR3C teleoperation stopped.")


if __name__ == "__main__":
    tyro.cli(main)
