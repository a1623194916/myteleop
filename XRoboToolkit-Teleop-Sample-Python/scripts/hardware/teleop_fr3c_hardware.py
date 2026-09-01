"""Teleoperate a single FR3C arm (real hardware) with a PICO controller.

Streams Placo IK joint targets to the robot via the Fairino SDK ServoJ
interface — the same architecture as the official dual UR5e hardware sample.

SAFETY: the arm will follow the selected controller while its GRIP is held.
Keep the e-stop within reach and clear the workspace before starting.

Input: XRoboToolkit SDK (requires the PC service running and PICO connected).
"""
from pathlib import Path

import tyro

PICO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = str(PICO_ROOT / "fr3c_assets/fr3c_teleop.urdf")
DEFAULT_MUJOCO_XML = str(PICO_ROOT / "fr3c_assets/scene_fr3c.xml")
# Sim home pose (tool pointing straight down) — matches scene_fr3c.xml.
DEFAULT_INITIAL_JOINT_DEG = [0.0, -90.0, 51.5708, -51.5708, 270.0, 0.0]


def main(
    robot_ip: str = "192.168.58.2",
    robot_urdf_path: str = DEFAULT_URDF,
    mujoco_xml_path: str = DEFAULT_MUJOCO_XML,
    initial_joints_deg: list[float] = DEFAULT_INITIAL_JOINT_DEG,
    scale_factor: float = 1.0,
    cmd_t: float = 0.01,
    smooth_tau_ms: float = 40.0,
    max_joint_step_deg: float = 1.0,
    controller_side: str = "auto",
    input_min_cutoff_hz: float = 2.0,
    input_beta: float = 0.02,
    position_deadband_mm: float = 1.5,
    rotation_deadband_deg: float = 0.5,
    reset: bool = False,
    visualize_mujoco: bool = False,
    visualize_placo: bool = False,
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
            joint speed (default 1 deg/tick at cmd_t=0.01 -> 100 deg/s).
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
        visualize_mujoco: Show a MuJoCo mirror of measured hardware joints.
            Renders in a SEPARATE process (never in the servo process — GIL
            contention there perturbs ServoJ send timing).
        visualize_placo: Open the MeshCat Placo visualization in a browser.
    """
    import gc

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
        visualize_placo=visualize_placo,
        smooth_tau_s=smooth_tau_ms / 1000.0,
        max_joint_step_deg=max_joint_step_deg,
        controller_side=controller_side,
        input_min_cutoff_hz=input_min_cutoff_hz,
        input_beta=input_beta,
        position_deadband_mm=position_deadband_mm,
        rotation_deadband_deg=rotation_deadband_deg,
    )
    # Move the startup object graph out of the generational GC scans: a full
    # collection mid-stream stalls the GIL (and the ServoJ send).
    gc.freeze()

    import threading

    stop_signal = threading.Event()

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

    arm_thread = threading.Thread(target=controller.run_arm_thread, args=(stop_signal,))
    ik_thread = threading.Thread(target=controller.run_ik_thread, args=(stop_signal,))
    arm_thread.start()
    ik_thread.start()

    mirror = None
    try:
        if visualize_mujoco:
            from xrobotoolkit_teleop.hardware.fr3c_mujoco_mirror import MirrorProcess

            mirror = MirrorProcess(mujoco_xml_path)
            mirror.start()
            while not stop_signal.is_set() and mirror.is_running():
                mirror.publish(controller.robot.get_current_joint_positions())
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
    print("FR3C teleoperation stopped.")


if __name__ == "__main__":
    tyro.cli(main)
