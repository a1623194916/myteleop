"""Teleoperate a single FR3C arm (real hardware) with a PICO controller.

Streams Placo IK joint targets to the robot via the Fairino SDK ServoJ
interface — the same architecture as the official dual UR5e hardware sample.

SAFETY: the arm will follow your controller as soon as you pull and hold the
right GRIP.  Keep the e-stop within reach and clear the workspace before
starting.

Input: XRoboToolkit SDK (requires the PC service running and PICO connected).
"""
from pathlib import Path

import tyro

PICO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = str(PICO_ROOT / "fr3c_assets/fr3c_teleop.urdf")
# Sim home pose (tool pointing straight down) — matches scene_fr3c.xml.
DEFAULT_INITIAL_JOINT_DEG = [0.0, -90.0, 51.5708, -51.5708, 270.0, 0.0]


def main(
    robot_ip: str = "192.168.58.2",
    robot_urdf_path: str = DEFAULT_URDF,
    initial_joints_deg: list[float] = DEFAULT_INITIAL_JOINT_DEG,
    scale_factor: float = 1.0,
    cmd_t: float = 0.01,
    smooth_alpha: float = 0.35,
    max_joint_step_deg: float = 1.0,
    reset: bool = False,
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
        smooth_alpha: Target smoothing factor per servo tick (0..1). Lower =
            smoother but laggier; raise if the arm feels too sluggish.
        max_joint_step_deg: Hard joint-step cap per servo tick (deg). Bounds
            joint speed (default 1 deg/tick at cmd_t=0.01 -> 100 deg/s).
        visualize_placo: Open the MeshCat Placo visualization in a browser.
    """
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
        smooth_alpha=smooth_alpha,
        max_joint_step_deg=max_joint_step_deg,
    )

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

    while not stop_signal.is_set():
        try:
            import time

            time.sleep(0.05)
        except KeyboardInterrupt:
            print("KeyboardInterrupt detected. Exiting...")
            stop_signal.set()

    arm_thread.join()
    ik_thread.join()
    controller.close()
    print("FR3C teleoperation stopped.")


if __name__ == "__main__":
    tyro.cli(main)
