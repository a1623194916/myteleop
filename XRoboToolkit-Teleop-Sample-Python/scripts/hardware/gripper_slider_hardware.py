"""Screen-slider gripper control: drag sliders, the grippers follow. No VR.

Two horizontal sliders (left arm / right arm) map 0..100% to gripper closure
(0 = fully open, top = --gripper-closed-percent). Uses the same production
follower as VR teleop (Fr3cVrGripperController), including the reactive
8-1 fault clearing this rig needs after every gripper motion.

The arms stay IDLE in this mode: no servo session is started, so nothing
moves but the grippers. Keep people clear of the grippers anyway.

Usage:
  PYTHONPATH=. .venv/bin/python scripts/hardware/gripper_slider_hardware.py
  # single arm:
  PYTHONPATH=. .venv/bin/python scripts/hardware/gripper_slider_hardware.py --no-left
  # headless self-test (auto sweep, no window):
  PYTHONPATH=. .venv/bin/python scripts/hardware/gripper_slider_hardware.py --auto-sweep --no-gui
"""
from pathlib import Path

import tyro

PICO_ROOT = Path(__file__).resolve().parents[3]

LEFT_ROBOT_IP = "192.168.5.22"
RIGHT_ROBOT_IP = "192.168.5.23"


def main(
    left_robot_ip: str = LEFT_ROBOT_IP,
    right_robot_ip: str = RIGHT_ROBOT_IP,
    no_left: bool = False,
    no_right: bool = False,
    gripper_index: int = 1,
    gripper_velocity: int = 30,
    gripper_force: int = 20,
    gripper_closed_percent: int = 97,
    gripper_min_change: float = 1.0,
    gripper_min_interval_s: float = 0.15,
    update_hz: float = 10.0,
    no_gui: bool = False,
    auto_sweep: bool = False,
    smoke: bool = False,
):
    """Args:
        left_robot_ip / right_robot_ip: FR3C controller IPs.
        no_left / no_right: control only one arm.
        gripper_index / velocity / force: Fairino gripper command parameters.
        gripper_closed_percent: slider top = this closure (97 leaves 3% margin).
        gripper_min_change / gripper_min_interval_s: follower rate limiting.
        update_hz: control loop rate.
        no_gui: no window (for --auto-sweep self-test).
        auto_sweep: ramp both sliders 0->100->0 automatically (self-test).
        smoke: open the GUI, programmatically drag both sliders to 50% and
            back, then exit — verifies the widget-callback chain live.
    """
    import sys
    import threading
    import time

    if str(PICO_ROOT) not in sys.path:
        sys.path.insert(0, str(PICO_ROOT))

    from xrobotoolkit_teleop.hardware.fr3c_gripper import Fr3cVrGripperController
    from xrobotoolkit_teleop.hardware.interface.fr3c import Fr3cController

    class SliderSource:
        """Drop-in XR replacement: the GUI slider supplies the trigger value."""

        def __init__(self):
            self.value = 0.0

        def get_key_value_by_name(self, name):
            return self.value

    arms = {}
    sliders = {}
    stop = threading.Event()

    specs = []
    if not no_left:
        specs.append(("left", left_robot_ip))
    if not no_right:
        specs.append(("right", right_robot_ip))
    if not specs:
        raise SystemExit("both --no-left and --no-right given; nothing to control")

    for label, ip in specs:
        print(f"Connecting to FR3C at {ip} ({label}) ...")
        arm = Fr3cController(robot_ip=ip, cmd_t=0.01)
        source = SliderSource()
        gripper = Fr3cVrGripperController(
            xr_client=source,
            robot=arm,
            controller_side=label,
            trigger_threshold=0.0,  # slider maps the FULL travel, no dead zone
            open_position_percent=0.0,
            closed_position_percent=float(gripper_closed_percent),
            min_position_change_percent=gripper_min_change,
            min_command_interval_s=gripper_min_interval_s,
            motion_done_gate=False,
            trigger_toggle=False,
            gripper_index=gripper_index,
            velocity=gripper_velocity,
            force=gripper_force,
        )
        arms[label] = arm
        sliders[label] = (source, gripper)
    print(f"Connected: {', '.join(arms)}. Arms stay idle; only grippers move.")

    def control_loop():
        period = 1.0 / update_hz
        while not stop.is_set():
            for source, gripper in sliders.values():
                try:
                    gripper.update()
                except Exception as e:
                    print(f"Gripper update failed (kept alive): {e}")
            stop.wait(period)

    worker = threading.Thread(target=control_loop, name="gripper-slider", daemon=True)
    worker.start()

    if auto_sweep and no_gui:
        print("Auto sweep: 0 -> 100 -> 0 ...")
        ramp = [x * 0.04 for x in range(26)] + [1.0 - x * 0.04 for x in range(1, 26)]
        for v in ramp:
            for source, _gripper in sliders.values():
                source.value = v
            time.sleep(0.1)
        print("Auto sweep done. Closing.")
        stop.set()
        worker.join(timeout=2.0)
        for arm in arms.values():
            arm.close()
        return

    # ---------- GUI ----------
    import tkinter as tk

    root = tk.Tk()
    root.title("FR3C 夹爪滑块")
    status = {}

    def make_row(arm_key, display):
        frame = tk.Frame(root, padx=10, pady=6)
        frame.pack(fill=tk.X)
        title = tk.Label(frame, text=f"{display} 夹爪  (0=全开, 100=闭合)", width=28, anchor="w")
        title.pack(side=tk.TOP, anchor="w")
        value_var = tk.StringVar(value="0%")

        def on_move(v):
            source, _ = sliders[arm_key]
            source.value = float(v) / 100.0
            value_var.set(f"{int(float(v))}%  目标 {float(v) / 100.0 * gripper_closed_percent:.0f}%")

        slider = tk.Scale(
            frame, from_=0, to=100, orient=tk.HORIZONTAL, length=420,
            showvalue=False, resolution=1, command=on_move,
        )
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        value_label = tk.Label(frame, textvariable=value_var, width=14, anchor="e")
        value_label.pack(side=tk.LEFT)
        status[arm_key] = value_var

        return slider

    sliders_ui = {}
    for arm_key in arms:
        sliders_ui[arm_key] = make_row(arm_key, "左臂" if arm_key == "left" else "右臂")

    if smoke:
        def _smoke():
            def step(v):
                for s in sliders_ui.values():
                    s.set(v)
            for v in (25, 50, 75, 50, 0):
                root.after(300, step, v)
            root.after(2200, on_close)
        root.after(500, _smoke)

    def on_close():
        stop.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    print("Slider GUI ready. Close the window to exit.")
    try:
        root.mainloop()
    finally:
        stop.set()
        worker.join(timeout=2.0)
        for arm in arms.values():
            try:
                arm.close()
            except Exception as e:
                print(f"close failed: {e}")
        print("Gripper slider closed.")


if __name__ == "__main__":
    tyro.cli(main)
