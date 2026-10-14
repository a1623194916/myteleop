# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""PiperX teleop DATA RECORDER (runs on robot machine 41).

Reuses the picosoftware teleop stack to let an operator teleoperate a single
PiperX arm with a VR controller while recording 30Hz episodes (Orbbec images
640x480 + arm state) with per-frame subtask labels into HDF5 (via the existing
``pico_recorder.EpisodeRecorder``), convertible to LeRobot.

Three concerns in one process:
  1. Control thread (~20Hz): XrClient -> MuJoCo IK / JointFollower ->
     arm.move_joints + arm.set_gripper(1 - trigger), gated by right-GRIP deadman.
  2. Button-poll thread (~20Hz): right-A rising edge toggles recorder start/stop;
     left-X rising edge advances the subtask index.
  3. EpisodeRecorder (30Hz daemon): snapshots arm getters + extra_fn (subtask,
     trigger, grip) + Orbbec frames into HDF5.

All picosoftware / hardware imports are LAZY (inside functions) so that
``--dry-run`` runs the full plumbing (recorder tick, A-edge, subtask counter,
HDF5 write) with FAKE getters / no XR / no arm / no camera.

Launch (hardware session, picosoftware venv)::

    cd /home/u22/kyz/pico_software
    XRoboToolkit-Teleop-Sample-Python/.venv/bin/python \
        piper_teleop_recorder/teleop_record.py \
        --config piper_teleop_recorder/configs/piper_reach.yaml --enable-hardware
"""

import argparse
import os
import sys
import threading
import time

import numpy as np

# Import the sibling pure modules the same way regardless of invocation: the
# script's own directory is placed on sys.path first (on machine 41 this is
# .../pico_software/piper_teleop_recorder).
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from recorder_config import RecorderConfig  # noqa: E402
from subtask_tracker import SubtaskTracker  # noqa: E402

# Flange -> gripper-eef tool offset along the flange local +Z (metres). The
# canonical value is ``xrobotoolkit_teleop...piper_x_model.TCP_OFFSET_M``; it is
# imported from there in the hardware path, and this literal is the dry-run
# fallback (the dry run must not import the MuJoCo model stack).
DEFAULT_TCP_OFFSET_M = 0.105


def flange_rpy_to_eef_rpy(flange6, tcp_offset_m):
    """Shift a flange pose ``[x,y,z, roll,pitch,yaw]`` to the gripper eef.

    The eef shares the flange orientation (the TCP joint is a pure translation
    along the flange local +Z), so only the position moves. Returns the eef as
    the same 6-D ``[x,y,z, roll,pitch,yaw]`` layout the recorder stores for the
    flange, so both ``tcp_flange`` and ``tcp_eef`` columns are directly
    comparable. Matches ``remote_piper.eef_math.flange_to_eef``.
    """
    from scipy.spatial.transform import Rotation  # lazy; euler->rotation

    f = np.asarray(flange6, dtype=np.float32).reshape(6)
    rot = Rotation.from_euler("xyz", f[3:6])  # XYZ-extrinsic, driver convention
    eef_pos = f[:3] + rot.apply([0.0, 0.0, float(tcp_offset_m)])
    return np.concatenate([eef_pos, f[3:6]]).astype(np.float32)


# --------------------------------------------------------------------------- #
# Recorder lifecycle driven by controller buttons (hardware-agnostic).
# --------------------------------------------------------------------------- #
class RecordController:
    """Toggle episode recording on right-A edges; advance subtask on left-X.

    Works with any ``button_source`` exposing ``get_button_state_by_name`` and
    ``get_key_value_by_name`` -- the real ``XrClient`` or a dry-run fake -- so
    the exact edge-detection / subtask plumbing is exercised in both modes.
    """

    def __init__(self, config, button_source, subtask_tracker,
                 recorder_factory, next_id_fn):
        self.cfg = config
        self.buttons = button_source
        self.subtask = subtask_tracker
        self._recorder_factory = recorder_factory
        self._next_id_fn = next_id_fn
        self._recorder = None
        self._recording = False
        self._prev_a = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.stop_requested = False  # set when right-B is pressed

    # ---- extra per-frame values written into v/subtask, v/right_trigger ... ----
    def extra_fn(self):
        return {
            "subtask": int(self.subtask.current_index),
            "right_trigger": float(
                self.buttons.get_key_value_by_name(self.cfg.buttons.gripper_key)
            ),
            "right_grip": float(
                self.buttons.get_key_value_by_name(self.cfg.buttons.deadman_key)
            ),
        }

    # ---- episode lifecycle ----
    def _start_episode(self):
        episode_id = int(self._next_id_fn())
        self.subtask.reset()
        rec = self._recorder_factory(episode_id)
        rec.start()
        with self._lock:
            self._recorder = rec
            self._recording = True
        print(f"[teleop] START episode {episode_id} (subtask -> 0)")

    def _stop_episode(self):
        with self._lock:
            rec = self._recorder
            self._recorder = None
            self._recording = False
        if rec is not None:
            steps, err = rec.stop()
            print(f"[teleop] STOP episode: {steps} frames (error={err})")

    @property
    def is_recording(self):
        with self._lock:
            return self._recording

    # ---- poll loop ----
    def _poll_once(self):
        a = bool(self.buttons.get_button_state_by_name(self.cfg.buttons.record_toggle))
        if a and not self._prev_a:  # rising edge
            if self.is_recording:
                self._stop_episode()
            else:
                self._start_episode()
        self._prev_a = a
        x = bool(self.buttons.get_button_state_by_name(self.cfg.buttons.subtask_mark))
        if self.subtask.update(x):
            print(f"[teleop] subtask -> {self.subtask.current_index} "
                  f"({self.subtask.current_name})")
        # Optional session stop on right-B.
        try:
            if self.buttons.get_button_state_by_name("B"):
                self.stop_requested = True
        except Exception:
            pass

    def run(self):
        period = 1.0 / float(self.cfg.control.poll_rate_hz)
        while not self._stop.is_set():
            tick = time.monotonic()
            self._poll_once()
            if self.stop_requested:
                break
            time.sleep(max(0.0, period - (time.monotonic() - tick)))

    def start_thread(self):
        self._thread = threading.Thread(target=self.run, daemon=True,
                                        name="record-poll")
        self._thread.start()

    def shutdown(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.is_recording:
            self._stop_episode()


# --------------------------------------------------------------------------- #
# Dry-run fakes (no XR / no arm / no camera). Prove the plumbing end to end.
# --------------------------------------------------------------------------- #
class _FakeButtons:
    """Scripted controller timeline: A start/stop pulses, two X advances."""

    def __init__(self, t0):
        self._t0 = t0

    def _t(self):
        return time.monotonic() - self._t0

    def get_button_state_by_name(self, name):
        t = self._t()
        if name == "A":
            return (0.30 <= t < 0.45) or (1.60 <= t < 1.75)
        if name == "X":
            return (0.70 <= t < 0.85) or (1.10 <= t < 1.25)
        return False  # "B" and others

    def get_key_value_by_name(self, name):
        import math
        t = self._t()
        if name == "right_trigger":
            return float(0.5 + 0.4 * math.sin(3.0 * t))
        if name == "right_grip":
            return 1.0  # deadman engaged throughout the fake session
        return 0.0


class _FakeArm:
    """Slowly-varying 6-dim joint feedback (radians)."""

    def __init__(self):
        self._t0 = time.monotonic()

    def read_joints(self):
        import math
        t = time.monotonic() - self._t0
        return np.array(
            [0.05 * math.sin(t + i) for i in range(6)], dtype=np.float32
        )


class _FakeOrbbec:
    """Returns the same ``(meta, frames)`` tuple shape as OrbbecImageClient."""

    def __init__(self, serials, w, h):
        import cv2
        img = np.zeros((int(h), int(w), 3), dtype=np.uint8)
        ok, buf = cv2.imencode(".jpg", img)
        if not ok:
            raise RuntimeError("cv2.imencode failed for the fake frame")
        self._jpg = buf.tobytes()
        self._serials = list(serials)
        self._w = int(w)
        self._h = int(h)
        self.last_meta = {"serials": self._serials}

    def fetch_frames(self):
        ns = time.time_ns()
        frames = {
            sn: {"jpg": self._jpg, "ns": ns, "w": self._w, "h": self._h}
            for sn in self._serials
        }
        meta = {
            "serials": self._serials,
            "frames": {
                sn: {"ns": ns, "w": self._w, "h": self._h}
                for sn in self._serials
            },
        }
        self.last_meta = meta
        return meta, frames

    def close(self):
        pass


def _ensure_picosoftware_on_path(cfg):
    """Make ``pico_recorder`` importable (it is a dir under picosoftware_root,
    not an installed package). ``xrobotoolkit_teleop`` is editable-installed."""
    root = cfg.picosoftware_root
    if root and os.path.isdir(root) and root not in sys.path:
        sys.path.insert(0, root)


def _save_session_config(cfg):
    """Write the fully-resolved config next to the session (provenance)."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(cfg.output_dir, cfg.task)
    os.makedirs(session_dir, exist_ok=True)
    path = os.path.join(session_dir, f"recorder_config_{ts}.yaml")
    cfg.save(path)
    print(f"[teleop] saved resolved config -> {path}")
    return path


# --------------------------------------------------------------------------- #
# Dry run
# --------------------------------------------------------------------------- #
def run_dry_run(cfg, duration_s=2.2):
    """Exercise the full recorder plumbing with fakes; verify the HDF5."""
    import tempfile

    # Never write into the real dataset dir during a dry run.
    cfg.output_dir = tempfile.mkdtemp(prefix="piper_dryrun_")
    print(f"[dry-run] output_dir -> {cfg.output_dir}")
    _ensure_picosoftware_on_path(cfg)
    _save_session_config(cfg)

    from pico_recorder.episode_recorder import (  # lazy: only needs h5py/numpy
        EpisodeRecorder,
        next_episode_id,
    )

    fake_arm = _FakeArm()
    fake_orbbec = _FakeOrbbec(cfg.camera.serials, cfg.camera.width, cfg.camera.height)
    buttons = _FakeButtons(time.monotonic())
    tracker = SubtaskTracker(cfg.subtasks)

    getters = {
        "joint_rad": fake_arm.read_joints,
        # Store BOTH TCP frames (flange + gripper eef); see the hardware path.
        "tcp_flange": lambda: np.zeros(6, dtype=np.float32),
        "tcp_eef": lambda: flange_rpy_to_eef_rpy(
            np.zeros(6, dtype=np.float32), DEFAULT_TCP_OFFSET_M
        ),
        "gripper_m": lambda: 0.03,
    }
    if cfg.teleop_mode == "dual":
        getters = {f"{side}_{key}": getter for side in ("left", "right")
                   for key, getter in getters.items()}

    ctrl_holder = {}

    def factory(episode_id):
        rec = EpisodeRecorder(
            cfg.output_dir, cfg.task, episode_id, rate=cfg.fps,
            getters=getters, extra_fn=ctrl_holder["ctrl"].extra_fn,
            orbbec_client=fake_orbbec,
            metadata={"robot": "piper_x", "teleop_mode": cfg.teleop_mode,
                      "arm_channels": cfg.arm_channels,
                      "tcp_offset_m": DEFAULT_TCP_OFFSET_M, "tcp_calibrated": False,
                      "joint_names": [f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)]
                      if cfg.teleop_mode == "dual" else [f"joint{i}" for i in range(1, 7)]},
        )
        return rec

    ctrl = RecordController(
        cfg, buttons, tracker,
        recorder_factory=factory,
        next_id_fn=lambda: next_episode_id(cfg.output_dir, cfg.task),
    )
    ctrl_holder["ctrl"] = ctrl
    ctrl.start_thread()

    deadline = time.monotonic() + float(duration_s)
    while time.monotonic() < deadline and not ctrl.stop_requested:
        time.sleep(0.02)
    ctrl.shutdown()

    _verify_dry_run_output(cfg)


def _verify_dry_run_output(cfg):
    import glob

    import h5py

    ep_dir = os.path.join(cfg.output_dir, cfg.task, "episodes")
    h5s = sorted(glob.glob(os.path.join(ep_dir, "ep_*.h5")))
    if not h5s:
        raise SystemExit(f"[dry-run][FAIL] no HDF5 written under {ep_dir}")
    path = h5s[0]
    with h5py.File(path, "r") as f:
        v_keys = sorted(f["v"].keys()) if "v" in f else []
        img_serials = sorted(f["images"].keys()) if "images" in f else []
        n = f["v"]["subtask"].shape[0] if ("v" in f and "subtask" in f["v"]) else 0
        subtask_vals = (
            np.asarray(f["v"]["subtask"][:]).astype(int).tolist()
            if ("v" in f and "subtask" in f["v"]) else []
        )
        img_frames = {
            sn: int(f["images"][sn]["jpeg"].shape[0]) for sn in img_serials
        }
    print("\n==================== DRY-RUN HDF5 ====================")
    print(f"file          : {path}")
    print(f"v/ columns    : {v_keys}")
    print(f"frames (N)    : {n}")
    print(f"images serials: {img_serials}  frames={img_frames}")
    if subtask_vals:
        print(f"subtask range : min={min(subtask_vals)} max={max(subtask_vals)} "
              f"(distinct={sorted(set(subtask_vals))})")
    print("======================================================\n")

    assert "subtask" in v_keys, "v/subtask column missing"
    assert "right_trigger" in v_keys, "v/right_trigger column missing"
    assert "right_grip" in v_keys, "v/right_grip column missing"
    for prefix in (("left_", "right_") if cfg.teleop_mode == "dual" else ("",)):
        for field in ("joint_rad", "tcp_flange", "tcp_eef"):
            assert prefix + field in v_keys, f"v/{prefix}{field} column missing"
    assert "timestamp" in v_keys, "v/timestamp column missing"
    assert img_serials, "no image columns written"
    assert n > 0, "no frames recorded"
    print("[dry-run][PASS] HDF5 contains v/subtask + arm state (tcp_flange + "
          "tcp_eef) + image columns, no hardware used.")


# --------------------------------------------------------------------------- #
# Hardware session
# --------------------------------------------------------------------------- #
def run_hardware(cfg, enable_hardware=False):
    """Use the same guarded controller as start_piper_x.sh; camera runs separately."""
    if not enable_hardware:
        print("Preflight only: motors NOT enabled. Pass --enable-hardware after:")
        print("  * CAN up at 1 Mbit/s, e-stop reachable, workspace clear")
        print(f"  * recording camera ready at {cfg.camera.host}:{cfg.camera.port}")
        print("  * XR/VR headset service running and controller data updating")
        return

    _ensure_picosoftware_on_path(cfg)
    sample_root = os.path.join(cfg.picosoftware_root, "XRoboToolkit-Teleop-Sample-Python")
    if sample_root not in sys.path:
        sys.path.insert(0, sample_root)
    from scripts.hardware.teleop_piper_x_hardware import main as teleop_main

    config_path = _save_session_config(cfg)
    teleop_main(
        channel=cfg.can.channel if cfg.teleop_mode == "single" else None,
        enable_hardware=True, firmware=cfg.can.firmware,
        rate_hz=cfg.control.rate_hz, max_joint_step_deg=cfg.control.max_joint_step_deg,
        scale_factor=cfg.control.scale_factor, speed_percent=cfg.can.speed_percent,
        gripper_max_width=cfg.can.gripper_max_width, record_config=config_path,
        gripper_force=cfg.can.gripper_force, no_pose_filter=cfg.control.no_pose_filter,
        position_deadband=cfg.control.position_deadband,
        rotation_deadband_deg=cfg.control.rotation_deadband_deg,
        max_linear_speed=cfg.control.max_linear_speed,
        max_angular_speed=cfg.control.max_angular_speed,
        mode=cfg.teleop_mode,
        controller_side=cfg.buttons.deadman_key.split("_")[0],
        left_channel=cfg.arm_channels.get("left", "can0"),
        right_channel=cfg.arm_channels.get("right", "can1"),
        left_firmware=cfg.arm_firmware.get("left"),
        right_firmware=cfg.arm_firmware.get("right"),
    )

# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", required=True, help="unified recorder YAML config")
    ap.add_argument("--dry-run", action="store_true",
                    help="run the full plumbing with fakes (no HW); verify HDF5")
    ap.add_argument("--enable-hardware", action="store_true",
                    help="enable PiperX motors (required for a real session)")
    ap.add_argument("--duration", type=float, default=2.2,
                    help="dry-run duration in seconds")
    args = ap.parse_args()

    cfg = RecorderConfig.from_yaml(args.config)
    if args.dry_run:
        run_dry_run(cfg, duration_s=args.duration)
    else:
        run_hardware(cfg, enable_hardware=args.enable_hardware)


if __name__ == "__main__":
    main()
