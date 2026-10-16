"""A-key recording attached to the guarded Piper-X hardware control loop."""

import threading
import time

import numpy as np

from pico_recorder.episode_recorder import EpisodeRecorder, next_episode_id
from pico_recorder.orbbec_client import OrbbecImageClient
from .recorder_config import RecorderConfig
from .subtask_tracker import SubtaskTracker
from .teleop_record import RecordController, _save_session_config


class CameraCache:
    """One thread owns ZMQ; sampling reads its newest JPEG without blocking CAN."""

    def __init__(self, config):
        self.cfg = config
        self.lock = threading.Lock()
        self.frames = None
        self.last_meta = None
        self.updated = 0.0
        self.last_ns = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name="record-camera")

    def start(self):
        self.thread.start()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                self.fetch_frames()
                return
            except RuntimeError:
                time.sleep(0.05)
        raise RuntimeError("No live recording camera. Start local 305 JPEG capture first.")

    def run(self):
        client = OrbbecImageClient(self.cfg.host, self.cfg.port, self.cfg.timeout_ms)
        try:
            while not self.stop_event.is_set():
                meta, frames = client.fetch_frames()
                selected = {sn: frames[sn] for sn in self.cfg.serials
                            if frames and sn in frames and frames[sn].get("jpg")}
                if len(selected) == len(self.cfg.serials) and selected:
                    stamps = tuple(selected[sn]["ns"] for sn in self.cfg.serials)
                    with self.lock:
                        if stamps != self.last_ns:
                            self.frames, self.last_meta = selected, meta
                            self.updated, self.last_ns = time.monotonic(), stamps
                self.stop_event.wait(1 / 30)
        finally:
            client.close()

    def fetch_frames(self):
        with self.lock:
            if not self.frames or time.monotonic() - self.updated > 1:
                raise RuntimeError("Recording camera stale (> 1 s); episode stopped")
            return self.last_meta, self.frames

    def close(self):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=3)


class SessionController(RecordController):
    def extra_fn(self):
        return {"subtask": int(self.subtask.current_index)}

    def run(self):
        while not self._stop.is_set():
            try:
                self._poll_once()
                with self._lock:
                    recorder = self._recorder
                if recorder and recorder.error:
                    self._stop_episode()
                    print("[rec] Failed episode retained; restore feedback/camera and press A again.")
            except Exception as exc:
                # A failed START must require a fresh press, not repeat while held.
                self._prev_a = True
                print(f"[rec] Recording request failed: {exc}", flush=True)
            if self.stop_requested:
                break
            self._stop.wait(1 / self.cfg.control.poll_rate_hz)


class PiperRecordingSession:
    def __init__(self, config_path, xr, tcp_offset_m, arm_channels=None, motion_settings=None):
        self.motion_settings = dict(motion_settings or {})
        self.cfg = RecorderConfig.from_yaml(config_path)
        self.arm_channels = dict(arm_channels if arm_channels is not None
                                 else self.cfg.arm_channels or {"right": self.cfg.can.channel})
        self.cfg.arm_channels = self.arm_channels
        self.cfg.teleop_mode = "dual" if len(self.arm_channels) == 2 else "single"
        self.xr, self.tcp_offset_m = xr, tcp_offset_m
        self.lock = threading.Lock()
        self.state = None
        self.updated = 0.0
        self.camera = CameraCache(self.cfg.camera)
        self.controller = SessionController(
            self.cfg, xr, SubtaskTracker(self.cfg.subtasks), self.make_recorder,
            lambda: next_episode_id(self.cfg.output_dir, self.cfg.task), stop_button=None)

    def prepare(self):
        if not self.cfg.camera.serials or not 0 < self.cfg.fps <= 60:
            raise ValueError("Recording requires camera serials and 0 < fps <= 60")
        if self.cfg.control.poll_rate_hz <= 0:
            raise ValueError("Recording poll rate must be positive")
        self.camera.start()
        _save_session_config(self.cfg)

    def pause(self):
        with self.lock:
            self.state = None
            self.updated = 0.0
        self.controller._stop_episode()
        self.controller._prev_a = True

    def update(self, state):
        with self.lock:
            self.state = {key: np.asarray(value, dtype=np.float32).copy()
                          for key, value in state.items()}
            self.updated = time.monotonic()

    def snapshot(self):
        with self.lock:
            if self.state is None or time.monotonic() - self.updated > 0.3:
                raise RuntimeError("Robot recording snapshot stale (> 300 ms)")
            state = dict(self.state)
        for key, value in state.items():
            if not np.all(np.isfinite(value)):
                raise RuntimeError(f"Missing/non-finite required robot field: {key}")
        if self.cfg.teleop_mode == "dual":
            for side in ("left", "right"):
                for field, shape in (("joint_rad", (6,)), ("action_joint_rad", (6,)),
                                     ("tcp_flange", (6,)), ("tcp_eef", (6,)),
                                     ("gripper_m", ()), ("action_gripper_m", ())):
                    key = f"{side}_{field}"
                    if key not in state or state[key].shape != shape:
                        raise RuntimeError(f"Missing/invalid required dual-arm field: {key}")
        return state

    def make_recorder(self, episode_id):
        self.snapshot()
        self.camera.fetch_frames()  # Reject START when any required source is stale.
        return EpisodeRecorder(
            self.cfg.output_dir, self.cfg.task, episode_id, rate=self.cfg.fps,
            snapshot_fn=self.snapshot, extra_fn=self.controller.extra_fn,
            orbbec_client=self.camera,
            metadata={"robot": "piper_x", "teleop_mode": self.cfg.teleop_mode,
                      "arm_channels": self.arm_channels, "arm_firmware": self.cfg.arm_firmware,
                      "pose_frame": "each arm's own robot_base (not a shared world frame)",
                      "joint_names": ([f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)]
                                      if self.cfg.teleop_mode == "dual" else [f"joint{i}" for i in range(1, 7)]),
                      "pose_format": "x,y,z,roll,pitch,yaw (metres,radians; extrinsic xyz)",
                      "tcp_flange": "SDK mechanical flange pose",
                      "tcp_eef": "flange translated along local +Z",
                      "tcp_offset_m": self.tcp_offset_m,
                      "tcp_calibrated": False,
                      "motion_settings": self.motion_settings,
                      "action_joint_rad": "last target sent to CAN; dual fields use left_/right_ prefixes",
                      "state_rate_hz": self.cfg.control.rate_hz,
                      "state_sampling": "recorder holds latest hardware-loop snapshot"})

    def start(self):
        self.controller.start_thread()
        print("[rec] Ready: A starts/stops an episode; X marks a subtask. VR video is optional.")

    def close(self):
        try:
            self.controller.shutdown()
        finally:
            self.camera.close()
