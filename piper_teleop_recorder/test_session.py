"""Check episode edges, strict snapshots and HDF5 row alignment without motors."""

import tempfile
import socket
import threading
import time
import unittest
from pathlib import Path

import h5py
import numpy as np

from pico_recorder.episode_recorder import EpisodeRecorder
from pico_recorder.orbbec_client import OrbbecImageClient
from piper_teleop_recorder.session import PiperRecordingSession
from piper_teleop_recorder.teleop_record import flange_rpy_to_eef_rpy


class Buttons:
    a = False

    def get_button_state_by_name(self, name):
        return self.a if name == "A" else False


class Camera:
    last_meta = {"serials": ["camera"]}

    def fetch_frames(self):
        return self.last_meta, {"camera": {"jpg": b"jpeg", "ns": time.time_ns(), "w": 640, "h": 480}}

    def close(self):
        pass


class RecordingTest(unittest.TestCase):
    def test_b_home_button_does_not_stop_hardware_recording_controller(self):
        buttons = Buttons()
        buttons.get_button_state_by_name = lambda name: name == "B"
        session = PiperRecordingSession(str(Path(__file__).parent / "configs/piper_local_305.yaml"), buttons, 0.105)
        session.controller._poll_once()
        self.assertFalse(session.controller.stop_requested)

    def test_dual_snapshot_requires_both_arms_and_records_channel_mapping(self):
        with tempfile.TemporaryDirectory() as root:
            config = Path(__file__).parent / "configs/piper_dual_local_305.yaml"
            session = PiperRecordingSession(str(config), Buttons(), 0.105,
                                            arm_channels={"left": "can1", "right": "can0"})
            session.cfg.output_dir = root
            session.camera = Camera()
            state = {}
            for side in ("left", "right"):
                for field in ("joint_rad", "action_joint_rad", "tcp_flange", "tcp_eef"):
                    state[f"{side}_{field}"] = np.zeros(6)
                state[f"{side}_gripper_m"] = 0.03
                state[f"{side}_action_gripper_m"] = 0.04
            incomplete = dict(state)
            incomplete.pop("right_tcp_eef")
            session.update(incomplete)
            with self.assertRaisesRegex(RuntimeError, "right_tcp_eef"):
                session.make_recorder(0)
            session.update(state)
            recorder = session.make_recorder(0)
            self.assertEqual(recorder.metadata["teleop_mode"], "dual")
            self.assertEqual(recorder.metadata["arm_channels"], {"left": "can1", "right": "can0"})
            self.assertEqual(len(recorder.metadata["joint_names"]), 12)

    def test_camera_loss_stops_and_marks_episode_without_restarting_on_held_a(self):
        class FailingCamera(Camera):
            failed = False

            def fetch_frames(self):
                if self.failed:
                    raise RuntimeError("camera stale")
                return super().fetch_frames()
        with tempfile.TemporaryDirectory() as root:
            buttons = Buttons()
            session = PiperRecordingSession(str(Path(__file__).parent / "configs/piper_local_305.yaml"), buttons, 0.105)
            session.cfg.output_dir = root
            session.camera = FailingCamera()
            session.update({"joint_rad": np.zeros(6)})
            buttons.a = True
            session.start()
            try:
                deadline = time.monotonic() + 1
                while not session.controller.is_recording and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertTrue(session.controller.is_recording)
                time.sleep(0.05)
                session.camera.failed = True
                while session.controller.is_recording and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertFalse(session.controller.is_recording)
                time.sleep(0.08)  # Held A must not automatically retry.
            finally:
                session.close()
            paths = list(Path(root).glob("*/episodes/*.h5"))
            self.assertEqual(len(paths), 1)
            with h5py.File(paths[0]) as h5:
                self.assertIn("camera stale", h5.attrs["error"])
                n = h5.attrs["steps"]
                self.assertEqual(len(h5["v/joint_rad"]), n)
                self.assertEqual(len(h5["images/camera/jpeg"]), n)

    def test_zmq_client_recovers_after_server_timeout(self):
        import zmq
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        client = OrbbecImageClient("127.0.0.1", port, timeout_ms=100)
        ready = threading.Event()
        def serve():
            with zmq.Context() as context:
                with context.socket(zmq.REP) as sock:
                    sock.setsockopt(zmq.LINGER, 0)
                    sock.bind(f"tcp://127.0.0.1:{port}")
                    ready.set()
                    if sock.poll(3000):
                        sock.recv()
                        sock.send_json({"serials": [], "frames": {}})
        worker = threading.Thread(target=serve, daemon=True)
        try:
            self.assertEqual(client.fetch_frames(), (None, None))
            worker.start()
            self.assertTrue(ready.wait(3))
            deadline = time.monotonic() + 3
            meta = None
            while meta is None and time.monotonic() < deadline:
                meta, _ = client.fetch_frames()
            self.assertIsNotNone(meta)
        finally:
            client.close()
            if worker.ident is not None:
                worker.join(timeout=3)

    def test_a_edges_and_shutdown_finalize_two_distinct_episodes(self):
        with tempfile.TemporaryDirectory() as root:
            buttons = Buttons()
            session = PiperRecordingSession(str(Path(__file__).parent / "configs/piper_local_305.yaml"), buttons, 0.105)
            session.cfg.output_dir = root
            session.camera = Camera()
            flange = np.array([0, 0, 0, 0, np.pi / 2, 0])
            session.update({"joint_rad": np.zeros(6), "tcp_flange": flange,
                            "tcp_eef": flange_rpy_to_eef_rpy(flange, 0.105), "gripper_m": 0.03})
            buttons.a = True
            session.controller._poll_once()
            session.controller._poll_once()  # Holding A must not STOP.
            self.assertTrue(session.controller.is_recording)
            time.sleep(0.08)
            buttons.a = False
            session.controller._poll_once()
            buttons.a = True
            session.controller._poll_once()
            self.assertFalse(session.controller.is_recording)
            buttons.a = False
            session.controller._poll_once()
            buttons.a = True
            session.controller._poll_once()
            time.sleep(0.05)
            session.close()  # B/Ctrl+C uses the same finalization path.
            paths = sorted(Path(root).glob("*/episodes/*.h5"))
            self.assertEqual(len(paths), 2)
            for path in paths:
                with h5py.File(path) as h5:
                    n = h5.attrs["steps"]
                    self.assertGreater(n, 0)
                    for ds in h5["v"].values():
                        self.assertEqual(len(ds), n)
                    self.assertEqual(len(h5["images/camera/jpeg"]), n)
                    np.testing.assert_allclose(h5["v/tcp_eef"][0, :3], [0.105, 0, 0], atol=1e-6)

    def test_start_rejects_missing_or_stale_robot_snapshot(self):
        session = PiperRecordingSession(str(Path(__file__).parent / "configs/piper_local_305.yaml"), Buttons(), 0.105)
        with self.assertRaisesRegex(RuntimeError, "stale"):
            session.snapshot()
        session.update({"tcp_flange": np.full(6, np.nan)})
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            session.snapshot()
        session.update({"joint_rad": np.zeros(6)})
        session.updated = time.monotonic() - 1
        with self.assertRaisesRegex(RuntimeError, "stale"):
            session.snapshot()

    def test_missing_fields_and_late_camera_are_padded_to_same_sample_index(self):
        with tempfile.TemporaryDirectory() as root:
            values = iter([None, [1, 2], None])
            rec = EpisodeRecorder(root, "task", 0, getters={"state": lambda: next(values)})
            rec.ep_dir.mkdir(parents=True, exist_ok=True)
            rec._h5 = h5py.File(rec.h5_path, "x")
            try:
                rec.started_at_ns = time.time_ns()
                rec._sample()
                rec.orbbec = Camera()
                rec._sample()
                rec._sample()
                self.assertEqual(rec._h5["v/state"].shape, (3, 2))
                self.assertTrue(np.isnan(rec._h5["v/state"][0]).all())
                self.assertTrue(np.isnan(rec._h5["v/state"][2]).all())
                self.assertEqual(len(rec._h5["images/camera/jpeg"]), 3)
                self.assertEqual(len(rec._h5["images/camera/jpeg"][0]), 0)
            finally:
                rec._h5.close()


if __name__ == "__main__":
    unittest.main()
