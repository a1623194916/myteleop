"""Real 305 JPEG -> A-key episodes -> aligned HDF5, with fake arm/VR only."""

import argparse
import subprocess
import tempfile
import time
from pathlib import Path

import cv2
import h5py
import numpy as np

from .recorder_config import RecorderConfig
from .session import PiperRecordingSession
from .test_session import Buttons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("single", "dual"), default="dual")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    with tempfile.TemporaryDirectory(prefix="piper_local_record_") as temp:
        filename = "piper_dual_local_305.yaml" if args.mode == "dual" else "piper_local_305.yaml"
        cfg = RecorderConfig.from_yaml(str(root / "piper_teleop_recorder/configs" / filename))
        cfg.output_dir = temp
        # Keep the normal endpoint available for a real user's camera process.
        import socket
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            cfg.camera.port = probe.getsockname()[1]
        config_path = str(Path(temp) / "config.yaml")
        cfg.save(config_path)
        process = subprocess.Popen(
            ["/usr/bin/python3", "-u", "-m", "pico_video.server", "--no-video",
             "--jpeg-port", str(cfg.camera.port)], cwd=root)
        buttons = Buttons()
        session = PiperRecordingSession(config_path, buttons, 0.105)
        state = {"joint_rad": np.zeros(6), "action_joint_rad": np.zeros(6),
                 "tcp_flange": np.zeros(6), "tcp_eef": [0, 0, 0.105, 0, 0, 0],
                 "gripper_m": 0.03, "action_gripper_m": 0.03,
                 "right_trigger": 0, "right_grip": 0, "teleop_active": False}
        if args.mode == "dual":
            single = state
            state = {}
            for side, sign in (("left", 1), ("right", -1)):
                for key, value in single.items():
                    if key.startswith("right_"):
                        state[key.replace("right_", f"{side}_", 1)] = value
                    else:
                        state[f"{side}_{key}"] = value
                state[f"{side}_joint_rad"] = np.full(6, sign * 0.1)
                state[f"{side}_tcp_flange"] = [0.2, sign * 0.1, 0.4, 0, 0, 0]
                state[f"{side}_tcp_eef"] = [0.2, sign * 0.1, 0.505, 0, 0, 0]
        try:
            # USB stream setup may require 15 s, before the session's 5 s readiness check.
            deadline = time.monotonic() + 20
            from pico_recorder.orbbec_client import OrbbecImageClient
            while True:
                if process.poll() is not None:
                    raise RuntimeError("Local camera process failed")
                probe = OrbbecImageClient("127.0.0.1", cfg.camera.port, 200)
                try:
                    _, frames = probe.fetch_frames()
                finally:
                    probe.close()
                if frames:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("Local camera startup timed out")
                time.sleep(0.1)
            session.prepare()
            for episode in range(2):
                session.update(state)
                buttons.a = True
                session.controller._poll_once()
                for _ in range(12):
                    session.update(state)
                    session.controller._poll_once()  # A remains held, no repeated toggle.
                    time.sleep(0.05)
                buttons.a = False
                session.controller._poll_once()
                buttons.a = True
                session.controller._poll_once()
                buttons.a = False
                session.controller._poll_once()
                path = Path(temp) / cfg.task / "episodes" / f"ep_{episode:05d}.h5"
                with h5py.File(path) as h5:
                    n = int(h5.attrs["steps"])
                    assert n >= 10 and h5.attrs["error"] == "None"
                    assert all(len(ds) == n for ds in h5["v"].values())
                    if args.mode == "dual":
                        assert len(h5.attrs["joint_names"]) == 12
                        np.testing.assert_allclose(h5["v/left_joint_rad"][0], 0.1)
                        np.testing.assert_allclose(h5["v/right_joint_rad"][0], -0.1)
                        assert h5["v/left_tcp_eef"].shape == h5["v/right_tcp_eef"].shape == (n, 6)
                    for serial in cfg.camera.serials:
                        images = h5[f"images/{serial}/jpeg"]
                        assert len(images) == n
                        decoded = cv2.imdecode(np.asarray(images[0], np.uint8), cv2.IMREAD_COLOR)
                        assert decoded.shape == (480, 640, 3)
                    print(f"{args.mode} episode {episode}: {n} aligned state/JPEG rows, 640x480 decode: PASS")
        finally:
            session.close()
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    main()
