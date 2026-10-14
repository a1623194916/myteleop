"""Local JPEG endpoint shared by recording and optional headset video."""

import json
import threading
import time

import cv2
import numpy as np
import zmq


class JpegServer:
    def __init__(self, frames, port=8765, serial="CV2856D0009T", width=640, height=480):
        self.frames, self.port, self.serial = frames, port, serial
        self.width, self.height = width, height
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        if not self.ready.wait(5):
            raise RuntimeError("JPEG server startup timed out")
        if self.error:
            raise RuntimeError(self.error)

    def run(self):
        context = zmq.Context()
        sock = context.socket(zmq.REP)
        sock.setsockopt(zmq.LINGER, 0)
        cached_ns, jpg = 0, b""
        try:
            sock.bind(f"tcp://127.0.0.1:{self.port}")
            self.ready.set()
            while not self.stop_event.is_set():
                if not sock.poll(100):
                    continue
                sock.recv()
                with self.frames.lock:
                    frame, updated, ns = self.frames.frame, self.frames.updated, self.frames.wall_ns
                if frame is None or time.monotonic() - updated > 1:
                    sock.send_json({"serials": [], "frames": {}})
                    continue
                if ns != cached_ns:
                    # Letterbox without changing the capture timestamp.
                    h, w = frame.shape[:2]
                    scale = min(self.width / w, self.height / h)
                    new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
                    x, y = (self.width - new_w) // 2, (self.height - new_h) // 2
                    canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    canvas[y:y + new_h, x:x + new_w] = cv2.resize(frame, (new_w, new_h))
                    ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    if not ok:
                        raise RuntimeError("JPEG encoding failed")
                    jpg, cached_ns = encoded.tobytes(), ns
                meta = {"serials": [self.serial], "frames": {
                    self.serial: {"ns": cached_ns, "w": self.width, "h": self.height}}}
                sock.send_multipart([json.dumps(meta).encode(), jpg])
        except Exception as exc:
            self.error = repr(exc)
            self.ready.set()
        finally:
            sock.close()
            context.term()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)
