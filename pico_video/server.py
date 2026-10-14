"""Gemini 305 RGB -> low-latency mono H264 -> XRoboToolkit headset.

Run with system Python (python3-gi, python3-opencv and GStreamer x264).
The control protocol is shared with the existing Jetson video service.
"""

import argparse
import logging
from pathlib import Path
import subprocess
import threading
import time

import cv2
import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst

from .protocol import VideoServer


def find_rgb_device():
    """Select the Orbbec MJPEG capture node; USB video indices can change."""
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        if "Orbbec" not in (node / "name").read_text():
            continue
        device = "/dev/" + node.name
        result = subprocess.run(["v4l2-ctl", "-d", device, "--list-formats"],
                                capture_output=True, text=True, timeout=3)
        if "'MJPG'" in result.stdout:
            return device
    raise RuntimeError("No Orbbec RGB MJPEG node found (install v4l-utils; check USB).")


class CameraFrames:
    def __init__(self, device, width=1280, height=720, fps=30, fake=False):
        self.lock = threading.Lock()
        self.frame = None
        self.updated = 0.0
        self.wall_ns = 0
        source = ("videotestsrc is-live=true pattern=ball" if fake else
                  f'v4l2src device="{device}" do-timestamp=true ! '
                  f'image/jpeg,width={width},height={height},framerate={fps}/1 ! jpegdec')
        self.pipeline = Gst.parse_launch(
            source + " ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
        )
        self.pipeline.get_by_name("sink").connect("new-sample", self.receive)

    def receive(self, sink):
        sample = sink.emit("pull-sample")
        buffer = sample.get_buffer()
        info = sample.get_caps().get_structure(0)
        width, height = info.get_value("width"), info.get_value("height")
        ok, mapped = buffer.map(Gst.MapFlags.READ)
        if ok:
            try:
                stride = ((width * 3 + 3) // 4) * 4
                frame = np.frombuffer(mapped.data, np.uint8).reshape(height, stride)
                with self.lock:
                    self.frame = frame[:, :width * 3].reshape(height, width, 3).copy()
                    self.updated = time.monotonic()
                    self.wall_ns = time.time_ns()
            finally:
                buffer.unmap(mapped)
        return Gst.FlowReturn.OK

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        # Gemini 305 USB negotiation can take several seconds after reopening.
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            with self.lock:
                if self.frame is not None:
                    return
            error = self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
            if error:
                raise RuntimeError(str(error.parse_error()))
            time.sleep(0.05)
        raise RuntimeError("No RGB frames. Check Gemini 305 RGB device and camera ownership.")

    def compose(self, width, height):
        with self.lock:
            frame, updated = self.frame, self.updated
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        if frame is None or time.monotonic() - updated > 1:
            cv2.putText(canvas, "CAMERA STALE", (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 2)
            return canvas
        h, w = frame.shape[:2]
        scale = min(width / w, height / h)
        out_w, out_h = max(1, int(w * scale)), max(1, int(h * scale))
        x, y = (width - out_w) // 2, (height - out_h) // 2
        canvas[y:y + out_h, x:x + out_w] = cv2.resize(frame, (out_w, out_h))
        return canvas

    def close(self):
        self.pipeline.set_state(Gst.State.NULL)


class Encoder:
    def __init__(self, width, height, fps, bitrate, on_encoded, **unused):
        self.width, self.height, self.fps = width, height, fps
        self.on_encoded = on_encoded
        self.pipeline = Gst.parse_launch(
            f"appsrc name=src is-live=true format=time block=false max-buffers=1 "
            f"leaky-type=downstream caps=video/x-raw,format=BGR,width={width},"
            f"height={height},framerate={fps}/1 ! videoconvert ! video/x-raw,format=I420 ! "
            f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={max(1, bitrate // 1000)} "
            f"key-int-max={fps} bframes=0 byte-stream=true option-string=repeat-headers=1 ! "
            "video/x-h264,profile=baseline,stream-format=byte-stream,alignment=au ! "
            "appsink name=sink emit-signals=true max-buffers=2 drop=false sync=false"
        )
        self.src = self.pipeline.get_by_name("src")
        self.pipeline.get_by_name("sink").connect("new-sample", self.receive)

    def receive(self, sink):
        buffer = sink.emit("pull-sample").get_buffer()
        self.on_encoded(buffer.extract_dup(0, buffer.get_size()), buffer.pts)
        return Gst.FlowReturn.OK

    def start(self):
        return self.pipeline.set_state(Gst.State.PLAYING) != Gst.StateChangeReturn.FAILURE

    def push_frame(self, frame, pts):
        error = self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
        if error:
            raise RuntimeError(str(error.parse_error()))
        rows = frame.reshape(self.height, self.width * 3)
        padding = (-self.width * 3) % 4
        if padding:
            rows = np.pad(rows, ((0, 0), (0, padding)))
        buffer = Gst.Buffer.new_wrapped(rows.tobytes())
        buffer.pts = pts
        buffer.duration = Gst.SECOND // self.fps
        return self.src.emit("push-buffer", buffer) == Gst.FlowReturn.OK

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto", help="305 RGB MJPEG node; auto detects Orbbec")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=13579)
    parser.add_argument("--fake-camera", action="store_true")
    parser.add_argument("--duration", type=float, default=0, help="0 runs until Ctrl+C")
    parser.add_argument("--no-video", action="store_true", help="capture JPEGs only; no headset video")
    parser.add_argument("--jpeg-port", type=int, default=0, help="0 disables local recording endpoint")
    parser.add_argument("--serial", default="CV2856D0009T")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    Gst.init(None)
    device = find_rgb_device() if args.device == "auto" and not args.fake_camera else args.device
    frames = CameraFrames(device, fake=args.fake_camera)
    server = VideoServer(frames, bind=args.bind, port=args.port, encoder_factory=Encoder)
    jpeg_server = None
    try:
        frames.start()
        if args.jpeg_port:
            from .jpeg_server import JpegServer
            jpeg_server = JpegServer(frames, port=args.jpeg_port, serial=args.serial)
            jpeg_server.start()
        if not args.no_video:
            server.start()
        print(f"Gemini 305 RGB ready; JPEG port={args.jpeg_port}; "
              f"PICO video={'off' if args.no_video else server.port}", flush=True)
        deadline = time.monotonic() + args.duration if args.duration else float("inf")
        while time.monotonic() < deadline:
            if jpeg_server and jpeg_server.error:
                raise RuntimeError(jpeg_server.error)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        if jpeg_server:
            jpeg_server.stop()
        frames.close()


if __name__ == "__main__":
    main()
