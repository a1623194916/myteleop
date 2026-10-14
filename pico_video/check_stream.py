"""Exercise the PICO protocol and decode real USB camera video without a headset."""

import argparse
import socket
import struct
import subprocess

from .protocol import VideoServer, pack_command, recv_exact
from .server import CameraFrames, Encoder, Gst, find_rgb_device


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fake-camera", action="store_true")
    args = parser.parse_args()
    Gst.init(None)
    frames = CameraFrames("unused" if args.fake_camera else find_rgb_device(), fake=args.fake_camera)
    server = VideoServer(frames, bind="127.0.0.1", port=0, encoder_factory=Encoder)
    try:
        frames.start()
        server.start()
        # Two successive sessions verify that CLOSE/disconnect releases workers
        # and a new OPEN_CAMERA starts a decodable stream with fresh SPS/PPS.
        for session in range(2):
            with socket.socket() as receiver:
                receiver.bind(("127.0.0.1", 0))
                receiver.listen(1)
                receiver.settimeout(8)
                with socket.create_connection(("127.0.0.1", server.port), timeout=5) as control:
                    payload = b"\xca\xfe\x01" + struct.pack(
                        "<7i", 1280, 720, 30, 4000000, 0, 0, receiver.getsockname()[1])
                    for value in (b"PANORAMA", b"127.0.0.1"):
                        payload += bytes([len(value)]) + value
                    control.sendall(pack_command("OPEN_CAMERA", payload))
                    stream, _ = receiver.accept()
                    with stream:
                        stream.settimeout(5)
                        packets = []
                        for _ in range(30):
                            length = struct.unpack(">I", recv_exact(stream, 4))[0]
                            if not 0 < length < 8000000:
                                raise RuntimeError("Invalid H264 packet size")
                            packets.append(recv_exact(stream, length))
                    control.sendall(pack_command("CLOSE_CAMERA"))
            result = subprocess.run(
                ["ffmpeg", "-v", "error", "-f", "h264", "-i", "pipe:0",
                 "-frames:v", "30", "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
                input=b"".join(packets), capture_output=True, timeout=15, check=True)
            expected = 30 * 1280 * 720 * 3
            if len(result.stdout) != expected or result.stderr:
                raise RuntimeError(f"Decode failed: {len(result.stdout)}/{expected}, {result.stderr!r}")
            print(f"Session {session + 1}: OPEN_CAMERA -> 30 H264 AUs -> 30 decoded RGB frames: PASS")
    finally:
        server.stop()
        frames.close()


if __name__ == "__main__":
    main()
