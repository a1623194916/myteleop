import ipaddress
import logging
import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass


log = logging.getLogger(__name__)


def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        part = sock.recv(size - len(data))
        if not part:
            raise EOFError("connection closed")
        data.extend(part)
    return bytes(data)


def pack_command(command, data=b""):
    encoded = command.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded + struct.pack("<I", len(data)) + data


def read_command(sock):
    size = struct.unpack("<I", recv_exact(sock, 4))[0]
    if not 0 < size <= 128:
        raise ValueError("invalid command length")
    command = recv_exact(sock, size).decode("utf-8")
    size = struct.unpack("<I", recv_exact(sock, 4))[0]
    if size > 4096:
        raise ValueError("invalid payload length")
    return command, recv_exact(sock, size)


@dataclass(frozen=True)
class CameraRequest:
    width: int
    height: int
    fps: int
    bitrate: int
    port: int
    camera: str
    ip: str

    @classmethod
    def parse(cls, data, peer):
        if len(data) < 33 or data[:3] != b"\xca\xfe\x01":
            raise ValueError("invalid camera request header")
        width, height, fps, bitrate, mvhevc, render_mode, port = struct.unpack_from("<7i", data, 3)
        offset = 31
        strings = []
        for _ in range(2):
            if offset >= len(data):
                raise ValueError("missing string length")
            size = data[offset]
            offset += 1
            if offset + size > len(data):
                raise ValueError("truncated string")
            strings.append(data[offset:offset + size].decode("utf-8"))
            offset += size
        camera, address = strings
        if offset != len(data):
            raise ValueError("trailing camera request data")
        if not (16 <= width <= 1920 and 16 <= height <= 1080 and width % 2 == 0 and height % 2 == 0):
            raise ValueError("unsupported resolution")
        if not (1 <= fps <= 60 and 1 <= bitrate <= 50000000 and 1 <= port <= 65535):
            raise ValueError("invalid stream parameters")
        if mvhevc:
            raise ValueError("only mono H264 supported")
        address = address or peer
        if ipaddress.ip_address(address) != ipaddress.ip_address(peer):
            raise ValueError("video destination must match control peer")
        return cls(width, height, fps, bitrate, port, camera, address)


class VideoServer:
    def __init__(self, frames, bind="0.0.0.0", port=13579, fps=30,
                 bitrate=4000000, encoder="nv", encoder_factory=None):
        self.frames = frames
        self.bind = bind
        self.port = port
        self.fps = fps
        self.bitrate = bitrate
        self.encoder = encoder
        self.encoder_factory = encoder_factory
        self._stop = threading.Event()
        self._stream_stop = threading.Event()
        self._listener = None
        self._control = None
        self._video = None
        self._thread = None
        self._stream_thread = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((self.bind, self.port))
            listener.listen(1)
            listener.settimeout(0.2)
        except BaseException:
            listener.close()
            raise
        self._listener = listener
        self.port = listener.getsockname()[1]
        self._stop.clear()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    @staticmethod
    def _close(sock):
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def _serve(self):
        while not self._stop.is_set():
            try:
                conn, peer = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._control = conn
            try:
                while not self._stop.is_set():
                    command, payload = read_command(conn)
                    if command == "OPEN_CAMERA":
                        request = CameraRequest.parse(payload, peer[0])
                        self._close_stream()
                        self._stream_stop.clear()
                        self._stream_thread = threading.Thread(target=self._stream, args=(request,), daemon=True)
                        self._stream_thread.start()
                    elif command == "CLOSE_CAMERA":
                        self._close_stream()
                    elif command == "PING":
                        conn.sendall(pack_command("PONG", payload))
            except (OSError, EOFError, ValueError) as exc:
                log.info("Control connection ended: %s", exc)
            finally:
                self._close_stream()
                self._close(conn)
                self._control = None

    def _stream(self, request):
        encoder = None
        video = None
        packets = queue.Queue(maxsize=8)

        def encoded(data, pts):
            if self._stream_stop.is_set():
                return
            packet = bytes(data)
            if packet:
                try:
                    packets.put_nowait(packet)
                except queue.Full:
                    self._stream_stop.set()

        try:
            factory = self.encoder_factory
            if factory is None:
                if __package__:
                    from .video_frames import GstEncoder
                else:
                    from video_frames import GstEncoder
                factory = GstEncoder
            video = socket.create_connection((request.ip, request.port), timeout=2)
            video.settimeout(0.5)
            video.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._video = video
            fps = min(request.fps, self.fps)
            encoder = factory(width=request.width, height=request.height, fps=fps,
                              bitrate=min(request.bitrate, self.bitrate),
                              use_jetson=self.encoder == "nv", on_encoded=encoded)
            if not encoder.start():
                raise RuntimeError("encoder startup failed")
            origin = time.monotonic()
            deadline = origin
            while not self._stop.is_set() and not self._stream_stop.is_set():
                now = time.monotonic()
                if now >= deadline:
                    frame = self.frames.compose(width=request.width, height=request.height)
                    if not encoder.push_frame(frame, int((now - origin) * 1000000000)):
                        raise RuntimeError("encoder rejected frame")
                    deadline = max(deadline + 1 / fps, time.monotonic())
                encoded_item = None
                if hasattr(encoder, "get_encoded"):
                    encoded_item = encoder.get_encoded(timeout=0.002)
                elif encoded_item is None:
                    try:
                        encoded_item = packets.get_nowait()
                    except queue.Empty:
                        pass
                if encoded_item is None:
                    try:
                        encoded_item = packets.get(timeout=max(0.001, min(0.02, deadline - time.monotonic())))
                    except queue.Empty:
                        continue
                packet = bytes(encoded_item[0]) if isinstance(encoded_item, tuple) else encoded_item
                if packet:
                    video.sendall(struct.pack(">I", len(packet)) + packet)
        except Exception:
            if not self._stop.is_set() and not self._stream_stop.is_set():
                log.exception("Video stream failed")
        finally:
            self._stream_stop.set()
            self._close(video)
            self._video = None
            if encoder is not None:
                encoder.stop()

    def _close_stream(self):
        self._stream_stop.set()
        self._close(self._video)
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=5)
            if self._stream_thread.is_alive():
                raise RuntimeError("video worker did not stop")
            self._stream_thread = None

    def stop(self):
        self._stop.set()
        self._stream_stop.set()
        self._close(self._control)
        self._close(self._video)
        self._close(self._listener)
        if self._thread is not None:
            self._thread.join(timeout=7)
            if self._thread.is_alive():
                raise RuntimeError("control worker did not stop")
            self._thread = None
