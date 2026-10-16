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

"""Robot-side camera backends for the remote Piper "dumb executor" bridge.

A server loop on the robot machine reads cameras through a :class:`CameraBackend`
(``read`` -> one square ``uint8`` HWC image per camera serial, ``close`` to
release) and ships the frames to the policy server. This module runs **on the
robot**, so it imports only :mod:`numpy`, :mod:`abc` and the standard library at
module top; the heavy libs (:mod:`zmq`, :mod:`cv2`, :mod:`pyrealsense2`) are
imported *lazily* inside the real adapters so the file loads cleanly without
them and can be copied to the robot standalone. It must not pull torch / ray /
gymnasium or anything from ``rlinf``, nor the sibling protocol / rpc modules.

Every camera on this robot is an **Orbbec**, streamed as JPEG frames from a
Jetson stream server. The primary real adapter is therefore
:class:`ZmqOrbbecCameraBackend`, a thin wrapper over the robot's existing
picosoftware ``pico_recorder.orbbec_client.OrbbecImageClient`` (ZMQ ``REQ`` ->
one bundle of all cameras' JPEGs) plus a :mod:`cv2` decode + center-crop +
resize. :class:`RealsenseCameraBackend` is kept only as a disabled stub
(RealSense is not used on this robot).

Only :class:`FakeCameraBackend` is unit-tested here. :class:`ZmqOrbbecCameraBackend`
is the real adapter, finalized against ``orbbec_client.py`` confirmed on machine
41; it needs ``zmq`` / ``cv2`` and a live Jetson stream server, so it is verified
in a hardware session, not in CI.

Channel order: a JPEG/cv2 round-trip is symmetric, but the base policy expects a
specific RGB/BGR channel order. :mod:`cv2` decodes to **BGR**; the ``channel_order``
ctor arg selects what the backend emits (default ``"rgb"`` -- flip BGR->RGB, the
usual order VLA image encoders expect). This default is a documented choice and
MUST be re-confirmed against the actual base checkpoint when it lands (plan
Task 8); set ``channel_order="bgr"`` to pass the decoded order through untouched.
"""

from abc import ABC, abstractmethod

import numpy as np

__all__ = [
    "CameraBackend",
    "FakeCameraBackend",
    "ZmqOrbbecCameraBackend",
    "RealsenseCameraBackend",
]

_DEFAULT_SIZE = 128


def _center_crop_resize(img: np.ndarray, size: int):
    """Center-crop ``img`` (HWC uint8) to a square, then resize to ``size``.

    Uses :mod:`cv2` (imported lazily, treated like a hardware lib so the module
    top stays import-light). Channel-order agnostic: it operates per-pixel and
    leaves the channel order untouched, so the caller applies the configured
    RGB/BGR order (see :class:`ZmqOrbbecCameraBackend`) separately.
    """
    import cv2  # lazy; VERIFY ON HARDWARE: channel order vs the base policy

    h, w = img.shape[:2]
    crop = min(h, w)
    y0 = (h - crop) // 2
    x0 = (w - crop) // 2
    cropped = img[y0 : y0 + crop, x0 : x0 + crop]
    resized = cv2.resize(cropped, (size, size), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(resized, dtype=np.uint8)


class CameraBackend(ABC):
    """Abstract multi-camera reader.

    Implementations own the physical cameras. The server loop calls
    :meth:`read` once per control tick and :meth:`close` on shutdown.
    """

    @abstractmethod
    def read(self) -> dict:
        """Return ``{serial: image}``; each image is ``uint8`` ``(H, W, 3)`` HWC."""

    @abstractmethod
    def close(self) -> None:
        """Release the cameras. Must be idempotent."""


class FakeCameraBackend(CameraBackend):
    """Pure-numpy stand-in that returns random-noise frames, no hardware.

    Args:
        serials: Camera serials/names; one frame per serial, keyed by serial.
        size: Side length of the square output frames (``size x size x 3``).
    """

    def __init__(self, serials: list, size: int = _DEFAULT_SIZE) -> None:
        self.serials = [str(s) for s in serials]
        self.size = int(size)
        self._rng = np.random.default_rng()

    def read(self) -> dict:
        return {
            s: self._rng.integers(
                0, 256, size=(self.size, self.size, 3), dtype=np.uint8
            )
            for s in self.serials
        }

    def close(self) -> None:
        return None


class ZmqOrbbecCameraBackend(CameraBackend):
    """Primary real backend: Orbbec frames over ZMQ as JPEG (finalized vs 41).

    Thin wrapper over the robot's existing
    ``pico_recorder.orbbec_client.OrbbecImageClient``: one ``fetch_frames()``
    call does a single ZMQ ``REQ``/``REP`` round-trip that returns **all**
    cameras' latest JPEG frames in one bundle, which this class decodes with
    :mod:`cv2`, center-crops to a square and resizes to ``size`` x ``size``.
    ``OrbbecImageClient`` (and :mod:`cv2`) are imported lazily so this file
    stays import-light; its package lives outside site-packages on the robot,
    so ``pico_software_path`` is prepended to ``sys.path`` before the import.

    Confirmed on machine 41 (``/home/u22/kyz/pico_software``):

    * ``OrbbecImageClient(host, port, timeout_ms=300)`` with
      ``fetch_frames() -> (meta, frames)`` where
      ``frames = {serial: {"jpg": bytes, "ns": int, "w": int, "h": int}}``;
      a timeout / no-camera condition returns ``(None, None)``.
    * Jetson stream server at ``192.168.5.27:8765``; serials
      ``CV2R46P00097`` and ``CPC85630003C`` (both 640x480).

    Args:
        endpoint_host: Jetson stream-server host (default ``"192.168.5.27"``).
        endpoint_port: Jetson stream-server port (default ``8765``).
        serials: Camera serials to return; keys of the dict from :meth:`read`.
            Each must be present in the server's bundle or :meth:`read` raises.
        size: Side length of the square output frames.
        channel_order: ``"rgb"`` (default) flips cv2's BGR to RGB; ``"bgr"``
            passes the decoded order through. See the module docstring -- the
            default is a documented choice to re-confirm against the base ckpt.
        timeout_s: Per-``read`` receive timeout; a silent server raises instead
            of hanging the control loop (mapped to ``OrbbecImageClient``'s ms).
        pico_software_path: Dir prepended to ``sys.path`` so ``pico_recorder``
            imports (default ``"/home/u22/kyz/pico_software"``, the robot path).
        endpoint: Legacy ``"tcp://host:port"`` form (call-site compatibility,
            e.g. ``robot_server.main``); when given it overrides
            ``endpoint_host`` / ``endpoint_port``.
    """

    def __init__(
        self,
        endpoint_host: str = "192.168.5.27",
        endpoint_port: int = 8765,
        serials: list | None = None,
        size: int = _DEFAULT_SIZE,
        channel_order: str = "rgb",
        timeout_s: float = 1.0,
        pico_software_path: str = "/home/u22/kyz/pico_software",
        endpoint: str | None = None,
    ) -> None:
        channel_order = str(channel_order).lower()
        if channel_order not in ("rgb", "bgr"):
            raise ValueError(
                f"channel_order must be 'rgb' or 'bgr', got {channel_order!r}"
            )

        # Accept the legacy "tcp://host:port" endpoint (robot_server / older
        # call sites) and split it into host + port for OrbbecImageClient.
        if endpoint is not None:
            hostport = str(endpoint).split("://", 1)[-1]
            host, _, port = hostport.rpartition(":")
            endpoint_host = host or endpoint_host
            if port:
                endpoint_port = int(port)

        self.endpoint_host = str(endpoint_host)
        self.endpoint_port = int(endpoint_port)
        self.serials = [str(s) for s in (serials or [])]
        self.size = int(size)
        self.channel_order = channel_order
        self.timeout_s = float(timeout_s)
        self.pico_software_path = str(pico_software_path)

        # Lazy hardware import: pico_recorder lives outside site-packages on the
        # robot, so make it importable, then open the ZMQ client (its connect is
        # non-blocking; a dead server surfaces as a timeout in read()).
        import sys

        if self.pico_software_path and self.pico_software_path not in sys.path:
            sys.path.insert(0, self.pico_software_path)
        from pico_recorder.orbbec_client import OrbbecImageClient

        timeout_ms = max(1, int(self.timeout_s * 1000))
        self._client = OrbbecImageClient(
            self.endpoint_host, self.endpoint_port, timeout_ms=timeout_ms
        )

    def read(self) -> dict:
        # One round-trip fetches every camera's latest frame as a bundle; pick
        # out the requested serials, JPEG-decode, crop/resize, fix channel order.
        import cv2  # lazy; decodes to BGR -- see channel_order

        meta, frames = self._client.fetch_frames()
        if frames is None:
            raise RuntimeError(
                "Orbbec stream server returned no frames "
                f"(timeout {self.timeout_s}s or no cameras @ "
                f"{self.endpoint_host}:{self.endpoint_port})"
            )
        out: dict = {}
        for serial in self.serials:
            entry = frames.get(serial)
            if entry is None:
                raise RuntimeError(
                    f"camera serial {serial!r} not in stream bundle "
                    f"(available: {sorted(frames)})"
                )
            jpeg = entry.get("jpg")
            if not jpeg:
                raise RuntimeError(f"empty JPEG payload for camera {serial!r}")
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)  # cv2 decodes to BGR
            if img is None:
                raise RuntimeError(f"failed to JPEG-decode frame for {serial!r}")
            frame = _center_crop_resize(img, self.size)
            if self.channel_order == "rgb":  # cv2 BGR -> RGB for the base policy
                frame = np.ascontiguousarray(frame[..., ::-1])
            out[serial] = frame
        return out

    def close(self) -> None:
        client, self._client = getattr(self, "_client", None), None
        if client is not None:
            # OrbbecImageClient owns its own zmq context; close() tears down
            # both its socket and that context.
            client.close()


class RealsenseCameraBackend(CameraBackend):
    """Disabled stub -- RealSense is NOT used on this robot (all cams Orbbec).

    Kept only for API parity / future bring-up. Use
    :class:`ZmqOrbbecCameraBackend` instead. A full ``pyrealsense2`` adapter
    (pipeline per serial -> color frame -> center-crop/resize) can be filled in
    here later (plan Task 8) if a RealSense is ever added.
    """

    def __init__(self, *args, **kwargs) -> None:
        raise NotImplementedError(
            "RealsenseCameraBackend is not used on this robot; all cameras are "
            "Orbbec -- use ZmqOrbbecCameraBackend."
        )

    def read(self) -> dict:
        raise NotImplementedError

    def close(self) -> None:
        return None
