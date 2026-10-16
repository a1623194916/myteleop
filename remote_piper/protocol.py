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

"""RPC wire protocol for the remote Piper "dumb executor" bridge.

The RLinf server runs the policy and sends robot *actions* to a remote robot
machine, which executes them and returns *observations*. Both sides import this
single module, so it must stay dependency-light: only :mod:`numpy`,
:mod:`msgpack`, :mod:`cv2` and the standard library. Do **not** import torch,
ray, zmq, gymnasium, or anything from the ``rlinf`` package here -- this file is
copied to the robot machine standalone.

Wire format (all envelopes are msgpack-encoded dicts):

Request (server -> robot)::

    {"kind": "reset"|"step"|"close", "action": <array-blob> | None}

Response (robot -> server), discriminated by ``"type"``::

    {"type": "obs", "state": {name: <array-blob>}, "frames": {name: <jpeg>},
     "human": {"reward": float|None, "done": bool, "stage_event": int|None}}
    {"type": "error", "msg": <str>}

The ``"human"`` key is **optional** (P2 human-in-the-loop reward): it is absent
whenever the robot side has no human-feedback source, so every pre-P2 obs
envelope and every ``encode_obs(state, frames)`` call round-trips unchanged.

Numeric arrays (joints / poses / gripper / actions) are packed as raw
``float32`` bytes plus shape so values and shapes round-trip exactly. Only
camera frames are JPEG-compressed (lossy) via OpenCV.
"""

from typing import Any, Optional

import cv2
import msgpack
import numpy as np

__all__ = [
    "VALID_KINDS",
    "jpeg_encode",
    "jpeg_decode",
    "encode_request",
    "decode_request",
    "encode_obs",
    "decode_obs",
    "encode_error",
    "decode_response",
]

# Allowed request kinds (server -> robot).
VALID_KINDS = ("reset", "step", "close")

# JPEG quality for camera-frame compression (0-100). 90 keeps frames crisp
# while still giving a large bandwidth win over raw RGB.
_JPEG_QUALITY = 90


# --------------------------------------------------------------------------- #
# Image (lossy) codec
# --------------------------------------------------------------------------- #
def jpeg_encode(img_uint8_hwc: np.ndarray) -> bytes:
    """Encode an ``HxWx3`` uint8 image to JPEG bytes."""
    arr = np.ascontiguousarray(img_uint8_hwc)
    if arr.dtype != np.uint8:
        raise ValueError(f"jpeg_encode expects uint8 image, got {arr.dtype}")
    ok, buf = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), _JPEG_QUALITY])
    if not ok:
        raise ValueError("cv2.imencode failed to encode image to JPEG")
    return buf.tobytes()


def jpeg_decode(data: bytes) -> np.ndarray:
    """Decode JPEG bytes back to an ``HxWx3`` uint8 image."""
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("cv2.imdecode failed to decode JPEG bytes")
    return img


# --------------------------------------------------------------------------- #
# Lossless float32 array (de)serialization
# --------------------------------------------------------------------------- #
def _pack_array(arr: np.ndarray) -> dict[str, Any]:
    """Pack an array as raw float32 bytes + shape (lossless, no JPEG)."""
    a = np.ascontiguousarray(arr, dtype=np.float32)
    return {"shape": list(a.shape), "data": a.tobytes()}


def _unpack_array(blob: dict[str, Any]) -> np.ndarray:
    """Inverse of :func:`_pack_array`; returns a writable float32 array."""
    shape = tuple(int(d) for d in blob["shape"])
    return np.frombuffer(blob["data"], dtype=np.float32).reshape(shape).copy()


def _packb(envelope: dict[str, Any]) -> bytes:
    return msgpack.packb(envelope, use_bin_type=True)


def _unpackb(data: bytes) -> dict[str, Any]:
    return msgpack.unpackb(data, raw=False)


# --------------------------------------------------------------------------- #
# Request (server -> robot)
# --------------------------------------------------------------------------- #
def encode_request(kind: str, action: Optional[np.ndarray]) -> bytes:
    """Encode a control request.

    Args:
        kind: One of :data:`VALID_KINDS` (``"reset"``, ``"step"``, ``"close"``).
        action: Action array for ``"step"``; ``None`` for ``"reset"``/``"close"``.
    """
    if kind not in VALID_KINDS:
        raise ValueError(
            f"unknown request kind {kind!r}; expected one of {VALID_KINDS}"
        )
    payload = None if action is None else _pack_array(action)
    return _packb({"kind": kind, "action": payload})


def decode_request(data: bytes) -> dict[str, Any]:
    """Decode a request into ``{"kind": str, "action": np.ndarray | None}``."""
    env = _unpackb(data)
    action = env.get("action")
    return {
        "kind": env["kind"],
        "action": None if action is None else _unpack_array(action),
    }


# --------------------------------------------------------------------------- #
# Optional human-in-the-loop feedback (P2)
# --------------------------------------------------------------------------- #
def _pack_human(human: dict[str, Any]) -> dict[str, Any]:
    """Normalize a human-feedback dict to plain msgpack scalars.

    Shape: ``{"reward": float|None, "done": bool, "stage_event": int|None}``.
    Missing keys default to a neutral value (``reward``/``stage_event`` -> None,
    ``done`` -> False).
    """
    reward = human.get("reward")
    stage_event = human.get("stage_event")
    return {
        "reward": None if reward is None else float(reward),
        "done": bool(human.get("done", False)),
        "stage_event": None if stage_event is None else int(stage_event),
    }


def _unpack_human(blob: dict[str, Any]) -> dict[str, Any]:
    """Inverse of :func:`_pack_human`; tolerant of missing keys."""
    return {
        "reward": blob.get("reward"),
        "done": bool(blob.get("done", False)),
        "stage_event": blob.get("stage_event"),
    }


# --------------------------------------------------------------------------- #
# Observation response (robot -> server)
# --------------------------------------------------------------------------- #
def encode_obs(
    state: dict[str, np.ndarray],
    frames: dict[str, np.ndarray],
    human: Optional[dict[str, Any]] = None,
) -> bytes:
    """Encode an observation: raw float32 ``state`` + JPEG-compressed ``frames``.

    ``human`` is optional human-in-the-loop feedback (P2). When ``None`` the
    ``"human"`` key is omitted entirely, so the envelope is byte-compatible with
    the pre-P2 wire format.
    """
    envelope: dict[str, Any] = {
        "type": "obs",
        "state": {name: _pack_array(arr) for name, arr in state.items()},
        "frames": {name: jpeg_encode(img) for name, img in frames.items()},
    }
    if human is not None:
        envelope["human"] = _pack_human(human)
    return _packb(envelope)


def decode_obs(data: bytes) -> dict[str, Any]:
    """Decode an observation envelope into ``{"state": ..., "frames": ...}``.

    A ``"human"`` key is included only when the envelope carried one.
    """
    env = _unpackb(data)
    if env.get("type") != "obs":
        raise ValueError(f"expected an obs response, got type {env.get('type')!r}")
    return _decode_obs_env(env)


def _decode_obs_env(env: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {
        "state": {name: _unpack_array(blob) for name, blob in env["state"].items()},
        "frames": {name: jpeg_decode(buf) for name, buf in env["frames"].items()},
    }
    human = env.get("human")
    if human is not None:
        out["human"] = _unpack_human(human)
    return out


# --------------------------------------------------------------------------- #
# Error response (robot -> server)
# --------------------------------------------------------------------------- #
def encode_error(msg: str) -> bytes:
    """Encode an error response carrying a human-readable message."""
    return _packb({"type": "error", "msg": str(msg)})


def decode_response(data: bytes) -> dict[str, Any]:
    """Decode any robot response, discriminating obs vs error by ``"type"``.

    Returns a dict with a ``"type"`` key:

    * ``{"type": "obs", "state": ..., "frames": ...}``
    * ``{"type": "error", "msg": str}``
    """
    env = _unpackb(data)
    resp_type = env.get("type")
    if resp_type == "obs":
        out = _decode_obs_env(env)
        out["type"] = "obs"
        return out
    if resp_type == "error":
        return {"type": "error", "msg": env["msg"]}
    raise ValueError(f"unknown response type {resp_type!r}")
