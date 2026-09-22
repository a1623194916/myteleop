"""Latest-only ZeroMQ receiver for XRoboToolkit VR messages."""

import json
import math
import time

import zmq


class VRInput:
    def __init__(self, endpoint, stale_timeout=0.20, controller_side="right"):
        self.endpoint = endpoint
        self.stale_timeout = float(stale_timeout)
        if controller_side not in ("left", "right"):
            raise ValueError("controller_side must be left or right")
        self.controller_side = controller_side
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(endpoint)
        self._latest = None
        self._received_at = None

    def poll(self):
        while True:
            try:
                payload = self._socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            message = json.loads(payload.decode("utf-8"))
            self._validate(message, (self.controller_side,))
            self._latest = message
            self._received_at = time.monotonic()
        return self._latest

    def is_stale(self, now=None):
        if self._received_at is None:
            return True
        current = time.monotonic() if now is None else now
        return current - self._received_at > self.stale_timeout

    @staticmethod
    def _validate(message, controller_sides=("left", "right")):
        for controller_side in controller_sides:
            side = f"{controller_side}_controller"
            controller = message.get(side)
            if not isinstance(controller, dict):
                raise ValueError(f"VR message is missing {side}")
            position = controller.get("position", [])
            orientation = controller.get("orientation", [])
            if len(position) != 3 or len(orientation) != 4:
                raise ValueError(f"VR message has an invalid {side} pose")
            pose_values = position + orientation
            if not all(math.isfinite(float(value)) for value in pose_values):
                raise ValueError(f"VR message has non-finite values in {side} pose")
            quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in orientation))
            if quaternion_norm < 0.5 or quaternion_norm > 1.5:
                raise ValueError(f"VR message has an invalid {side} quaternion")

    def close(self):
        self._socket.close()
        self._context.term()
