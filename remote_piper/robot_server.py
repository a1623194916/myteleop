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

"""Robot-side ZMQ ``REP`` server loop for the remote Piper "dumb executor".

This runs **on the robot machine**. The RLinf policy server sends control
requests (``reset`` / ``step`` / ``close``) through :class:`RobotRpcClient`
(Task 2) over a ZMQ ``REQ`` socket; this ``REP`` server answers each one by
driving an :class:`~.arm_backend.ArmBackend` and reading a
:class:`~.camera_backend.CameraBackend`, then shipping the observation back.
The loop is a strict ``recv`` -> handle -> ``send`` cycle: every request yields
exactly one reply (an observation, or an ``encode_error`` envelope on any
fault), so the ``REQ``/``REP`` pair never desynchronizes.

Only :mod:`zmq`, the sibling :mod:`protocol` module and the backend ABCs are
imported at module top -- the file must load on a machine without the real
hardware libraries. The real backends (:class:`PiperArmBackend`,
:class:`ZmqOrbbecCameraBackend`) are imported *lazily* inside :func:`main`, so
the module and its tests import cleanly with Fake backends only. Imports are
package-relative so the whole ``remote_piper`` package can be copied to the
robot and run standalone.
"""

import argparse
import threading
import time
from typing import Any, Optional

import zmq

from . import protocol
from .arm_backend import ArmBackend
from .camera_backend import CameraBackend

__all__ = [
    "HumanFeedbackSource",
    "KeyboardListenerFeedback",
    "RobotServer",
    "main",
]

# Default bind endpoint: all interfaces, port 5555.
_DEFAULT_BIND = "tcp://0.0.0.0:5555"
# How often (ms) the serve loop wakes from ``poll`` to re-check the stop flag.
# Small enough that ``stop()`` + ``join()`` return promptly; large enough to
# avoid busy-spinning while idle.
_POLL_INTERVAL_MS = 100


# --------------------------------------------------------------------------- #
# Human-in-the-loop feedback (P2) -- read on the robot machine (where the human
# operator is), shipped back inside the obs response.
# --------------------------------------------------------------------------- #
class HumanFeedbackSource:
    """Pluggable per-step human reward/done source read on the robot machine.

    The server calls :meth:`read` once per ``step`` and includes the returned
    dict in the obs response. Implementations must be non-blocking; return the
    *latest* feedback, not block waiting for a keypress.
    """

    def read(self) -> Optional[dict[str, Any]]:
        """Return ``{"reward": float|None, "done": bool, "stage_event": int|None}``.

        May return ``None`` to send no human fields for this step.
        """
        raise NotImplementedError

    def close(self) -> None:
        """Release any resources. Default is a no-op."""


class KeyboardListenerFeedback(HumanFeedbackSource):
    """Human feedback from a physical keyboard on the robot machine.

    Reuses :class:`~rlinf.envs.realworld.common.keyboard.keyboard_listener.KeyboardListener`
    (same key semantics as ``KeyboardRewardDoneWrapper``):

    * ``c`` -> ``+success_reward`` and ``done`` (task success),
    * ``a`` -> ``+failure_reward`` and ``done`` (task failure),
    * ``b`` -> neutral ``0.0`` reward, no ``done``,
    * a stage-mark key (``x`` or space) -> ``stage_event`` (lossless edge).

    The ``KeyboardListener`` (and its ``evdev`` dependency) is imported and
    constructed lazily so this module stays import-light / robot-copyable and
    tests can inject a fake ``listener``.
    """

    def __init__(
        self,
        *,
        stage_keys: tuple[str, ...] = ("x", "Key.space"),
        success_reward: float = 1.0,
        failure_reward: float = -1.0,
        listener: Any = None,
    ) -> None:
        if listener is None:
            from rlinf.envs.realworld.common.keyboard.keyboard_listener import (
                KeyboardListener,
            )

            listener = KeyboardListener()
        self._listener = listener
        self._stage_keys = tuple(stage_keys)
        self._success_reward = float(success_reward)
        self._failure_reward = float(failure_reward)
        self._stage_count = 0

    def read(self) -> dict[str, Any]:
        stage_event: Optional[int] = None
        try:
            pressed = self._listener.pop_pressed_keys()
        except Exception:  # noqa: BLE001 - a listener hiccup must not crash step
            pressed = []
        if any(k in pressed for k in self._stage_keys):
            self._stage_count += 1
            stage_event = self._stage_count

        key = self._listener.get_key()
        reward: Optional[float] = None
        done = False
        if key == "c":
            reward, done = self._success_reward, True
        elif key == "a":
            reward, done = self._failure_reward, True
        elif key == "b":
            reward, done = 0.0, False
        return {"reward": reward, "done": done, "stage_event": stage_event}


class RobotServer:
    """Strict one-request/one-reply ``REP`` server driving arm + camera.

    Args:
        arm: Arm backend to drive (``reset`` / ``apply`` / ``read_state``).
        camera: Camera backend to read frames from each tick.
        bind: ZMQ endpoint to bind the ``REP`` socket to. Use a ``:0`` port
            (e.g. ``tcp://127.0.0.1:0``) to get an OS-chosen port, then read
            :attr:`bound_port`.
        rate_hz: Control-loop rate cap for ``step``. Each ``step`` is paced to
            take at least ``1 / rate_hz`` seconds before replying. ``<= 0``
            disables pacing.
        human_feedback: Optional :class:`HumanFeedbackSource` (P2). When set,
            its latest value is read once per ``step`` and shipped in the obs
            response's ``"human"`` field; when ``None`` no human field is sent.

    The backends are **owned by the caller**: :meth:`stop` tears down only the
    ZMQ socket/context, never the backends. The caller closes those (see
    :func:`main`).
    """

    def __init__(
        self,
        arm: ArmBackend,
        camera: CameraBackend,
        bind: str = _DEFAULT_BIND,
        rate_hz: float = 10.0,
        human_feedback: Optional[HumanFeedbackSource] = None,
    ) -> None:
        self.arm = arm
        self.camera = camera
        self.rate_hz = float(rate_hz)
        self._period = (1.0 / self.rate_hz) if self.rate_hz > 0 else 0.0
        # Optional human-in-the-loop reward/done source (P2). When ``None`` the
        # obs response carries no ``"human"`` field, so sim/dry-run is unaffected.
        self._human_feedback = human_feedback
        self._stop_event = threading.Event()
        self._closed = False

        # Private context so termination is isolated and cannot block (LINGER=0
        # on the socket means no undelivered message pins the context open).
        self._ctx = zmq.Context()
        self._socket = self._ctx.socket(zmq.REP)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(bind)
        endpoint = self._socket.getsockopt_string(zmq.LAST_ENDPOINT)
        # Expose the actually-bound port so a ``:0`` bind is testable.
        self.bound_port = int(endpoint.rsplit(":", 1)[1])

    # ------------------------------------------------------------------ #
    # One request -> one reply
    # ------------------------------------------------------------------ #
    def handle_one(self) -> None:
        """Receive one request, dispatch it, and send exactly one reply.

        The reply is an ``encode_obs`` envelope on success or an
        ``encode_error`` envelope on *any* exception -- the ``REP`` socket is
        never left without a reply (that would desync the ``REQ`` peer).
        """
        data = self._socket.recv()
        try:
            req = protocol.decode_request(data)
            kind = req["kind"]
            if kind == "reset":
                self.arm.reset()
                reply = protocol.encode_obs(self.arm.read_state(), self.camera.read())
            elif kind == "step":
                t0 = time.perf_counter()
                self.arm.apply(req["action"])
                human = self._read_human_feedback()
                reply = protocol.encode_obs(
                    self.arm.read_state(), self.camera.read(), human=human
                )
                elapsed = time.perf_counter() - t0
                if self._period > 0.0:
                    time.sleep(max(0.0, self._period - elapsed))
            elif kind == "close":
                # Stop first so the loop winds down even if obs-building fails;
                # still reply once so the client's close() recv succeeds.
                self._stop_event.set()
                reply = protocol.encode_obs(self.arm.read_state(), self.camera.read())
            else:
                raise ValueError(f"unknown request kind {kind!r}")
        except Exception as exc:  # noqa: BLE001 - must always reply, never crash
            reply = protocol.encode_error(str(exc))
        self._socket.send(reply)

    def _read_human_feedback(self) -> Optional[dict[str, Any]]:
        """Read the latest human feedback, or ``None`` when no source is set.

        A faulty source must never desync the REP loop, so any exception is
        swallowed and treated as "no human feedback this step".
        """
        if self._human_feedback is None:
            return None
        try:
            return self._human_feedback.read()
        except Exception:  # noqa: BLE001 - a feedback fault must not drop the reply
            return None

    # ------------------------------------------------------------------ #
    # Serve loop + lifecycle
    # ------------------------------------------------------------------ #
    def serve_forever(self) -> None:
        """Handle requests until :meth:`stop` (or a ``close`` request) fires.

        Polls with a short timeout so an external :meth:`stop` is observed
        promptly instead of blocking forever in ``recv``. Always releases the
        ZMQ socket + context on exit (``LINGER=0``), so the owning thread can be
        joined without hanging.
        """
        poller = zmq.Poller()
        poller.register(self._socket, zmq.POLLIN)
        try:
            while not self._stop_event.is_set():
                socks = dict(poller.poll(timeout=_POLL_INTERVAL_MS))
                if self._socket in socks:
                    self.handle_one()
        except zmq.ZMQError:
            # Context terminated / socket closed out from under us -> exit quietly.
            pass
        finally:
            self._close_zmq()

    def stop(self) -> None:
        """Signal the serve loop to exit. Idempotent; backends are untouched."""
        self._stop_event.set()

    def _close_zmq(self) -> None:
        """Close the ``REP`` socket and terminate the context (idempotent)."""
        if self._closed:
            return
        self._closed = True
        sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.close(linger=0)
            except zmq.ZMQError:
                pass
        ctx, self._ctx = self._ctx, None
        if ctx is not None:
            try:
                ctx.term()
            except zmq.ZMQError:
                pass


# --------------------------------------------------------------------------- #
# Real-hardware entry point (machine 41)
# --------------------------------------------------------------------------- #
def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Robot-side ZMQ REP server for the remote Piper bridge.",
    )
    p.add_argument(
        "--bind",
        default=_DEFAULT_BIND,
        help=f"ZMQ bind endpoint (default: {_DEFAULT_BIND}).",
    )
    p.add_argument(
        "--port",
        type=int,
        default=None,
        help="Override just the port of --bind (binds on 0.0.0.0).",
    )
    p.add_argument(
        "--rate-hz",
        type=float,
        default=10.0,
        help="Control-loop rate cap for step (default: 10).",
    )
    p.add_argument(
        "--reward-done-mode",
        default="none",
        choices=["none", "human", "keyboard"],
        help=(
            "Human-in-the-loop reward/done source (P2). 'none' (default) sends "
            "no human fields (sim/dry-run). 'human'/'keyboard' reads a physical "
            "keyboard on this machine each step (c=success+done, a=fail+done, "
            "b=neutral, x/space=stage mark)."
        ),
    )
    # Cameras (ZmqOrbbecCameraBackend).
    p.add_argument(
        "--cameras",
        nargs="+",
        default=[],
        metavar="SERIAL",
        help="Camera serials/names to request from the Jetson stream server.",
    )
    p.add_argument(
        "--camera-endpoint",
        default="tcp://127.0.0.1:5556",
        help="ZMQ endpoint of the Orbbec Jetson stream server.",
    )
    p.add_argument("--camera-size", type=int, default=128, help="Square frame size.")
    p.add_argument(
        "--camera-timeout-s",
        type=float,
        default=1.0,
        help="Per-read camera recv timeout (s).",
    )
    # Piper arm (PiperArmBackend).
    p.add_argument("--can-channel", default="can0", help="SocketCAN interface.")
    p.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate.")
    p.add_argument("--robot-id", default="piper_x", help="pyAgxArm robot id.")
    p.add_argument(
        # ``--gripper-max-width`` is the canonical name; ``--max-gripper-width``
        # is kept as a back-compat alias (older DEPLOY commands used it).
        "--gripper-max-width",
        "--max-gripper-width",
        dest="max_gripper_width",
        type=float,
        default=0.07,
        help="Fully-open gripper width (m); the [0,1] opening scales by this.",
    )
    p.add_argument("--gripper-force", type=float, default=1.0, help="Gripper force.")
    p.add_argument(
        "--max-step-rad",
        type=float,
        default=0.006,
        help="Per-step joint delta cap (rad); bounds how far a single step moves.",
    )
    p.add_argument(
        "--tcp-frame",
        default="eef",
        choices=["eef", "flange"],
        help=(
            "Which TCP frame the EE-delta action and the proprio tcp_pose use: "
            "'eef' (gripper TCP, default) or 'flange' (link6). Must match the "
            "frozen base's proprio frame."
        ),
    )
    p.add_argument(
        "--delta-frame",
        default="world",
        choices=["world"],
        help=(
            "Frame the 7-D EE-delta action is composed in. Only 'world' "
            "(base-frame additive dpos + left-multiply axis-angle drot) is "
            "implemented (see eef_math.compose_ee_delta); it must match the demo "
            "converter's --delta-frame and the frozen base's OSC_POSE convention."
        ),
    )

    # ------------------------------------------------------------------ #
    # REQUIRED safety limits -- no silent/unsafe default home or joint range.
    # A zeros home and +/-pi limits (the backend placeholders) are almost
    # certainly wrong for the physical PiperX, so the operator MUST pass real,
    # measured values; _resolve_arm_safety validates them before any hardware
    # is touched.
    # ------------------------------------------------------------------ #
    safety = p.add_argument_group("safety limits (REQUIRED before real hardware)")
    safety.add_argument(
        "--joint-limit-lower",
        nargs=6,
        type=float,
        required=True,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Per-joint lower bound (rad), 6 values. Clamped by JointFollower.",
    )
    safety.add_argument(
        "--joint-limit-upper",
        nargs=6,
        type=float,
        required=True,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Per-joint upper bound (rad), 6 values; must exceed the lower bound.",
    )
    safety.add_argument(
        "--rest-qpos",
        nargs=6,
        type=float,
        required=True,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Home joint vector (rad), 6 values; reset() moves here. Must lie "
        "within [--joint-limit-lower, --joint-limit-upper].",
    )
    safety.add_argument(
        "--tracking-error-rad",
        type=float,
        default=0.2,
        help="JointFollower runaway guard: raise if measured joints drift this "
        "far (rad) from the last command (default 0.2).",
    )
    return p


# Per-step joint deltas above this (rad) are almost certainly a typo rather than
# a safe cap for a real arm, so the server refuses them instead of letting the
# arm lurch toward an IK solution in one tick.
_MAX_SAFE_STEP_RAD = 0.5
_N_JOINTS = 6


def _resolve_arm_safety(args: argparse.Namespace) -> dict:
    """Validate the REQUIRED safety limits and return the arm-ctor kwargs.

    Raises :class:`ValueError` (never silently defaults) when a bound is missing
    or inconsistent, so an unsafe home pose / joint limit can never reach the
    real arm. Pure-Python and hardware-free, so it is unit-tested without a
    robot (the parser's ``required=True`` guarantees the three vectors exist;
    this adds the cross-field range checks argparse cannot express).
    """
    lower = [float(v) for v in args.joint_limit_lower]
    upper = [float(v) for v in args.joint_limit_upper]
    rest = [float(v) for v in args.rest_qpos]
    for name, seq in (
        ("--joint-limit-lower", lower),
        ("--joint-limit-upper", upper),
        ("--rest-qpos", rest),
    ):
        if len(seq) != _N_JOINTS:
            raise ValueError(f"{name} needs {_N_JOINTS} values, got {len(seq)}")
    bad = [i for i, (lo, hi) in enumerate(zip(lower, upper)) if not lo < hi]
    if bad:
        raise ValueError(
            "joint limits require lower < upper for every joint; bad joint "
            f"index(es) {bad}: lower={[lower[i] for i in bad]} "
            f"upper={[upper[i] for i in bad]}"
        )
    out = [
        i for i, (q, lo, hi) in enumerate(zip(rest, lower, upper)) if not lo <= q <= hi
    ]
    if out:
        raise ValueError(
            "--rest-qpos home must lie within the joint limits; out-of-range "
            f"joint index(es) {out}: rest={[rest[i] for i in out]}"
        )
    if not args.max_step_rad > 0:
        raise ValueError(f"--max-step-rad must be > 0, got {args.max_step_rad}")
    if args.max_step_rad > _MAX_SAFE_STEP_RAD:
        raise ValueError(
            f"--max-step-rad={args.max_step_rad} exceeds the safety cap "
            f"{_MAX_SAFE_STEP_RAD} rad; refusing (an uncapped per-step delta can "
            "make the arm lurch). Pass a smaller cap."
        )
    if not args.max_gripper_width > 0:
        raise ValueError(
            f"--gripper-max-width must be > 0, got {args.max_gripper_width}"
        )
    if not args.tracking_error_rad > 0:
        raise ValueError(
            f"--tracking-error-rad must be > 0, got {args.tracking_error_rad}"
        )
    if args.delta_frame != "world":  # defensive; argparse choices already enforce
        raise ValueError(
            f"--delta-frame={args.delta_frame!r} is not implemented; the backend "
            "composes EE-deltas in the base/world frame only."
        )
    return {"lower": lower, "upper": upper, "rest_qpos": rest}


def main(argv=None) -> None:
    """Assemble the real backends and serve forever (runs on machine 41).

    The real backends are imported here, lazily, so the module (and its tests)
    load without the hardware libraries present. A leading ``--dual`` flag (two
    PiperX on ``can0`` / ``can1``) delegates to :func:`main_dual`; without it
    this is the single-arm path, unchanged.
    """
    import sys

    raw = list(sys.argv[1:] if argv is None else argv)
    if "--dual" in raw:
        return main_dual([a for a in raw if a != "--dual"])

    parser = _build_parser()
    args = parser.parse_args(argv)

    # Validate the REQUIRED safety limits BEFORE importing/constructing any
    # hardware, so a bad home pose or joint range fails fast with a clear
    # message instead of ever reaching the arm.
    try:
        safety = _resolve_arm_safety(args)
    except ValueError as exc:
        parser.error(str(exc))  # NoReturn: prints + exits(2)

    bind = args.bind
    if args.port is not None:
        bind = f"tcp://0.0.0.0:{args.port}"

    # Lazy hardware imports -- keep the module top import-light / robot-copyable.
    from .arm_backend import PiperArmBackend
    from .camera_backend import ZmqOrbbecCameraBackend

    print(
        "[robot_server] safety limits: "
        f"rest_qpos={safety['rest_qpos']} "
        f"lower={safety['lower']} upper={safety['upper']} "
        f"max_step_rad={args.max_step_rad} tracking_error_rad={args.tracking_error_rad} "
        f"gripper_max_width={args.max_gripper_width} tcp_frame={args.tcp_frame} "
        f"delta_frame={args.delta_frame}",
        flush=True,
    )

    arm = PiperArmBackend(
        can_channel=args.can_channel,
        bitrate=args.bitrate,
        robot_id=args.robot_id,
        max_gripper_width=args.max_gripper_width,
        gripper_force=args.gripper_force,
        rest_qpos=safety["rest_qpos"],
        joint_limits=(safety["lower"], safety["upper"]),
        max_step_rad=args.max_step_rad,
        tracking_error_rad=args.tracking_error_rad,
        tcp_frame=args.tcp_frame,
    )
    camera = ZmqOrbbecCameraBackend(
        endpoint=args.camera_endpoint,
        serials=args.cameras,
        size=args.camera_size,
        timeout_s=args.camera_timeout_s,
    )

    # Human-in-the-loop reward/done source (P2) -- the operator is at this
    # machine, so the keyboard is read here and shipped back in the obs.
    human_feedback: Optional[HumanFeedbackSource] = None
    if args.reward_done_mode in ("human", "keyboard"):
        human_feedback = KeyboardListenerFeedback()

    server = RobotServer(
        arm, camera, bind=bind, rate_hz=args.rate_hz, human_feedback=human_feedback
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.stop()
    finally:
        # Backends are owned here, not by the server -- release them explicitly.
        arm.close()
        camera.close()
        if human_feedback is not None:
            human_feedback.close()


# --------------------------------------------------------------------------- #
# Dual-arm (two PiperX, 14-D) entry point. The serve loop + RobotServer are
# already backend-agnostic, so only the CLI + backend assembly differ: per-arm
# safety (left / right), two CAN channels, and a DualPiperArmBackend. Reuses the
# single-arm _resolve_arm_safety per arm. main() delegates here on `--dual`.
# --------------------------------------------------------------------------- #
# DUAL_PLACEHOLDER
def _build_dual_parser() -> argparse.ArgumentParser:
    """Parser for the dual (two-arm) server: shared caps + per-arm safety.

    Mirrors :func:`_build_parser` but takes two CAN channels and REQUIRED
    per-arm joint limits / rest pose (``--left-*`` / ``--right-*``). The shared
    control caps (rate, gripper, step, tcp/delta frame, tracking) apply to both
    arms. ``_resolve_dual_arm_safety`` validates each arm via the single-arm
    ``_resolve_arm_safety``.
    """
    p = argparse.ArgumentParser(
        description="Robot-side ZMQ REP server for the DUAL remote Piper bridge.",
    )
    p.add_argument("--bind", default=_DEFAULT_BIND, help=f"(default: {_DEFAULT_BIND}).")
    p.add_argument("--port", type=int, default=None, help="Override just the port.")
    p.add_argument("--rate-hz", type=float, default=10.0, help="step rate cap.")
    p.add_argument(
        "--reward-done-mode",
        default="none",
        choices=["none", "human", "keyboard"],
        help="Human-in-the-loop reward/done source (P2); see single-arm --help.",
    )
    # Cameras (one ZmqOrbbecCameraBackend bundles both wrist cams).
    p.add_argument("--cameras", nargs="+", default=[], metavar="SERIAL")
    p.add_argument("--camera-endpoint", default="tcp://127.0.0.1:5556")
    p.add_argument("--camera-size", type=int, default=128)
    p.add_argument("--camera-timeout-s", type=float, default=1.0)
    # Two CAN channels (left, right).
    p.add_argument(
        "--can-channels",
        nargs=2,
        default=["can0", "can1"],
        metavar=("LEFT", "RIGHT"),
        help="SocketCAN interfaces for the left and right arm (default can0 can1).",
    )
    p.add_argument("--bitrate", type=int, default=1_000_000)
    p.add_argument("--robot-id", default="piper_x")
    p.add_argument(
        "--gripper-max-width",
        "--max-gripper-width",
        dest="max_gripper_width",
        type=float,
        default=0.07,
    )
    p.add_argument("--gripper-force", type=float, default=1.0)
    p.add_argument("--max-step-rad", type=float, default=0.006)
    p.add_argument("--tcp-frame", default="eef", choices=["eef", "flange"])
    p.add_argument("--delta-frame", default="world", choices=["world"])
    p.add_argument("--tracking-error-rad", type=float, default=0.2)
    # REQUIRED per-arm safety (same semantics / validation as the single arm).
    for side in ("left", "right"):
        g = p.add_argument_group(f"{side}-arm safety limits (REQUIRED)")
        g.add_argument(
            f"--{side}-joint-limit-lower",
            dest=f"{side}_joint_limit_lower",
            nargs=6,
            type=float,
            required=True,
            metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        )
        g.add_argument(
            f"--{side}-joint-limit-upper",
            dest=f"{side}_joint_limit_upper",
            nargs=6,
            type=float,
            required=True,
            metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        )
        g.add_argument(
            f"--{side}-rest-qpos",
            dest=f"{side}_rest_qpos",
            nargs=6,
            type=float,
            required=True,
            metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        )
    return p


def _resolve_dual_arm_safety(args: argparse.Namespace) -> list:
    """Validate per-arm safety, reusing the single-arm ``_resolve_arm_safety``.

    Returns ``[left_kwargs, right_kwargs]`` (each ``{lower, upper, rest_qpos}``).
    Raises :class:`ValueError` on any inconsistent arm, before any hardware.
    """
    resolved = []
    for side in ("left", "right"):
        shim = argparse.Namespace(
            joint_limit_lower=getattr(args, f"{side}_joint_limit_lower"),
            joint_limit_upper=getattr(args, f"{side}_joint_limit_upper"),
            rest_qpos=getattr(args, f"{side}_rest_qpos"),
            max_step_rad=args.max_step_rad,
            max_gripper_width=args.max_gripper_width,
            tracking_error_rad=args.tracking_error_rad,
            delta_frame=args.delta_frame,
        )
        try:
            resolved.append(_resolve_arm_safety(shim))
        except ValueError as exc:
            raise ValueError(f"{side} arm: {exc}") from exc
    return resolved


def main_dual(argv=None) -> None:
    """Assemble the DUAL backend (two PiperX) and serve forever (machine 41).

    Reached from :func:`main` on a leading ``--dual`` flag. Validates per-arm
    safety first, then builds a :class:`DualPiperArmBackend` (one
    :class:`PiperArmBackend` per CAN channel) + one camera backend bundling both
    wrist cams, and serves with the same backend-agnostic :class:`RobotServer`.
    """
    parser = _build_dual_parser()
    args = parser.parse_args(argv)

    try:
        safety = _resolve_dual_arm_safety(args)
    except ValueError as exc:
        parser.error(str(exc))  # NoReturn: prints + exits(2)

    bind = args.bind
    if args.port is not None:
        bind = f"tcp://0.0.0.0:{args.port}"

    # Lazy hardware imports -- keep the module top import-light / robot-copyable.
    from .arm_backend import DualPiperArmBackend
    from .camera_backend import ZmqOrbbecCameraBackend

    left, right = safety
    print(
        "[robot_server:dual] per-arm safety: "
        f"left(rest={left['rest_qpos']} lower={left['lower']} upper={left['upper']}) "
        f"right(rest={right['rest_qpos']} lower={right['lower']} upper={right['upper']}) "
        f"can={list(args.can_channels)} max_step_rad={args.max_step_rad} "
        f"tracking_error_rad={args.tracking_error_rad} "
        f"gripper_max_width={args.max_gripper_width} tcp_frame={args.tcp_frame} "
        f"delta_frame={args.delta_frame}",
        flush=True,
    )

    arm = DualPiperArmBackend(
        can_channels=list(args.can_channels),
        bitrate=args.bitrate,
        robot_id=args.robot_id,
        max_gripper_width=args.max_gripper_width,
        gripper_force=args.gripper_force,
        rest_qpos=[left["rest_qpos"], right["rest_qpos"]],
        joint_limits=[
            (left["lower"], left["upper"]),
            (right["lower"], right["upper"]),
        ],
        max_step_rad=args.max_step_rad,
        tracking_error_rad=args.tracking_error_rad,
        tcp_frame=args.tcp_frame,
    )
    camera = ZmqOrbbecCameraBackend(
        endpoint=args.camera_endpoint,
        serials=args.cameras,
        size=args.camera_size,
        timeout_s=args.camera_timeout_s,
    )

    human_feedback: Optional[HumanFeedbackSource] = None
    if args.reward_done_mode in ("human", "keyboard"):
        human_feedback = KeyboardListenerFeedback()

    server = RobotServer(
        arm, camera, bind=bind, rate_hz=args.rate_hz, human_feedback=human_feedback
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.stop()
    finally:
        arm.close()
        camera.close()
        if human_feedback is not None:
            human_feedback.close()


if __name__ == "__main__":
    main()
