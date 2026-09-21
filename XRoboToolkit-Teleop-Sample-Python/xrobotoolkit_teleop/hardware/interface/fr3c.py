"""FR3C (Fairino) robot interface via the vendor Python SDK.

Mirrors the official UR interface (universal_robots.py): reset to an initial
joint pose, then stream joint targets with ServoJ from a dedicated servo
thread.  All SDK calls take DEGREES; this interface converts from radians at
the boundary, same as the rest of the framework.

Servo session lifecycle (required by the controller):
    start_servo()          -> ServoMoveStart
    servo_joints(q)  x N   -> ServoJ on absolute cmd_t deadlines (metronomic)
    stop_servo()           -> ServoMoveEnd

Error handling: the controller rejects every motion command with error 14
("接口执行失败" / interface execution failed) while ANY robot-level fault is
latched — e.g. servo drive fault 8-1 ("Runaway fault", joint position control
lost; see the manual's Appendix 3), which latches once at servo-session start
and clears cleanly with ResetAllError. That is NOT a speed problem: slowing
the stream cannot fix it, so on 14 the interface polls GetRobotErrorCode,
auto-clears resettable faults with ResetAllError and keeps streaming; a fault
that survives repeated clears stops the session fast. Genuine per-step
overspeed (no fault latched) still auto-scales cmdT down; a burst of
consecutive errors flags the stream dead so the caller can stop.
"""
import sys
import threading
import time
import importlib.util
from pathlib import Path

import numpy as np

from xrobotoolkit_teleop.hardware.fr3c_control_utils import AbsoluteDeadlinePacer


class _ThreadSafeServerProxy:
    """Serialize every XML-RPC call on one robot's connection.

    The Fairino SDK stores a single ``xmlrpc.client.ServerProxy`` per robot
    whose transport reuses one ``http.client.HTTPConnection`` across calls.
    When the servo stream and the forward-state loop issue requests
    concurrently, the underlying connection state machine
    interleaves and raises ``http.client.CannotSendRequest('Request-sent')``
    (surfaced as ``Arm servo thread failed: Request-sent``), killing the servo
    thread. Wrapping the proxy funnels the arm/state SDK traffic through one
    RLock. UDP ServoJ bypasses XML-RPC entirely, so gripper calls can use the
    vendor SDK connection without interrupting the realtime stream.
    """
    def __init__(self, proxy: object):
        self._proxy = proxy
        self._lock = threading.RLock()

    def __getattr__(self, name: str):
        target = getattr(self._proxy, name)
        if not callable(target):
            return target
        lock = self._lock

        def locked(*args, **kwargs):
            with lock:
                return target(*args, **kwargs)

        return locked


DEFAULT_ROBOT_IP = "192.168.58.2"
SERVO_CMD_T = 0.01  # ServoJ command period (s); Fairino recommends 0.008-0.016
SERVO_TRANSPORTS = {"xmlrpc": 0, "udp": 1}
RESET_VELOCITY = 20.0  # MoveJ velocity percent for reset moves
RESET_ARRIVAL_TOL_DEG = 1.0
RESET_TIMEOUT = 30.0
MAX_CONSECUTIVE_SERVO_ERRORS = 50
SERVO_SLOWDOWN_FACTOR = 1.5  # cmdT multiplier when the controller reports speed over limit (err 14)
# Do not let the err-14 slowdown escalate without bound. Each 1.5x growth
# makes the per-command step larger, which can keep the controller reporting
# overspeed forever (cmdT realistically ballooned 10ms -> 6.5s). Cap it inside
# the valid servo command period and let a persistent condition reach the
# consecutive-error limit so the session stops instead of wedging.
SERVO_MAX_CMD_T = 0.03

# ServoJ error 14 = "接口执行失败" (interface execution failed). While any
# robot-level fault is latched, the controller rejects ServoJ, ServoMoveEnd,
# ActGripper etc. with this code regardless of the commanded motion. It is a
# fault-state rejection, NOT joint overspeed, so the recovery is to detect and
# clear the latched fault, not to slow the stream down.
SERVO_ERR_INTERFACE_REJECTED = 14
# How often (s) a rejected stream polls GetRobotErrorCode / retries
# ResetAllError, and how many failed recovery rounds stop the session.
SERVO_FAULT_POLL_INTERVAL_S = 0.5
MAX_FAULT_RECOVERY_ATTEMPTS = 10

# FR3C firmware is 3.9.9: only the matching official SDK works (CNDE + XML-RPC).
FAIRINO_SDK_PATHS = [
    "/home/u22/kyz/pico_software/fair_ws/fairino-python-sdk-v2.2.9_robot3.9.9/linux/fairino",
]


def _load_robot_module(namespace: str | None = None):
    # The Fairino SDK keeps CNDE sockets, reconnect flags, and connection
    # metadata as class-level globals on ``Robot.RPC``.  Two robots imported
    # through the normal module name therefore overwrite each other's state.
    # Load a private module copy per arm in dual teleop so each RPC class owns
    # an independent set of globals.  ``namespace=None`` preserves the normal
    # import path for diagnostics and existing integrations.
    if namespace:
        safe_name = "".join(ch if ch.isalnum() else "_" for ch in namespace)
        module_name = f"_fairino_robot_{safe_name}"
        cached = sys.modules.get(module_name)
        if cached is not None:
            return cached
        for path in FAIRINO_SDK_PATHS:
            robot_py = Path(path) / "Robot.py"
            if not robot_py.exists():
                continue
            spec = importlib.util.spec_from_file_location(module_name, robot_py)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
            return module
    try:
        import Robot  # noqa: F401  (already on path, e.g. compiled Robot.so)
        return Robot
    except ImportError:
        pass
    for path in FAIRINO_SDK_PATHS:
        if Path(path).exists():
            sys.path.insert(0, path)
            try:
                import Robot
                return Robot
            except ImportError:
                sys.path.remove(path)
    raise ImportError(
        "Fairino SDK module 'Robot' not found; checked: " + ", ".join(FAIRINO_SDK_PATHS)
    )


class Fr3cController:
    def __init__(
        self,
        robot_ip: str = DEFAULT_ROBOT_IP,
        tool: int = 1,
        user: int = 0,
        cmd_t: float = SERVO_CMD_T,
        sdk_namespace: str | None = None,
        servo_transport: str = "udp",
    ):
        robot_module = _load_robot_module(namespace=sdk_namespace)
        print(f"Connecting to FR3C at {robot_ip} ...")
        self._sdk = robot_module
        self.robot = robot_module.RPC(robot_ip)
        # Newer SDK (v2.2.x) exposes the connection flag as RPC.is_connect
        # (older bindings used RPC.is_connect).
        if not robot_module.RPC.is_connect:
            raise ConnectionError(f"FR3C XML-RPC connection failed ({robot_ip})")
        self.robot_ip = robot_ip
        self.tool = tool
        self.user = user
        transport = str(servo_transport).strip().lower()
        if transport not in SERVO_TRANSPORTS:
            raise ValueError(
                f"unsupported ServoJ transport {servo_transport!r}; "
                "expected 'udp' or 'xmlrpc'"
            )
        self.servo_transport = transport
        self._servo_cmd_type = SERVO_TRANSPORTS[transport]
        print(
            f"ServoJ transport: {transport} "
            f"({'controller-confirmed' if transport == 'xmlrpc' else 'send-only'})"
        )
        # The Fairino XML-RPC client reuses one underlying HTTP connection and
        # is NOT thread safe: concurrent requests from the servo stream, the
        # end-effector thread, and shutdown interleave the connection state
        # machine (http.client.CannotSendRequest / ResponseNotReady).
        # Servo and state RPC calls below therefore run under this lock, and
        # the raw connection is wrapped so state readers cannot interleave.
        self._rpc_lock = threading.Lock()
        # Serialize activation and motion calls on the SDK's XML-RPC channel.
        self._gripper_rpc_lock = threading.Lock()
        proxy = getattr(self.robot, "robot", None)
        if proxy is not None:
            try:
                self.robot.robot = _ThreadSafeServerProxy(proxy)
            except (AttributeError, TypeError):
                pass  # mock SDk used in offline tests
        self._cmd_t = cmd_t
        self._pacer = AbsoluteDeadlinePacer(cmd_t)
        self._last_send_late_s = 0.0
        self._last_servo_rpc_ms = 0.0
        self._last_servo_error = 0
        self._prefer_realtime_fault = True
        self._servo_active = False
        self._servo_ready = threading.Event()
        # The CNDE gripper_active bit is unreliable on the TG-9801 path, so
        # remember activation locally. If ResetAllError clears a gripper while
        # recovering a ServoJ fault, the gripper worker can restore it without
        # running a blocking ActGripper call on the servo thread.
        self._gripper_expected_active = False
        self._gripper_needs_activation = False
        # A gripper 485 timeout is reported in the CNDE package even when
        # ServoJ itself still returns success.  Let the gripper worker ask the
        # servo thread to clear it at a serialized, realtime-safe point.
        self._gripper_recovery_requested = threading.Event()
        self._consecutive_errors = 0
        self._last_fault_check_s = 0.0
        self._fault_recovery_attempts = 0
        self._wait_state_ready()
        print(f"Connected to FR3C at {robot_ip}")

    def _servo_move_start(self):
        if getattr(self, "_servo_cmd_type", 0) == 1:
            return self.robot.ServoMoveStart(cmdType=1)
        return self.robot.ServoMoveStart()

    def _servo_move_end(self):
        if getattr(self, "_servo_cmd_type", 0) == 1:
            return self.robot.ServoMoveEnd(cmdType=1)
        return self.robot.ServoMoveEnd()

    def _servo_j(self, joint_deg):
        kwargs = {"cmdT": self._cmd_t}
        if getattr(self, "_servo_cmd_type", 0) == 1:
            kwargs["cmdType"] = 1
        return self.robot.ServoJ(joint_deg, [0.0, 0.0, 0.0, 0.0], **kwargs)

    def _wait_state_ready(self, timeout: float = 10.0):
        """The SDK exposes joint state only after its realtime thread (port
        20004, 30 ms connect timeout, no retry) delivers the first packet;
        wait for it and re-kick the socket if needed."""
        if not hasattr(self.robot, "robot_state_pkg"):
            return  # mock SDK (offline tests)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pkg = self.robot.robot_state_pkg
            if pkg is not None and not isinstance(pkg, type):
                return
            if not getattr(self.robot, "sock_cli_state_state", True):
                try:
                    self.robot.connect_to_robot()
                except Exception:
                    pass
            time.sleep(0.1)
        raise ConnectionError(
            f"FR3C realtime state not received from {self.robot_ip}; check the network path"
        )

    def get_current_joint_positions(self) -> np.ndarray:
        """Current joint positions in RADIANS (read from the realtime state
        package, no RPC round trip)."""
        _, jpos_deg = self.robot.GetActualJointPosDegree()
        return np.deg2rad(np.asarray(jpos_deg, dtype=float))

    def get_realtime_joint_positions(self) -> np.ndarray | None:
        """Read CNDE joint state without an XML-RPC round trip."""
        try:
            pkg = self.robot.robot_state_pkg
            values = getattr(pkg, "jt_cur_pos", None)
            if values is None or len(values) < 6:
                return None
            return np.deg2rad(np.asarray(values[:6], dtype=float))
        except Exception:
            return None

    def reset(self, initial_joint_positions: np.ndarray):
        """MoveJ (blocking) to the given joint pose in RADIANS."""
        target_deg = np.rad2deg(np.asarray(initial_joint_positions, dtype=float)).tolist()
        err = self.robot.MoveJ(
            target_deg, self.tool, self.user, vel=RESET_VELOCITY, ovl=100.0, blendT=-1.0
        )
        if err != 0:
            raise RuntimeError(f"MoveJ to initial pose failed, error code {err}")

        t0 = time.monotonic()
        while time.monotonic() - t0 < RESET_TIMEOUT:
            cur_deg = np.rad2deg(self.get_current_joint_positions())
            if np.max(np.abs(np.asarray(cur_deg) - target_deg)) < RESET_ARRIVAL_TOL_DEG:
                print("Reached initial position.")
                return
            time.sleep(0.1)
        raise TimeoutError("FR3C did not reach the initial pose in time")

    def activate_gripper(
        self,
        index: int = 1,
        reset_delay: float = 1.0,
        activation_delay: float = 2.0,
    ):
        """Reset and activate an already configured Fairino gripper."""
        if index <= 0:
            raise ValueError("gripper index must be positive")
        if reset_delay < 0.0 or activation_delay < 0.0:
            raise ValueError("gripper activation delays must be non-negative")

        with self._gripper_rpc_lock:
            with self._rpc_lock:
                err = self.robot.ActGripper(index, 0)
            if err != 0:
                raise RuntimeError(f"ActGripper reset failed, error code {err}")
        time.sleep(reset_delay)
        with self._gripper_rpc_lock:
            with self._rpc_lock:
                err = self.robot.ActGripper(index, 1)
            if err != 0:
                raise RuntimeError(f"ActGripper activation failed, error code {err}")
        time.sleep(activation_delay)
        self._gripper_expected_active = True
        self._gripper_needs_activation = False

    def clear_gripper_warning(
        self,
        index: int = 1,
        error_settle_delay: float = 0.3,
        reset_delay: float = 0.5,
    ) -> None:
        """Clear a stale gripper-motion warning before ServoMoveStart.

        ``gripper_fault`` is independent of the robot main/sub error code, so
        the normal servo preflight may not see a tool-485 motion timeout.  The
        controller alarm is cleared first, then ``ActGripper(reset)`` cancels
        any pending gripper motion and clears the gripper's own state machine.
        Activation is deliberately left to the post-ServoMoveStart barrier.
        """
        if index <= 0:
            raise ValueError("gripper index must be positive")
        if error_settle_delay < 0.0 or reset_delay < 0.0:
            raise ValueError("gripper warning-clear delays must be non-negative")

        self.reset_all_errors()
        time.sleep(error_settle_delay)
        with self._gripper_rpc_lock:
            with self._rpc_lock:
                err = self.robot.ActGripper(index, 0)
        if err != 0:
            raise RuntimeError(f"ActGripper warning reset failed, error code {err}")
        time.sleep(reset_delay)
        self._gripper_expected_active = False
        self._gripper_needs_activation = False

    def move_gripper(
        self,
        position_percent: float,
        index: int = 1,
        velocity: int = 20,
        force: int = 20,
        max_time_ms: int = 1000,
        lock_timeout_s: float | None = None,
    ) -> int:
        """Send one nonblocking target to a configured parallel gripper.

        ``lock_timeout_s=0`` is used by the teleop follower. Gripper calls use
        the vendor SDK's XML-RPC connection; UDP ServoJ remains independent.
        """
        if not 0.0 <= position_percent <= 100.0:
            raise ValueError("position_percent must be in [0, 100]")
        for name, value in (("velocity", velocity), ("force", force)):
            if not 0 <= value <= 100:
                raise ValueError(f"{name} must be in [0, 100]")
        if not 0 <= max_time_ms <= 30000:
            raise ValueError("max_time_ms must be in [0, 30000]")

        acquired = (
            self._gripper_rpc_lock.acquire()
            if lock_timeout_s is None
            else self._gripper_rpc_lock.acquire(timeout=max(0.0, float(lock_timeout_s)))
        )
        if not acquired:
            return 1
        try:
            with self._rpc_lock:
                err = self.robot.MoveGripper(
                    int(index), int(round(position_percent)), int(velocity), int(force),
                    int(max_time_ms), 1, 0, 0.0, 0, 0,
                )
        finally:
            self._gripper_rpc_lock.release()
        if err != 0:
            raise RuntimeError(f"MoveGripper failed, error code {err}")
        return err

    def get_gripper_motion_done(self, index: int = 1) -> bool:
        """True when the gripper finished its current move (SDK >= v2.2.x).

        The new firmware reports gripper motion completion both via
        ``GetGripperMotionDone`` and in the realtime state package. Letting a
        teleop loop wait on this before issuing the next target avoids sending
        a replacement target into an in-progress move.
        """
        try:
            with self._rpc_lock:
                ret = self.robot.GetGripperMotionDone()
            err = int(ret[0])
            if err != 0:
                return True  # can't determine -> don't block teleop on that
            fault, status = int(ret[1]), int(ret[2])
            return status == 1 and fault == 0
        except Exception:
            return True  # safe default: never stall the send chain

    def gripper_fault_code(self) -> int:
        """Return the CNDE gripper fault (0 healthy, 1 commonly 485 timeout).

        The Fairino SDK exposes this separately from ``main_code``/``sub_code``.
        Returning ``-1`` means that no realtime package is available yet.
        """
        try:
            pkg = self.robot.robot_state_pkg
            if pkg is None:
                return -1
            return int(getattr(pkg, "gripper_fault", 0))
        except (TypeError, ValueError, AttributeError):
            return -1

    def request_gripper_recovery(self) -> None:
        """Ask the realtime servo path to clear a gripper-only fault.

        ResetAllError shares the robot RPC connection with ServoJ.  The
        gripper worker therefore only sets this flag while the servo thread
        performs the actual clear between two serialized commands.
        """
        self._gripper_recovery_requested.set()

    def _service_gripper_recovery(self) -> bool:
        request = getattr(self, "_gripper_recovery_requested", None)
        if request is None or not request.is_set() or not self._servo_active:
            return False
        try:
            self.reset_all_errors()
        except Exception as e:
            print(f"ResetAllError for gripper fault failed: {e}")
            return False
        request.clear()
        print("Gripper fault cleared; waiting for gripper re-activation.")
        return True

    def get_robot_error_code(self) -> tuple[int, int]:
        """Latest latched robot-level error as (main, sub); (0, 0) = healthy."""
        with self._rpc_lock:
            err, codes = self.robot.GetRobotErrorCode()
        if err != 0:
            raise RuntimeError(f"GetRobotErrorCode failed, error code {err}")
        return int(codes[0]), int(codes[1])

    def ensure_fault_free(self, max_attempts: int = 3) -> bool:
        """Clear resettable latched faults before starting a servo session.

        Returns True when the robot reports no fault (or the query itself
        fails, which must not block startup). Retries ResetAllError a few
        times because a fault can re-latch between the clear and the recheck.
        Gripper re-activation after a clear is the gripper controller's job
        (single ownership — this path never touches the gripper).
        """
        for _ in range(max_attempts):
            try:
                main_code, sub_code = self.get_robot_error_code()
            except Exception as e:
                print(f"GetRobotErrorCode during preflight failed: {e}")
                return True
            if main_code == 0:
                return True
            print(f"Robot fault {main_code}-{sub_code} latched; ResetAllError...")
            try:
                self.reset_all_errors()
            except Exception as e:
                print(f"ResetAllError during preflight failed: {e}")
            time.sleep(0.3)
        try:
            main_code, sub_code = self.get_robot_error_code()
        except Exception as e:
            print(f"GetRobotErrorCode during preflight failed: {e}")
            return True
        if main_code != 0:
            print(
                f"Robot fault {main_code}-{sub_code} persists after {max_attempts} "
                "clear attempts; check the web pendant (fault may not be resettable)."
            )
            return False
        return True

    def reset_all_errors(self):
        """Clear all resettable robot-level errors (e.g. latched 8-1).

        Servo drive fault 8-1 ("Runaway fault", joint position control lost)
        latches once at servo-session start and is cleared cleanly here.
        NOTE: ResetAllError also DEACTIVATES the configured gripper; the
        gripper controller re-activates it on its own thread (single
        ownership: the servo path never touches the gripper).
        """
        with self._rpc_lock:
            err = self.robot.ResetAllError()
        if err != 0:
            raise RuntimeError(f"ResetAllError failed, error code {err}")
        if getattr(self, "_gripper_expected_active", False):
            self._gripper_needs_activation = True

    @property
    def gripper_needs_activation(self) -> bool:
        """Whether a previous fault clear invalidated gripper activation."""
        return bool(getattr(self, "_gripper_needs_activation", False))

    def gripper_active(self, index: int = 1) -> bool:
        """True when the realtime state reports the gripper active.

        Unknown state (no pkg yet) returns False so callers re-activate.
        NOTE: on the current rig (TG-9801 via tool-board 485) this pkg field
        always reads 0 — do NOT use it to decide whether the gripper works.
        """
        try:
            pkg = self.robot.robot_state_pkg
            return pkg is not None and int(pkg.gripper_active) == 1
        except Exception:
            return False

    def latched_fault_code(self) -> tuple[int, int]:
        """(main, sub) robot fault from the realtime pkg — NO RPC round trip.

        Reads the CNDE state that the SDK thread refreshes at 8 ms, so the
        gripper control loop can poll it at full rate. (-1, -1) when the pkg
        is missing (unknown).
        """
        try:
            pkg = self.robot.robot_state_pkg
            if pkg is None:
                return (-1, -1)
            return (int(pkg.main_code), int(pkg.sub_code))
        except Exception:
            return (-1, -1)

    def recover_gripper(
        self,
        index: int = 1,
        reset_delay: float = 1.0,
        activation_delay: float = 2.0,
    ):
        """Full gripper recovery: clear latched errors, then reset+activate."""
        self.reset_all_errors()
        self.activate_gripper(
            index=index, reset_delay=reset_delay, activation_delay=activation_delay
        )

    def set_tool_do(self, index: int, status: bool, smooth: int = 0, block: int = 1) -> int:
        """Set a tool-side digital output (0 = off, 1 = on).

        Used for binary end effectors such as a vacuum suction cup. Nonblocking
        (``block=1``) by default so an edge-driven control loop never stalls on
        the XML-RPC round trip.
        """
        if not 0 <= int(index) <= 1:
            raise ValueError("tool DO index must be in [0, 1]")
        with self._rpc_lock:
            err = self.robot.SetToolDO(int(index), 1 if status else 0, int(smooth), int(block))
        if err != 0:
            raise RuntimeError(f"SetToolDO failed, error code {err}")
        return err

    def start_servo(self):
        if self._servo_active:
            return
        if not self.ensure_fault_free():
            raise RuntimeError(
                "ServoMoveStart refused: robot fault latched and not clearable"
            )
        self._servo_ready.clear()
        if getattr(self, "_servo_cmd_type", 0) == 1:
            err = self._servo_move_start()
        else:
            with self._rpc_lock:
                err = self._servo_move_start()
        if err != 0:
            # A fault latched between the preflight and the start call also
            # rejects ServoMoveStart with error 14; clear once and retry.
            if self.ensure_fault_free():
                if getattr(self, "_servo_cmd_type", 0) == 1:
                    err = self._servo_move_start()
                else:
                    with self._rpc_lock:
                        err = self._servo_move_start()
        if err != 0:
            raise RuntimeError(f"ServoMoveStart failed, error code {err}")
        self._servo_active = True
        print("Servo session started.")

    def servo_joints(self, joint_positions: np.ndarray):
        """Stream one joint target (RADIANS) on an absolute cmd_t schedule.

        Pacing is metronomic (AbsoluteDeadlinePacer): the send always targets
        the next cmd_t grid point, RPC latency eats the slack instead of
        delaying the following point, and overruns catch up immediately. The
        last tick's lateness is exposed via ``last_send_late_s`` for
        diagnostics."""
        # Handle a tool-side 485 timeout even when the controller continues to
        # accept ServoJ packets.  This keeps the arm stream alive and lets the
        # gripper worker re-activate the tool through the SDK RPC channel.
        if self._service_gripper_recovery():
            # ResetAllError needs a short settling interval and also clears
            # the tool activation.  Skip this one ServoJ tick; the gripper
            # worker will re-activate the tool while UDP ServoJ stays independent.
            self._pacer.tick()
            return
        late_s = self._pacer.tick()
        self._last_send_late_s = late_s
        jpos_deg = np.rad2deg(np.asarray(joint_positions, dtype=float)).tolist()
        send_start_s = time.monotonic()
        try:
            if getattr(self, "_servo_cmd_type", 0) == 1:
                err = self._servo_j(jpos_deg)
            else:
                with self._rpc_lock:
                    err = self._servo_j(jpos_deg)
        finally:
            self._last_servo_rpc_ms = (time.monotonic() - send_start_s) * 1000.0
        self._last_servo_error = int(err)
        self._pacer.anchor_after_send(send_start_s)
        if err == 0:
            self._servo_ready.set()
            self._consecutive_errors = 0
            self._fault_recovery_attempts = 0
            return
        self._consecutive_errors += 1
        now = time.monotonic()
        if err == SERVO_ERR_INTERFACE_REJECTED and now - self._last_fault_check_s >= SERVO_FAULT_POLL_INTERVAL_S:
            # Error 14 = the controller rejects the interface while a robot
            # fault is latched; slowing the stream cannot fix that. Detect,
            # auto-clear and resume; only genuine overspeed (no fault) is
            # treated with the cmdT slowdown below.
            self._last_fault_check_s = now
            # CNDE is refreshed independently at 8 ms. Never add another
            # blocking XML-RPC round trip to the ServoJ error path: that was
            # observed as multi-second holes in the command stream.
            fault = self.latched_fault_code()
            if fault == (-1, -1):
                # Keep lightweight fake SDKs and older integrations working;
                # live controllers use CNDE only and never make this fallback
                # on the ServoJ thread.
                if not getattr(self, "_prefer_realtime_fault", False):
                    try:
                        fault = self.get_robot_error_code()
                    except Exception:
                        fault = (0, 0)
                else:
                    fault = (0, 0)
            if fault[0] != 0:
                # Death in the fault path is governed by the recovery-attempt
                # budget below; the consecutive counter would hit its limit
                # within a single poll interval (50 ticks at 10 ms = 0.5 s)
                # long before any recovery round completes.
                self._consecutive_errors = 0
                self._fault_recovery_attempts += 1
                if self._fault_recovery_attempts > MAX_FAULT_RECOVERY_ATTEMPTS:
                    print(
                        f"ServoJ rejected (14): robot fault {fault[0]}-{fault[1]} "
                        f"could not be cleared after {MAX_FAULT_RECOVERY_ATTEMPTS} "
                        "attempts. Stopping the stream."
                    )
                    self._consecutive_errors = max(
                        self._consecutive_errors, MAX_CONSECUTIVE_SERVO_ERRORS
                    )
                    return
                print(
                    f"ServoJ rejected (14): robot fault {fault[0]}-{fault[1]} "
                    f"latched; ResetAllError "
                    f"({self._fault_recovery_attempts}/{MAX_FAULT_RECOVERY_ATTEMPTS})..."
                )
                try:
                    self.reset_all_errors()
                except Exception as e:
                    print(f"ResetAllError during servo recovery failed: {e}")
                return
            # No fault latched: err 14 here means the per-step target jump is
            # too large (genuine overspeed). Slow the stream down.
            self._cmd_t = min(self._cmd_t * SERVO_SLOWDOWN_FACTOR, SERVO_MAX_CMD_T)
            self._pacer.period_s = self._cmd_t
            print(
                f"ServoJ speed over limit (err 14, no fault); cmd_t raised to "
                f"{self._cmd_t * 1000:.1f} ms ({self._consecutive_errors} consecutive)"
            )
        elif err != 0 and self._consecutive_errors <= 5:
            print(f"ServoJ error code {err} ({self._consecutive_errors} consecutive)")

    @property
    def last_send_late_s(self) -> float:
        """How late the previous ServoJ send woke past its deadline (s)."""
        return self._last_send_late_s

    @property
    def servo_active(self) -> bool:
        """Whether the ServoJ session currently owns motion RPCs."""
        return bool(self._servo_active)

    @property
    def servo_ready(self) -> bool:
        """Whether at least one ServoJ point was submitted successfully."""
        ready = getattr(self, "_servo_ready", None)
        return bool(ready is not None and ready.is_set())

    @property
    def last_servo_rpc_ms(self) -> float:
        """Duration of the previous ServoJ RPC, for diagnostics only."""
        return float(getattr(self, "_last_servo_rpc_ms", 0.0))

    @property
    def last_servo_error(self) -> int:
        """Return code from the previous ServoJ call, for diagnostics only."""
        return int(getattr(self, "_last_servo_error", 0))

    @property
    def servo_stream_dead(self) -> bool:
        return self._consecutive_errors >= MAX_CONSECUTIVE_SERVO_ERRORS

    @property
    def realtime_fault_code(self) -> tuple[int, int]:
        """Fault code from the latest CNDE frame without an RPC call."""
        return self.latched_fault_code()

    @property
    def realtime_collision_state(self) -> int:
        pkg = getattr(self.robot, "robot_state_pkg", None)
        try:
            return int(getattr(pkg, "collision_state", getattr(pkg, "collisionState", 0)))
        except (TypeError, ValueError):
            return 0

    @property
    def realtime_collision_level(self):
        pkg = getattr(self.robot, "robot_state_pkg", None)
        value = getattr(pkg, "collision_level", getattr(pkg, "collisionLevel", None))
        try:
            return [int(v) for v in value[:6]] if value is not None else None
        except (TypeError, ValueError, IndexError):
            return None

    def stop_servo(self):
        if not self._servo_active:
            return
        self._servo_active = False
        if getattr(self, "_servo_cmd_type", 0) == 1:
            err = self._servo_move_end()
        else:
            with self._rpc_lock:
                err = self._servo_move_end()
        if err != 0:
            # A latched fault makes the controller reject ServoMoveEnd with
            # error 14 as well; clear and retry once so the session ends
            # cleanly instead of leaving the controller in servo mode. The
            # session is over, so gripper re-activation is irrelevant here.
            if self.ensure_fault_free():
                if getattr(self, "_servo_cmd_type", 0) == 1:
                    err = self._servo_move_end()
                else:
                    with self._rpc_lock:
                        err = self._servo_move_end()
        if err != 0:
            print(f"ServoMoveEnd failed, error code {err}")
        else:
            print("Servo session ended.")

    def close(self):
        try:
            # Disable the SDK's auto-reconnect before closing: a recv error
            # during the session spawns a reconnect thread that ignores the
            # stop flag and would re-register CNDE after CloseRPC, keeping the
            # controller's single-client slot occupied for every later client.
            rpc_cls = type(self.robot)
            if hasattr(rpc_cls, "_reconnect_enable"):
                rpc_cls._reconnect_enable = False
            self.stop_servo()
        except Exception as e:
            print(f"stop_servo raised during close: {e}")
        try:
            self.robot.CloseRPC()
        except Exception as e:
            print(f"CloseRPC raised: {e}")
        print(f"FR3C connection closed ({self.robot_ip}).")
