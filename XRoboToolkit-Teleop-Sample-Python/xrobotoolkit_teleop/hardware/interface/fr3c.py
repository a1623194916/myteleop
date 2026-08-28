"""FR3C (Fairino) robot interface via the vendor Python SDK.

Mirrors the official UR interface (universal_robots.py): reset to an initial
joint pose, then stream joint targets with ServoJ from a dedicated servo
thread.  All SDK calls take DEGREES; this interface converts from radians at
the boundary, same as the rest of the framework.

Servo session lifecycle (required by the controller):
    start_servo()          -> ServoMoveStart
    servo_joints(q)  x N   -> ServoJ at ~cmdT pacing (blocks ~cmdT per call)
    stop_servo()           -> ServoMoveEnd

Error handling ported from field-proven practice (fairmove.py):
error code 14 (speed over limit) auto-scales the command period down;
a burst of consecutive errors flags the stream dead so the caller can stop.
"""
import sys
import time
from pathlib import Path

import numpy as np

DEFAULT_ROBOT_IP = "192.168.58.2"
SERVO_CMD_T = 0.01  # ServoJ command period (s); Fairino recommends 0.008-0.016
RESET_VELOCITY = 20.0  # MoveJ velocity percent for reset moves
RESET_ARRIVAL_TOL_DEG = 1.0
RESET_TIMEOUT = 30.0
MAX_CONSECUTIVE_SERVO_ERRORS = 50
SERVO_SLOWDOWN_FACTOR = 1.5  # cmdT multiplier when the controller reports speed over limit (err 14)

FAIRINO_SDK_PATHS = [
    "/home/u22/kyz/pico_software/fair_ws/fairino-python-sdk-v2.1.3.1_robot3.8.3/mine",
    "/home/u22/kyz/pico_software/fair_ws/fairino-python-sdk-v2.1.3.1_robot3.8.3/linux/fairino",
]


def _load_robot_module():
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
    ):
        robot_module = _load_robot_module()
        print(f"Connecting to FR3C at {robot_ip} ...")
        self._sdk = robot_module
        self.robot = robot_module.RPC(robot_ip)
        if not robot_module.RPC.is_conect:
            raise ConnectionError(f"FR3C XML-RPC connection failed ({robot_ip})")
        self.robot_ip = robot_ip
        self.tool = tool
        self.user = user
        self._cmd_t = cmd_t
        self._servo_active = False
        self._consecutive_errors = 0
        self._wait_state_ready()
        print(f"Connected to FR3C at {robot_ip}")

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

    def start_servo(self):
        if self._servo_active:
            return
        err = self.robot.ServoMoveStart()
        if err != 0:
            raise RuntimeError(f"ServoMoveStart failed, error code {err}")
        self._servo_active = True
        print("Servo session started.")

    def servo_joints(self, joint_positions: np.ndarray):
        """Stream one joint target (RADIANS).  Blocks ~cmd_t per call so a
        tight caller loop issues commands at the commanded rate, like the UR
        interface's initPeriod/waitPeriod pacing."""
        t0 = time.monotonic()
        jpos_deg = np.rad2deg(np.asarray(joint_positions, dtype=float)).tolist()
        err = self.robot.ServoJ(jpos_deg, [0.0, 0.0, 0.0, 0.0], cmdT=self._cmd_t)
        if err == 14:
            self._cmd_t *= SERVO_SLOWDOWN_FACTOR
            print(f"ServoJ speed over limit; slowed cmd_t to {self._cmd_t * 1000:.1f} ms")
            self._consecutive_errors = 0
        elif err != 0:
            self._consecutive_errors += 1
            if self._consecutive_errors <= 5:
                print(f"ServoJ error code {err} ({self._consecutive_errors} consecutive)")
        else:
            self._consecutive_errors = 0

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, self._cmd_t - elapsed))

    @property
    def servo_stream_dead(self) -> bool:
        return self._consecutive_errors >= MAX_CONSECUTIVE_SERVO_ERRORS

    def stop_servo(self):
        if not self._servo_active:
            return
        err = self.robot.ServoMoveEnd()
        self._servo_active = False
        if err != 0:
            print(f"ServoMoveEnd failed, error code {err}")
        else:
            print("Servo session ended.")

    def close(self):
        self.stop_servo()
        try:
            self.robot.CloseRPC()
        except Exception as e:
            print(f"CloseRPC raised: {e}")
        print(f"FR3C connection closed ({self.robot_ip}).")
