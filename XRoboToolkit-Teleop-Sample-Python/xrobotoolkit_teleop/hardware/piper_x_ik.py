"""Bounded pose IK with continuous seeds and explicit forward-kinematics checks."""

from dataclasses import dataclass
import time

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from xrobotoolkit_teleop.simulation.piper_x_model import JOINTS, TCP_LINK


@dataclass
class PoseIKResult:
    joints: np.ndarray
    position_error_m: float
    rotation_error_rad: float
    reached: bool
    iterations: int
    elapsed_ms: float


class ContinuousPoseIK:
    """Solve inside position limits, starting from the last accepted command.

    The small posture residual prefers the current branch. The feedback box is
    a spatial tracking envelope, not a joint velocity limit. FK uses private
    MuJoCo data so candidate evaluation never overwrites measured robot state.
    """

    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        ids = [model.joint(name).id for name in JOINTS]
        self.qadr = model.jnt_qposadr[ids]
        self.vadr = model.jnt_dofadr[ids]
        self.lower, self.upper = model.jnt_range[ids].T.copy()
        self.body = model.body(TCP_LINK).id
        self.jp = np.zeros((3, model.nv))
        self.jr = np.zeros((3, model.nv))
        self.previous = None
        self.rotation_weight = 0.08  # metres per radian; prefer position at limits.
        self.posture_weight = 0.003

    def reset(self, measured):
        self.previous = np.clip(np.asarray(measured, dtype=float), self.lower, self.upper)

    @staticmethod
    def _log_jacobian(vector):
        angle = np.linalg.norm(vector)
        x, y, z = vector
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        factor = 1 / 12 if angle < 1e-4 else (1 - angle / (2 * np.tan(angle / 2))) / angle**2
        return np.eye(3) - 0.5 * cross + factor * (cross @ cross)

    def solve(self, target, measured):
        started = time.monotonic()
        target = np.asarray(target, dtype=float)
        measured = np.asarray(measured, dtype=float)
        if target.shape != (4, 4) or measured.shape != (6,) or not np.all(np.isfinite(target)) or not np.all(np.isfinite(measured)):
            raise ValueError("Pose IK requires a finite transform and six finite joints")
        if self.previous is None:
            self.reset(measured)
        lower = np.maximum(self.lower, measured - 0.55)
        upper = np.minimum(self.upper, measured + 0.55)
        if np.any(lower >= upper):
            raise RuntimeError("Pose IK feedback lies outside the joint tracking envelope")
        seed = np.clip(self.previous, lower + 1e-10, upper - 1e-10)
        cached_q = None
        residual = jacobian = None

        def evaluate(q):
            nonlocal cached_q, residual, jacobian
            if cached_q is not None and np.array_equal(cached_q, q):
                return
            self.data.qpos[self.qadr] = q
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)
            position = self.data.xpos[self.body] - target[:3, 3]
            rotation = Rotation.from_matrix(
                self.data.xmat[self.body].reshape(3, 3) @ target[:3, :3].T
            ).as_rotvec()
            mujoco.mj_jacBody(self.model, self.data, self.jp, self.jr, self.body)
            residual = np.concatenate((position, self.rotation_weight * rotation,
                                       self.posture_weight * (q - seed)))
            jacobian = np.vstack((self.jp[:, self.vadr],
                                  self.rotation_weight * self._log_jacobian(rotation) @ self.jr[:, self.vadr],
                                  self.posture_weight * np.eye(6)))
            cached_q = q.copy()

        def fun(q):
            evaluate(q)
            return residual.copy()

        def jac(q):
            evaluate(q)
            return jacobian.copy()

        initial = fun(seed)
        solution = least_squares(fun, seed, jac=jac, bounds=(lower, upper),
                                 max_nfev=12, ftol=1e-5, xtol=1e-5, gtol=1e-7, x_scale=0.3)
        candidate = solution.x
        final = fun(candidate)
        if not np.all(np.isfinite(candidate)) or np.dot(final, final) > np.dot(initial, initial) + 1e-12:
            candidate, final = seed, initial
        # An unreachable goal remains explicit; never report convergence solely
        # because the optimizer stopped at a joint bound or iteration budget.
        position_error = float(np.linalg.norm(final[:3]))
        rotation_error = float(np.linalg.norm(final[3:6]) / self.rotation_weight)
        self.previous = candidate.copy()
        return PoseIKResult(candidate.copy(), position_error, rotation_error,
                            position_error <= 0.005 and rotation_error <= np.deg2rad(5),
                            solution.nfev, (time.monotonic() - started) * 1000)
