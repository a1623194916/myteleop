# FR3C Hardware Teleoperation Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the delayed-feedback source of low-frequency FR3C motion, select the left PICO controller for `192.168.5.22`, and show the real robot state in a MuJoCo window during hardware teleoperation.

**Architecture:** Keep the Placo IK state continuous by anchoring it to the last ServoJ command instead of repeatedly replacing it with delayed measured joints. Put controller-side resolution and joint trajectory limiting in a dependency-light utility module. Run a passive MuJoCo mirror on the main thread while the IK and ServoJ workers run in the background.

**Tech Stack:** Python 3.10, NumPy, Placo, MuJoCo, Fairino Python SDK, `unittest`, Tyro.

---

### Task 1: Lock Down Control-Side and Trajectory Contracts

**Files:**
- Create: `XRoboToolkit-Teleop-Sample-Python/tests/test_fr3c_control_utils.py`
- Create: `XRoboToolkit-Teleop-Sample-Python/xrobotoolkit_teleop/hardware/fr3c_control_utils.py`

- [ ] **Step 1: Write failing tests for IP mapping and command continuity**

```python
def test_auto_side_uses_left_controller_for_robot_22(self):
    self.assertEqual(resolve_controller_side("192.168.5.22", "auto"), "left")

def test_trajectory_advances_from_previous_command(self):
    trajectory = JointCommandTrajectory(alpha=0.5, max_step_rad=1.0)
    trajectory.reset(np.zeros(1))
    self.assertAlmostEqual(trajectory.advance(np.ones(1))[0], 0.5)
    self.assertAlmostEqual(trajectory.advance(np.ones(1))[0], 0.75)
```

- [ ] **Step 2: Run the tests and confirm imports fail**

Run: `.venv/bin/python -m unittest tests.test_fr3c_control_utils -v`

Expected: `ModuleNotFoundError` for `fr3c_control_utils`.

- [ ] **Step 3: Implement the dependency-light helpers**

```python
ROBOT_IP_CONTROLLER_SIDES = {"192.168.5.22": "left", "192.168.5.23": "right"}

def resolve_controller_side(robot_ip: str, requested_side: str) -> str:
    side = requested_side.lower()
    if side == "auto":
        return ROBOT_IP_CONTROLLER_SIDES.get(robot_ip, "right")
    if side not in {"left", "right"}:
        raise ValueError("controller_side must be one of: auto, left, right")
    return side
```

`JointCommandTrajectory.advance()` must use only its prior command and the current IK target. It must validate `0 < alpha <= 1` and `max_step_rad > 0`, and cap each joint step.

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `.venv/bin/python -m unittest tests.test_fr3c_control_utils -v`

Expected: all tests pass.

### Task 2: Remove Measured-State Re-Anchoring

**Files:**
- Modify: `XRoboToolkit-Teleop-Sample-Python/xrobotoolkit_teleop/hardware/fr3c_teleop_controller.py`
- Modify: `XRoboToolkit-Teleop-Sample-Python/scripts/hardware/teleop_fr3c_hardware.py`

- [ ] **Step 1: Resolve the PICO side at startup**

Add `controller_side: str = "auto"` to both constructors. Resolve it once, then configure `pose_source`, `control_trigger`, and the arm label from the resolved side.

- [ ] **Step 2: Make command state the IK state source**

Replace the per-cycle call to `get_current_joint_positions()` in `calc_target_joint_position()` with a locked copy of the latest command sent by the ServoJ thread.

- [ ] **Step 3: Advance one continuous trajectory in the ServoJ thread**

Initialize `JointCommandTrajectory` from the measured startup position. On every tick, snapshot the latest IK target, advance from the previous command, send that command, and publish it back as the next IK seed.

- [ ] **Step 4: Verify syntax and helper tests**

Run: `.venv/bin/python -m compileall scripts/hardware xrobotoolkit_teleop/hardware`

Expected: compilation succeeds.

### Task 3: Add the Hardware MuJoCo Mirror

**Files:**
- Create: `XRoboToolkit-Teleop-Sample-Python/xrobotoolkit_teleop/hardware/fr3c_mujoco_mirror.py`
- Create: `XRoboToolkit-Teleop-Sample-Python/tests/test_fr3c_mujoco_mirror.py`
- Modify: `XRoboToolkit-Teleop-Sample-Python/scripts/hardware/teleop_fr3c_hardware.py`

- [ ] **Step 1: Test robust joint-name mapping without opening a viewer**

Load `fr3c_assets/scene_fr3c.xml`, call `set_joint_positions()` with six known values, and assert each `j1` through `j6` qpos address contains the corresponding value.

- [ ] **Step 2: Implement a passive real-state mirror**

`Fr3cMujocoMirror` loads the XML once, resolves qpos addresses by joint name, and updates qpos followed by `mujoco.mj_forward()`. Its `run()` method launches `mujoco.viewer.launch_passive`, polls the hardware state at 30 Hz, and stops teleoperation if the viewer closes.

- [ ] **Step 3: Make visualization the hardware default**

Add `visualize_mujoco: bool = True` and `mujoco_xml_path`. Run the mirror on the main thread; Tyro exposes `--no-visualize-mujoco` for headless operation.

- [ ] **Step 4: Run all offline tests**

Run: `.venv/bin/python -m unittest discover -s tests -p 'test_fr3c_*.py' -v`

Expected: all tests pass without connecting to a robot or opening a window.

### Task 4: Documentation and Remote Verification

**Files:**
- Modify: `run.md`

- [ ] **Step 1: Document default hand mapping and MuJoCo behavior**

State that `.22` uses the left controller, `.23` uses the right controller, `--controller-side` overrides the mapping, and `--no-visualize-mujoco` disables the hardware mirror.

- [ ] **Step 2: Run the existing FR3C fake-input simulation**

Run: `PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_mujoco.py --input-source fake --headless-duration 2`

Expected: headless simulation completes without an exception.

- [ ] **Step 3: Inspect the final diff and worktree**

Run: `git diff --check` and `git status --short`

Expected: no whitespace errors; only planned files are modified or added.
