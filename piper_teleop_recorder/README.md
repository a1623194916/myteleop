# PiperX Teleop Data Recorder

Teleoperate one or two **PiperX** arms with VR controllers and record **30 Hz**
episodes (Orbbec images 640×480 + arm state) with **per-frame subtask labels**
into HDF5, convertible to LeRobot. It reuses the existing picosoftware
teleop stack on robot machine 41 and is connected to the guarded Piper-X
entrypoint and local USB camera service.

## Current machine: local Gemini 305, A-key recording, no VR video

For normal use on machine 41, start PC Service, connect PICO and bring CAN up,
then run from the repository root:

```bash
bash start_piper_x.sh --mode dual --enable-hardware --left-channel can0 --right-channel can1
# Or one arm only:
bash start_piper_x.sh --mode single --controller-side right --channel can1 --enable-hardware
```

This launches the local 305 JPEG service and enables A-key recording. Headset
video is **off by default**; `--video` opts in. Right A starts an episode and
the next press stops/finalizes it. B/Ctrl+C finalizes the active episode before
disconnecting. `--no-camera` selects pure teleoperation with recording disabled.

Dual is the launcher default. Left/right GRIP and TRIGGER operate independent
arms, sharing one XR client. A toggles one episode containing both arms and
the shared image. If either selected arm/hand fails, the full session ends.
Single mode only opens its selected CAN channel. Verify physical hand/CAN
mapping; interface names do not determine which robot is physically left.

Dual config is `configs/piper_dual_local_305.yaml`; single config is
`configs/piper_local_305.yaml`. Both use local endpoint
`127.0.0.1:8765`, camera `CV2856D0009T`, 640×480 JPEG and 30 Hz HDF5 sampling.
State comes from the hardware control loop (20 Hz by default), so repeated
snapshots are expected. Files land at
`/home/u22/kyz/datasets/piper_dual/episodes/ep_XXXXX.h5` (dual) or
`/home/u22/kyz/datasets/piper_reach/episodes/ep_XXXXX.h5` (single), plus JSON.

In dual mode, every robot field uses `left_` / `right_` prefixes, such as
`left_joint_rad`, `right_joint_rad`, `left_tcp_flange`, `right_tcp_eef`,
`left_action_joint_rad` and `right_gripper_m`. Required six-axis/TCP and gripper
fields from both arms must be present and finite before A can start an episode.
Metadata includes mode, channel mapping and 12 joint names. Each TCP belongs
to that arm's own base frame; there is no shared-world transform or inter-arm
collision check. The single-arm LeRobot converter below is for single mode;
dual raw fields can be read with `pico_recorder/convert_to_lerobot.py`, whose
existing action convention is measured joint replay, not recorded command targets.

Both `v/tcp_flange` (SDK flange feedback) and `v/tcp_eef` (flange local +Z
offset by 0.105 m) are saved in the robot base frame, as metres and extrinsic
XYZ Euler angles in radians. The gripper center offset is **not physically
calibrated**, and metadata records that fact. `v/action_joint_rad` stores the
last joint target actually sent; `v/action_gripper_m` stores the gripper target.

Starts require fresh state and camera images. Missing/non-finite state, a robot
snapshot older than 300 ms or a camera older than 1 s fails the episode and
retains it with an error. Camera loss ends recording while teleoperation
continues. Recovery requires pressing A again. All stored state and JPEG
columns have one row per recording tick; missing legacy fields are padded.

Verified without enabling robot motors:

```bash
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python -m unittest piper_teleop_recorder.test_session -v
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python -m piper_teleop_recorder.check_local_recording
```

The second command uses real 305 images with fake arm/controller inputs and
verifies two A-key episodes, aligned rows and JPEG decoding. Actual CAN/VR
recording still requires a live hardware session. Full steps are in
[run.md](../run.md).

The following standalone launch instructions describe the older Jetson camera
setup; its hardware function now delegates to the same guarded entrypoint.

## What it does (one process, three concerns)

1. **Control thread (~20 Hz).** `XrClient` → MuJoCo IK / `JointFollower` →
   `arm.move_joints` + `arm.set_gripper(1 − trigger)`, gated by the right-GRIP
   deadman. (Same loop as `scripts/hardware/teleop_piper_x_hardware.py`.)
2. **Button-poll thread (~20 Hz).** Right-**A** rising edge toggles episode
   recording start/stop; left-**X** rising edge advances the subtask index
   through the predefined list (clamped at the last entry; never wraps).
3. **`EpisodeRecorder` (30 Hz daemon).** Snapshots the arm getters, `extra_fn`
   (subtask index, trigger, grip) and Orbbec frames into one HDF5 per episode.

### Controller map

| Input | Action |
|-------|--------|
| Right **A** | Toggle start / stop episode recording (press-edge) |
| Right **GRIP** (value > 0.9) | Deadman — hold to engage arm teleop |
| Right **TRIGGER** | Gripper opening = `1 − clip(trigger, 0, 1)` |
| Left **X** | Mark current frame as subtask boundary (advance index) |
| Right **B** | Quit the session |

## Files

| File | Role |
|------|------|
| `subtask_tracker.py` | Pure: left-X rising-edge → bounded subtask index |
| `recorder_config.py` | Pure: unified YAML config dataclass + load/`save` |
| `single_arm_lerobot.py` | Pure 6-dim concat/feature shaping + offline LeRobot writer |
| `teleop_record.py` | Orchestration (41-only); lazy HW imports; `--dry-run` |
| `configs/piper_reach.yaml` | Default unified config (edit this) |

The three pure modules are unit-tested locally in
`tests/unit_tests/test_piper_teleop_recorder.py`.

## Unified config

`configs/piper_reach.yaml` holds every tunable: `fps`, image `width`/`height`
(640×480), camera `serials`, CAN/gripper params, `task`, the predefined
`subtasks` list, `output_dir`, and the button mapping. The fully-resolved
config is written next to each session as
`{output_dir}/{task}/recorder_config_<timestamp>.yaml` for provenance.

## Dry run (no hardware)

Runs the full plumbing (recorder tick, A-edge toggle, subtask counter, HDF5
write) with fake getters / no XR / no arm / no camera, then verifies the HDF5:

```bash
cd /home/u22/kyz/pico_software
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python \
    piper_teleop_recorder/teleop_record.py \
    --config piper_teleop_recorder/configs/piper_reach.yaml --dry-run
```

Writes a throwaway HDF5 under `/tmp/piper_dryrun_*` with
`v/{subtask,joint_rad,tcp_flange,tcp_eef,gripper_m,right_trigger,right_grip,timestamp}`
and `images/<serial>/jpeg` columns.

## Hardware session (steps the operator runs on machine 41)

Preconditions (this tool does **not** start these):

1. **CAN up** — `sudo ip link set can0 up type can bitrate 1000000` (1 Mbit/s).
2. **Jetson Orbbec `stream_server` up** on `192.168.5.27:8765` (the port in the
   config). Serials `CV2R46P00097`, `CPC85630003C`.
3. **XR / VR headset service running** (XRoboToolkit desktop service on the
   display; the recorder falls back to `DISPLAY=:1`).
4. e-stop reachable, workspace clear.

Launch (motors are enabled **only** with `--enable-hardware`; without it the
tool prints a preflight checklist and exits):

```bash
cd /home/u22/kyz/pico_software
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python \
    piper_teleop_recorder/teleop_record.py \
    --config piper_teleop_recorder/configs/piper_reach.yaml --enable-hardware
```

Then: hold right-GRIP to move the arm, right-TRIGGER controls the gripper,
press right-A to start a recording and right-A again to stop it, press left-X to
step to the next subtask, right-B to quit.

Episodes land in `{output_dir}/{task}/episodes/ep_{NNNNN}.h5` (+ a sidecar
`.json`).

## Offline convert to LeRobot (single-arm, 6-dim)

The stock `pico_recorder/convert_to_lerobot.py` requires both a left and a right
joint field (dual-arm, 12-dim). This recorder is single-arm, so use the
single-arm writer, which reuses the picosoftware HDF5 readers unmodified and
runs in the LeRobot venv:

```bash
cd /home/u22/kyz/pico_software
/home/u22/kyz/lerobot_env/bin/python \
    piper_teleop_recorder/single_arm_lerobot.py \
    --record-dir /home/u22/kyz/datasets \
    --task piper_reach \
    --repo-id piper_reach \
    --task-description "reach and place"
```

`observation.state` = `action` = the 6-dim `joint_rad`; `gripper_m`,
`right_trigger` and `subtask` are written as 1-dim `observation.*` channels;
images are decoded from the stored JPEGs.

## What still needs a hardware session

The real teleop path (`--enable-hardware`) has **not** been exercised — it needs
the arm, the VR headset and the Jetson stream server. Verify there: deadman
gating, gripper mapping, the live Orbbec 30 Hz feed, and that
`arm.driver.get_flange_pose()` / `arm.gripper.get_gripper_status()` return live
values. The current guarded path refuses recording while required fields are
missing or stale; it no longer silently drops required TCP/gripper columns.
