# fr3c_assets

MuJoCo assets for the FR3C teleoperation demo, generated from
`fr3c_ros2_control/urdf/fr3c.urdf`.

## Contents

- `fr3c_teleop.urdf` — kinematics-only URDF (ros2_control blocks, visual DAEs,
  calibration/safety tags stripped; joint damping added).
- `fr3c_robot.xml` — the same robot compiled to MJCF (via
  `mj_saveLastXML`), included by `scene_fr3c.xml`.
- `fr3c_stl/` — collision STL meshes used by both the URDF and the MJCF.
- `scene_fr3c.xml` — simulation scene: floor, lights, a mocap target marker
  for the right hand, position actuators (kp per joint) and the `home`
  keyframe `qpos = [0, -pi/2, 0.9, -1.2, pi/2, 0]`.
- `build_fr3c_assets.py` — regenerates everything above from the FR3C URDF:

  ```bash
  cd /home/u22/kyz/pico_software
  XRoboToolkit-Teleop-Sample-Python/.venv/bin/python fr3c_assets/build_fr3c_assets.py
  ```

## Regeneration notes

- Visual meshes in the original URDF are `.dae`, which MuJoCo does not load;
  the teleop model therefore uses the collision STLs for rendering.
- `tool0` is appended as a fixed end-effector link, but MuJoCo welds fixed
  joints into the parent body, so the teleop config tracks `wrist3_Link`.
- The home keyframe and `ctrl` targets keep the arm folded like a typical UR
  start pose; j5 uses the FR3C convention `[0, 2*pi]` (single-sided).
