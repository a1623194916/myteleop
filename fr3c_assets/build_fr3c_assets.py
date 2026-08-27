#!/usr/bin/env python3
"""Build MuJoCo assets for the FR3C teleop demo.

Reads fr3c_ros2_control/urdf/fr3c.urdf, strips ROS-only bits, rewrites mesh
paths to a local STL folder, appends a tool0 end-effector link, copies the
collision STL files, and compiles the result into a MuJoCo MJCF snippet that
the teleop scene can include.

Run with the teleop venv python:
  .venv/bin/python /tmp/build_fr3c_assets.py
"""
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PICO_ROOT = Path("/home/u22/kyz/pico_software")
SRC_URDF = PICO_ROOT / "fr3c_ros2_control" / "urdf" / "fr3c.urdf"
SRC_STL = PICO_ROOT / "fr3c_ros2_control" / "meshes" / "fr3c" / "collision"
OUT_DIR = PICO_ROOT / "fr3c_assets"
STL_DIR = OUT_DIR / "fr3c_stl"

OUT_DIR.mkdir(parents=True, exist_ok=True)
STL_DIR.mkdir(parents=True, exist_ok=True)

tree = ET.parse(SRC_URDF)
root = tree.getroot()

# 1) Strip ROS-only blocks that neither MuJoCo nor placo need.
for tag in ("ros2_control", "transmission"):
    for el in root.findall(tag):
        root.remove(el)

for joint in root.findall("joint"):
    for tag in ("calibration", "safety_controller"):
        el = joint.find(tag)
        if el is not None:
            joint.remove(el)

# 2) Drop visual meshes (DAE, which MuJoCo does not load) and keep collision STL.
for link in root.findall("link"):
    vis = link.find("visual")
    if vis is not None:
        link.remove(vis)

# 2b) Give every revolute joint a bit of damping so the position actuators
# don't oscillate in MuJoCo.
for joint in root.findall("joint"):
    if joint.get("type") != "revolute":
        continue
    dyn = joint.find("dynamics")
    if dyn is None:
        dyn = ET.SubElement(joint, "dynamics")
    if not dyn.get("damping"):
        dyn.set("damping", "0.3")

# 3) Rewrite package:// mesh paths to the local STL folder.
for mesh in root.iter("mesh"):
    fn = mesh.get("filename", "")
    if fn.startswith("package://fr3c_ros2_control/meshes/fr3c/collision/"):
        mesh.set("filename", "fr3c_stl/" + fn.rsplit("/", 1)[1])
    elif fn.startswith("package://"):
        raise SystemExit(f"Unhandled mesh path after rewrite: {fn}")

# 4) Append a tool0 end-effector link (welded to wrist3_Link).
tool_link = ET.Element("link", {"name": "tool0"})
tool_joint = ET.SubElement(root, "joint", {"name": "wrist3_to_tool0", "type": "fixed"})
ET.SubElement(tool_joint, "parent", {"link": "wrist3_Link"})
ET.SubElement(tool_joint, "child", {"link": "tool0"})
ET.SubElement(tool_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
root.append(tool_link)

out_urdf = OUT_DIR / "fr3c_teleop.urdf"
tree.write(out_urdf, encoding="unicode", xml_declaration=False)
print(f"wrote {out_urdf}")

# 5) Copy collision STL files next to the URDF.
for stl in sorted(SRC_STL.glob("*.STL")):
    shutil.copy2(stl, STL_DIR / stl.name)
print(f"copied {len(list(STL_DIR.glob('*.STL')))} STL files to {STL_DIR}")

# 6) Compile with MuJoCo and save the robot MJCF for <include> in the scene.
import mujoco  # noqa: E402  (venv-only dependency)

model = mujoco.MjModel.from_xml_path(str(out_urdf))
robot_xml = OUT_DIR / "fr3c_robot.xml"
try:
    mujoco.mj_saveLastXML(str(robot_xml), model)
except Exception as exc:  # pragma: no cover
    print(f"mj_saveLastXML failed: {exc}")
    sys.exit(2)
print(f"wrote {robot_xml}")
print(
    f"model summary: nq={model.nq} njnt={model.njnt} nbody={model.nbody} nu={model.nu}"
)
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    lo, hi = model.jnt_range[i]
    print(f"  joint {name}: range [{lo:.4f}, {hi:.4f}]")
