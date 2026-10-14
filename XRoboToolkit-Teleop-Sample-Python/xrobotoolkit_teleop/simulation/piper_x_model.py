"""Build a Piper-X MuJoCo scene from AgileX's pinned URDF and collision meshes."""

from pathlib import Path
from urllib.request import urlopen
import xml.etree.ElementTree as ET

import mujoco


SOURCE_REV = "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
SOURCE = f"https://raw.githubusercontent.com/agilexrobotics/agx_arm_urdf/{SOURCE_REV}/piper_x"
ASSETS = Path(__file__).resolve().parents[2] / "assets" / "piper_x" / "generated"
JOINTS = tuple(f"joint{i}" for i in range(1, 7))
HOME = (0.0, 1.15, -1.45, 0.0, 0.3, 0.0)
GRIPPER_JOINTS = ("gripper_joint1", "gripper_joint2")
DEFAULT_GRIPPER_WIDTH = 0.07
TCP_LINK = "piper_x_tcp"
# Midway along the two official finger meshes (link6-local z spans 0.066-0.143 m).
TCP_OFFSET_M = 0.105


def _fetch(relative: str, destination: Path) -> None:
    if destination.exists():
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    with urlopen(f"{SOURCE}/{relative}", timeout=30) as response:
        payload = response.read()
    destination.write_bytes(payload)


def build_scene(asset_dir: Path = ASSETS) -> tuple[Path, Path]:
    """Return (URDF, MJCF) with the official Piper-X arm and gripper meshes."""
    asset_dir.mkdir(parents=True, exist_ok=True)
    source_urdf = asset_dir / "source.urdf"
    gripper_xacro = asset_dir / "source_gripper.xacro"
    _fetch("urdf/piper_x_description.urdf", source_urdf)
    _fetch("urdf/piper_x_with_gripper_description.xacro", gripper_xacro)
    urdf_path = asset_dir / "piper_x.urdf"
    scene_path = asset_dir / "scene.xml"
    if urdf_path.exists() and scene_path.exists():
        scene_root = ET.parse(scene_path).getroot()
        if scene_root.get("model") == "piper_x_teleop_v4":
            return urdf_path, scene_path

    root = ET.parse(source_urdf).getroot()
    attachment = ET.parse(gripper_xacro).getroot()
    for part in attachment:
        if part.tag not in {"link", "joint"} or part.get("name") in {"gripper_link", "gripper"}:
            continue
        mimic = part.find("mimic")
        if mimic is not None:
            part.remove(mimic)
        root.append(part)
    ET.SubElement(root, "link", {"name": TCP_LINK})
    tcp_joint = ET.SubElement(root, "joint", {"name": "piper_x_tcp_joint", "type": "fixed"})
    ET.SubElement(tcp_joint, "origin", {"xyz": f"0 0 {TCP_OFFSET_M}", "rpy": "0 0 0"})
    ET.SubElement(tcp_joint, "parent", {"link": "link6"})
    ET.SubElement(tcp_joint, "child", {"link": TCP_LINK})
    for link in root.findall("link"):
        # DAE visual meshes are not supported by MuJoCo's URDF importer.
        for visual in link.findall("visual"):
            link.remove(visual)
        for mesh in link.findall("./collision/geometry/mesh"):
            filename = mesh.attrib["filename"]
            name = Path(filename).name
            _fetch(f"meshes/{name}", asset_dir / name)
            mesh.set("filename", name)
    ET.ElementTree(root).write(urdf_path, encoding="unicode")

    model = mujoco.MjModel.from_xml_path(str(urdf_path))
    mujoco.mj_saveLastXML(str(scene_path), model)
    scene = ET.parse(scene_path)
    mjcf = scene.getroot()
    mjcf.set("model", "piper_x_teleop_v4")
    world = mjcf.find("worldbody")
    if world is None:
        raise ValueError("Imported Piper-X URDF has no worldbody")
    link6 = world.find(".//body[@name='link6']")
    if link6 is None:
        raise ValueError("Imported Piper-X model has no link6")
    # MJCF keeps this fixed frame as a named body for the viewer/controller.
    if world.find(f".//body[@name='{TCP_LINK}']") is None:
        ET.SubElement(link6, "body", {"name": TCP_LINK, "pos": f"0 0 {TCP_OFFSET_M}"})
    for joint in world.iter("joint"):
        if joint.get("name") in JOINTS:
            joint.set("armature", "0.1")
    # Imported convex collision meshes overlap at their neighboring links.
    # Preserve the official geometry and external-object contact eligibility,
    # but avoid the arm colliding with itself in this free-space teleop scene.
    for geom in world.iter("geom"):
        if geom.get("type") == "mesh":
            geom.set("conaffinity", "0")
    ET.SubElement(world, "light", {"pos": "0 -0.6 1.4", "dir": "0 0 -1"})
    ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "pos": "0 0 -0.02", "size": "2 2 0.01", "rgba": "0.75 0.78 0.8 1", "contype": "0", "conaffinity": "0"})
    ET.SubElement(world, "body", {"name": "target", "mocap": "true", "pos": "0.4 0 0.4"}).append(
        ET.Element("geom", {"type": "sphere", "size": "0.025", "rgba": "0.9 0.25 0.2 0.5", "contype": "0", "conaffinity": "0"})
    )
    actuators = ET.SubElement(mjcf, "actuator")
    for name in (*JOINTS, *GRIPPER_JOINTS):
        ET.SubElement(actuators, "position", {
            "name": name, "joint": name,
            "kp": "1000" if name in JOINTS else "300",
            "kv": "35" if name in JOINTS else "4",
        })
    keyframe = ET.SubElement(mjcf, "keyframe")
    home_qpos = " ".join(map(str, (*HOME, DEFAULT_GRIPPER_WIDTH / 2, -DEFAULT_GRIPPER_WIDTH / 2)))
    ET.SubElement(keyframe, "key", {
        "name": "home",
        "qpos": home_qpos,
        "ctrl": home_qpos,
    })
    scene.write(scene_path, encoding="unicode")
    # Fail during preparation rather than at the first teleoperation tick.
    mujoco.MjModel.from_xml_path(str(scene_path))
    return urdf_path, scene_path
