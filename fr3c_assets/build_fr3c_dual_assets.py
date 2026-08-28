#!/usr/bin/env python3
"""Build dual-arm FR3C assets from the single-arm ones.

Reads fr3c_assets/fr3c_teleop.urdf and fr3c_assets/fr3c_robot.xml (generated
by build_fr3c_assets.py), emits left_/right_ prefixed copies mounted at
(0, +0.275, 0) and (0, -0.275, 0), each yawed 180 deg so both arms reach +x:
  - fr3c_dual_robot.xml   (MuJoCo: two arm chains under static base bodies)
  - fr3c_teleop_dual.urdf (Placo: world -> two fixed base mounts)

Run with the teleop venv python:
  XRoboToolkit-Teleop-Sample-Python/.venv/bin/python fr3c_assets/build_fr3c_dual_assets.py
"""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

PICO_ROOT = Path("/home/u22/kyz/pico_software")
ASSET_DIR = PICO_ROOT / "fr3c_assets"

SIDES = {
    "left": {"pos": "0 0.275 0"},
    "right": {"pos": "0 -0.275 0"},
}
YAW_180_QUAT = "0 0 0 1"  # MuJoCo wxyz: 180 deg about z
ROOT_LINK = "world"


def build_urdf():
    src = ET.parse(ASSET_DIR / "fr3c_teleop.urdf").getroot()
    out = ET.Element("robot", {"name": "fr3c_dual"})
    ET.SubElement(out, "link", {"name": ROOT_LINK})

    for side in SIDES:
        p = lambda name: f"{side}_{name}"

        for el in src:
            if el.tag == "gazebo":
                continue
            if el.tag == "link":
                if el.get("name") == ROOT_LINK:
                    continue
                copy = deepcopy(el)
                copy.set("name", p(el.get("name")))
                out.append(copy)
            elif el.tag == "joint":
                if el.get("name") == "world_to_base":
                    continue
                copy = deepcopy(el)
                copy.set("name", p(el.get("name")))
                copy.find("parent").set("link", p(el.find("parent").get("link")))
                copy.find("child").set("link", p(el.find("child").get("link")))
                out.append(copy)

        mount = ET.Element("joint", {"name": f"world_to_{side}_base", "type": "fixed"})
        ET.SubElement(mount, "parent", {"link": ROOT_LINK})
        ET.SubElement(mount, "child", {"link": f"{side}_base_link"})
        ET.SubElement(mount, "origin", {
            "xyz": SIDES[side]["pos"],
            "rpy": "0 0 3.141592653589793",
        })
        out.append(mount)

    dest = ASSET_DIR / "fr3c_teleop_dual.urdf"
    ET.indent(ET.ElementTree(out), space="  ")
    ET.ElementTree(out).write(dest, encoding="unicode", xml_declaration=False)
    print(f"wrote {dest}")


def build_mjcf():
    src = ET.parse(ASSET_DIR / "fr3c_robot.xml").getroot()
    out = ET.Element("mujoco", {"model": "fr3c_dual"})
    ET.SubElement(out, "compiler", {"angle": "radian"})

    asset = ET.SubElement(out, "asset")
    worldbody = ET.SubElement(out, "worldbody")
    src_base_geom = src.find("worldbody").find("geom")
    src_arm_chain = src.find("worldbody").find("body")

    for side in SIDES:
        for mesh in src.find("asset").findall("mesh"):
            m = deepcopy(mesh)
            m.set("name", f"{side}_{m.get('name')}")
            asset.append(m)

        base_body = ET.SubElement(worldbody, "body", {
            "name": f"{side}_robot_base",
            "pos": SIDES[side]["pos"],
            "quat": YAW_180_QUAT,
        })
        base_geom = deepcopy(src_base_geom)
        base_geom.set("mesh", f"{side}_{base_geom.get('mesh')}")
        base_body.append(base_geom)

        arm = deepcopy(src_arm_chain)
        for el in arm.iter():
            if el.tag in ("body", "joint") and el.get("name"):
                el.set("name", f"{side}_{el.get('name')}")
            elif el.tag == "geom" and el.get("mesh"):
                el.set("mesh", f"{side}_{el.get('mesh')}")
        base_body.append(arm)

    dest = ASSET_DIR / "fr3c_dual_robot.xml"
    ET.indent(ET.ElementTree(out), space="  ")
    ET.ElementTree(out).write(dest, encoding="unicode", xml_declaration=False)
    print(f"wrote {dest}")


if __name__ == "__main__":
    build_urdf()
    build_mjcf()
