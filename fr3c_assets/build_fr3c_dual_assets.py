#!/usr/bin/env python3
"""Build dual-arm FR3C assets from the single-arm ones.

Reads fr3c_assets/fr3c_teleop.urdf and fr3c_assets/fr3c_robot.xml (generated
by build_fr3c_assets.py), emits left_/right_ prefixed copies mounted at the
configured base poses (MJCF static body + matching URDF fixed mount):
  - fr3c_dual_robot.xml   (MuJoCo: two arm chains under static base bodies)
  - fr3c_teleop_dual.urdf (Placo: world -> two fixed base mounts)

Base placement is configurable (MJCF world frame: +x forward, +y left, +z up):
  XRoboToolkit-Teleop-Sample-Python/.venv/bin/python fr3c_assets/build_fr3c_dual_assets.py \
      --left-pos 0 0.275 0 --right-pos 0 -0.275 0 --left-yaw-deg 180 --right-yaw-deg 180
"""
import argparse
import math
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

PICO_ROOT = Path("/home/u22/kyz/pico_software")
ASSET_DIR = PICO_ROOT / "fr3c_assets"

ROOT_LINK = "world"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--left-pos", nargs=3, type=float, default=[0.0, 0.275, 0.0],
                   metavar=("X", "Y", "Z"), help="left base position in world (m)")
    p.add_argument("--right-pos", nargs=3, type=float, default=[0.0, -0.275, 0.0],
                   metavar=("X", "Y", "Z"), help="right base position in world (m)")
    p.add_argument("--left-yaw-deg", type=float, default=180.0,
                   help="left base yaw about world +z (deg); 180 = arm reaches +x")
    p.add_argument("--right-yaw-deg", type=float, default=180.0,
                   help="right base yaw about world +z (deg); 180 = arm reaches +x")
    return p.parse_args()


def yaw_quat_wxyz(yaw_deg):
    """MuJoCo (w, x, y, z) quaternion for a yaw about world +z."""
    half = math.radians(yaw_deg) / 2.0
    return f"{math.cos(half):.9f} 0 0 {math.sin(half):.9f}"


def build_urdf(sides):
    src = ET.parse(ASSET_DIR / "fr3c_teleop.urdf").getroot()
    out = ET.Element("robot", {"name": "fr3c_dual"})
    ET.SubElement(out, "link", {"name": ROOT_LINK})

    for side in sides:
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

        cfg = sides[side]
        mount = ET.Element("joint", {"name": f"world_to_{side}_base", "type": "fixed"})
        ET.SubElement(mount, "parent", {"link": ROOT_LINK})
        ET.SubElement(mount, "child", {"link": f"{side}_base_link"})
        ET.SubElement(mount, "origin", {
            "xyz": " ".join(str(v) for v in cfg["pos"]),
            "rpy": f"0 0 {math.radians(cfg['yaw_deg']):.9f}",
        })
        out.append(mount)

    dest = ASSET_DIR / "fr3c_teleop_dual.urdf"
    ET.indent(ET.ElementTree(out), space="  ")
    ET.ElementTree(out).write(dest, encoding="unicode", xml_declaration=False)
    print(f"wrote {dest}")


def build_mjcf(sides):
    src = ET.parse(ASSET_DIR / "fr3c_robot.xml").getroot()
    out = ET.Element("mujoco", {"model": "fr3c_dual"})
    ET.SubElement(out, "compiler", {"angle": "radian"})

    asset = ET.SubElement(out, "asset")
    worldbody = ET.SubElement(out, "worldbody")
    src_base_geom = src.find("worldbody").find("geom")
    src_arm_chain = src.find("worldbody").find("body")

    for side in sides:
        for mesh in src.find("asset").findall("mesh"):
            m = deepcopy(mesh)
            m.set("name", f"{side}_{m.get('name')}")
            asset.append(m)

        cfg = sides[side]
        base_body = ET.SubElement(worldbody, "body", {
            "name": f"{side}_robot_base",
            "pos": " ".join(str(v) for v in cfg["pos"]),
            "quat": yaw_quat_wxyz(cfg["yaw_deg"]),
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
    args = parse_args()
    sides = {
        "left": {"pos": args.left_pos, "yaw_deg": args.left_yaw_deg},
        "right": {"pos": args.right_pos, "yaw_deg": args.right_yaw_deg},
    }
    build_urdf(sides)
    build_mjcf(sides)

