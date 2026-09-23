#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Generate roboarm.urdf — Piper (gripper removed) + adapter + LinkerHand O6 right.

    python3 urdf/make_urdf.py            # write src/roboarm_description/urdf/roboarm.urdf
    python3 urdf/make_urdf.py --check    # verify the checked-in file matches this script

Sources (vendored under urdf/source/, override with env if you have newer ones):

    ROBOARM_PIPER_URDF_SRC   AgileX piper_description, gripper-less variant
                             (the official `piper_no_gripper_description.urdf`;
                             pointing this at the with-gripper file also works —
                             the gripper is stripped below either way)
    ROBOARM_HAND_URDF_SRC    LinkerHand O6 RIGHT-hand URDF (rh_* links)

What it does, in order:

  1. drops the Piper gripper — links gripper_base/link7/link8 and joints
     joint6_to_gripper_base/joint7/joint8. The real arm has no gripper there; a
     LinkerHand O6 hangs off the flange instead. (No-op for the default source,
     which is already gripper-less; kept so a with-gripper URDF can be fed in
     without producing a robot with a phantom gripper.)
  2. rewrites every mesh URI into this ROS package's own layout:
         package://piper_description/meshes/X  ->  package://roboarm_description/meshes/piper/X
         meshes/X (hand, relative)             ->  package://roboarm_description/meshes/hand/X
  3. appends the hand's links/joints as-is (its own rh_* names are kept — they are
     what the hand's URDF, the MuJoCo scene and the vendor docs all use).
  4. bolts the hand to the flange with a FIXED joint, no rotation, 17.5 mm out:
         link6 ──rh_adapter_joint──> rh_hand_base_link      (xyz 0 0 0.0175)
     and adds the adapter's own mesh as a second <visual> on link6, so the part
     that makes this robot not-a-Piper is actually visible in RViz.
     The joint does NOT need to be in any joint_states stream: it is fixed, so
     robot_state_publisher latches it onto /tf_static.
  5. self-checks: exactly one root link, no link with two parents, no duplicate
     joint names, the expected link/joint set, and every mesh file present on
     disk (a typo in a mesh path would otherwise only show up as an RViz warning).

The 17.5 mm comes from the MuJoCo scene (sim/piper_linker/scene.xml), where the
same adapter is modelled as <body name="rh_hand_base_link" pos="0 0 0.0175">.
Keeping the URDF and the scene in agreement is the point: the sim is how we
validate grasp poses without risking the hardware.
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
OUT = PKG / "src" / "roboarm_description" / "urdf" / "roboarm.urdf"
MESH_DIR = PKG / "src" / "roboarm_description" / "meshes"

PIPER_SRC = Path(
    os.environ.get("ROBOARM_PIPER_URDF_SRC", HERE / "source" / "piper_no_gripper.urdf")
)
HAND_SRC = Path(
    os.environ.get("ROBOARM_HAND_URDF_SRC", HERE / "source" / "linkerhand_o6_right.urdf")
)

ROBOT_NAME = "roboarm_piper_linkerhand_o6_right"
MESH_PREFIX = "package://roboarm_description/meshes"

ADAPTER_JOINT = "rh_adapter_joint"
ADAPTER_MESH = f"{MESH_PREFIX}/piper/adapter.stl"
HAND_OFFSET_Z = 0.0175

DROP_JOINTS = {"joint6_to_gripper_base", "joint7", "joint8"}
DROP_LINKS = {"gripper_base", "link7", "link8"}

EXPECTED_PIPER_LINKS = {"base_link", "link1", "link2", "link3", "link4", "link5", "link6"}
EXPECTED_PIPER_JOINTS = {f"joint{i}" for i in range(1, 7)}
EXPECTED_HAND_LINKS = {
    "rh_hand_base_link",
    "rh_thumb_metacarpals_base2", "rh_thumb_metacarpals", "rh_thumb_distal",
    "rh_index_proximal", "rh_index_distal",
    "rh_middle_proximal", "rh_middle_distal",
    "rh_ring_proximal", "rh_ring_distal",
    "rh_pinky_proximal", "rh_pinky_distal",
}
EXPECTED_HAND_JOINTS = {
    "rh_thumb_cmc_yaw", "rh_thumb_cmc_pitch", "rh_thumb_ip",
    "rh_index_mcp_pitch", "rh_index_dip",
    "rh_middle_mcp_pitch", "rh_middle_dip",
    "rh_ring_mcp_pitch", "rh_ring_dip",
    "rh_pinky_mcp_pitch", "rh_pinky_dip",
}

HEADER = """\
  <!-- 由 robonix/packages/primitive-piper_with_linkerhand-description-rbnx/urdf/make_urdf.py 生成，请勿手改。
       Piper(去夹爪) + 17.5mm 同轴连接件 + LinkerHand O6 右手。
       重新生成: python3 urdf/make_urdf.py -->"""


def indent(elem: ET.Element, level: int = 0) -> None:
    """Pretty-print (ElementTree has no built-in indenter)."""
    pad = "\n  " * level
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + "  "
        for child in elem:
            indent(child, level + 1)
        if not (child.tail or "").strip():
            child.tail = pad
    if level and not (elem.tail or "").strip():
        elem.tail = pad


def rewrite_meshes(node: ET.Element, *, prefix: str) -> None:
    """Point every <mesh filename> at this package's mesh tree."""
    for mesh in node.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn.startswith("package://"):
            # e.g. package://piper_description/meshes/link1.STL
            fn = fn.split("/meshes/", 1)[-1] if "/meshes/" in fn else fn.rsplit("/", 1)[-1]
        else:
            # hand URDF uses paths relative to its own package: meshes/x.STL
            fn = fn.split("meshes/", 1)[-1] if "meshes/" in fn else fn.rsplit("/", 1)[-1]
        mesh.set("filename", f"{MESH_PREFIX}/{prefix}/{fn}")


def build() -> ET.Element:
    piper = ET.parse(PIPER_SRC).getroot()
    hand = ET.parse(HAND_SRC).getroot()

    # 1. drop the gripper, 2. re-home the arm meshes
    for j in list(piper.findall("joint")):
        if j.get("name") in DROP_JOINTS:
            piper.remove(j)
    for l in list(piper.findall("link")):
        if l.get("name") in DROP_LINKS:
            piper.remove(l)
    rewrite_meshes(piper, prefix="piper")

    # 3. hand links/joints (and its top-level materials, if it has any)
    for mat in hand.findall("material"):
        piper.append(mat)
    for child in list(hand):
        if child.tag in ("link", "joint"):
            piper.append(child)
    rewrite_meshes(hand, prefix="hand")

    # 4a. the adapter itself: a second visual on link6
    link6 = piper.find(".//link[@name='link6']")
    assert link6 is not None, "piper source has no link6"
    adapter_visual = ET.SubElement(link6, "visual")
    ET.SubElement(adapter_visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geom = ET.SubElement(adapter_visual, "geometry")
    ET.SubElement(geom, "mesh", {"filename": ADAPTER_MESH})

    # 4b. the fixed joint that mounts the hand on the flange
    adapter_joint = ET.Element("joint", {"name": ADAPTER_JOINT, "type": "fixed"})
    ET.SubElement(adapter_joint, "origin", {"xyz": f"0 0 {HAND_OFFSET_Z}", "rpy": "0 0 0"})
    ET.SubElement(adapter_joint, "parent", {"link": "link6"})
    ET.SubElement(adapter_joint, "child", {"link": "rh_hand_base_link"})
    piper.append(adapter_joint)

    piper.set("name", ROBOT_NAME)
    return piper


def self_check(root: ET.Element) -> list[str]:
    """Return a list of problems (empty == good)."""
    problems: list[str] = []
    links = {l.get("name") for l in root.findall("link")}
    joints = root.findall("joint")

    names = [j.get("name") for j in joints]
    if len(names) != len(set(names)):
        problems.append(f"duplicate joint names: {sorted(n for n in names if names.count(n) > 1)}")

    children: set[str] = set()
    for j in joints:
        child = j.find("child")
        if child is None or j.find("parent") is None:
            problems.append(f"joint {j.get('name')} is missing parent/child")
            continue
        link = child.get("link")
        if link in children:
            problems.append(f"{link} has two parents")
        children.add(link)

    roots = links - children
    if roots != {"base_link"}:
        problems.append(f"expected exactly one root (base_link); got {sorted(roots)}")

    expect_joints = EXPECTED_PIPER_JOINTS | EXPECTED_HAND_JOINTS | {ADAPTER_JOINT}
    got_joints = set(names)
    if missing := expect_joints - got_joints:
        problems.append(f"missing joints: {sorted(missing)}")
    if extra := got_joints - expect_joints:
        problems.append(f"unexpected joints: {sorted(extra)}")

    expect_links = EXPECTED_PIPER_LINKS | EXPECTED_HAND_LINKS
    if missing := expect_links - links:
        problems.append(f"missing links: {sorted(missing)}")
    if extra := links - expect_links:
        problems.append(f"unexpected links: {sorted(extra)}")

    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        if not fn.startswith(f"{MESH_PREFIX}/"):
            problems.append(f"mesh outside this package: {fn}")
            continue
        rel = fn[len(MESH_PREFIX) + 1 :]
        if not (MESH_DIR / rel).is_file():
            problems.append(f"mesh file missing on disk: meshes/{rel}")

    return problems


def to_xml(root: ET.Element) -> str:
    indent(root)
    body = ET.tostring(root, encoding="unicode")
    open_tag, _, rest = body.partition(">")
    return f"<?xml version='1.0' encoding='utf-8'?>\n{open_tag}>\n{HEADER}{rest}\n"


def canon(elem: ET.Element):
    """Order-insensitive canonical form, for --check."""
    return (
        elem.tag,
        tuple(sorted(elem.attrib.items())),
        (elem.text or "").strip(),
        tuple(canon(c) for c in elem),
    )


def main(argv: list[str]) -> int:
    check = "--check" in argv[1:]
    root = build()
    problems = self_check(root)
    if problems:
        for p in problems:
            print(f"[make_urdf] FAIL {p}", file=sys.stderr)
        return 1

    xml = to_xml(root)
    if check:
        if not OUT.is_file():
            print(f"[make_urdf] FAIL {OUT} does not exist", file=sys.stderr)
            return 1
        current = canon(ET.parse(OUT).getroot())
        if current != canon(ET.fromstring(xml)):
            print(
                f"[make_urdf] FAIL {OUT} is out of date — re-run without --check",
                file=sys.stderr,
            )
            return 1
        print(f"[make_urdf] OK {OUT.relative_to(PKG)} matches the generator")
        return 0

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(xml, encoding="utf-8")
    n_links = len(root.findall("link"))
    n_joints = len(root.findall("joint"))
    print(f"[make_urdf] {n_links} links, {n_joints} joints -> {OUT.relative_to(PKG)}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
