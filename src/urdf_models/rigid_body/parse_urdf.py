"""Parse a URDF file into a tree and print it."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: Optional[str] = None
    origin_rpy: Optional[str] = None
    axis: Optional[str] = None


@dataclass
class Link:
    name: str
    children: list["Link"] = field(default_factory=list)
    joint_to_parent: Optional[Joint] = None


def parse_urdf(filepath: str) -> tuple[dict[str, Link], list[Joint]]:
    tree = ET.parse(filepath)
    root = tree.getroot()

    links: dict[str, Link] = {}
    joints: list[Joint] = []

    for link_el in root.findall("link"):
        name = link_el.get("name")
        links[name] = Link(name=name)

    for joint_el in root.findall("joint"):
        name = joint_el.get("name")
        joint_type = joint_el.get("type")
        parent = joint_el.find("parent").get("link")
        child = joint_el.find("child").get("link")

        origin_el = joint_el.find("origin")
        origin_xyz = origin_el.get("xyz") if origin_el is not None else None
        origin_rpy = origin_el.get("rpy") if origin_el is not None else None

        axis_el = joint_el.find("axis")
        axis = axis_el.get("xyz") if axis_el is not None else None

        joints.append(Joint(
            name=name,
            joint_type=joint_type,
            parent=parent,
            child=child,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=axis,
        ))

    return links, joints


def build_tree(links: dict[str, Link], joints: list[Joint]) -> Link:
    children: set[str] = set()

    for joint in joints:
        child_link = links[joint.child]
        child_link.joint_to_parent = joint
        links[joint.parent].children.append(child_link)
        children.add(joint.child)

    roots = [link for name, link in links.items() if name not in children]
    if len(roots) != 1:
        raise ValueError(f"Expected exactly one root link, found: {[r.name for r in roots]}")
    return roots[0]


def print_tree(node: Link, prefix: str = "", is_last: bool = True) -> None:
    connector = "└── " if is_last else "├── "
    joint_info = ""
    if node.joint_to_parent:
        j = node.joint_to_parent
        joint_info = f"  [{j.name} | {j.joint_type}]"

    print(f"{prefix}{connector}{node.name}{joint_info}")

    child_prefix = prefix + ("    " if is_last else "│   ")
    for i, child in enumerate(node.children):
        print_tree(child, child_prefix, is_last=(i == len(node.children) - 1))


if __name__ == "__main__":
    import sys

    urdf_path = sys.argv[1] if len(sys.argv) > 1 else "robot.urdf"
    links, joints = parse_urdf(urdf_path)
    root = build_tree(links, joints)

    print(f"Robot link tree ({len(links)} links, {len(joints)} joints)\n")
    print(root.name)
    for i, child in enumerate(root.children):
        print_tree(child, prefix="", is_last=(i == len(root.children) - 1))
