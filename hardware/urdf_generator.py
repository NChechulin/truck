from collections import defaultdict
from pathlib import Path
from typing import Iterable
from lxml import etree
import argparse
import shutil
import math
import json
import yaml


class URDFBuilder:
    def __init__(
        self,
        robot_name: str,
        cad_export_path: Path,
        mesh_output_path: Path,
        mesh_uri_prefix: str,
        float_precision: int = 5,
    ):
        self._robot_name = robot_name
        self._cad_export_path = cad_export_path
        self._mesh_output_path = mesh_output_path
        self._mesh_uri_prefix = mesh_uri_prefix
        self._precision = float_precision
        self._cad_groups = None
        self._cad_components = None
        self._cad_joints = None
        self._cad_collisions = None
        self._link_tags = []
        self._joint_tags = []
        self._link_meta = {}
        self._import_data()

    def find_cad_group(self, *selector):
        for comp in self._cad_components.values():
            if all(x in comp["id"] for x in selector):
                return comp["group"]
        raise ValueError("Can't find matching component")

    def find_cad_joint(self, name):
        return self._cad_joints[name]

    def find_cad_collision(self, name):
        return self._cad_collisions[name]

    def register_link(self, name: str, group_id: int, collision_id: str = None):
        group = self._cad_groups[group_id]
        link_tag = etree.Element("link", name=name)

        if group.get("mesh") is not None:
            visual_tag = etree.Element("visual")
            # X=left, Y=up, Z=forward ---> X=forward, Y=right, Z=up
            rpy = (math.pi / 2, 0, math.pi / 2)
            visual_tag.append(etree.Element("origin", rpy=self._str(rpy)))
            source_mesh_path = self._cad_export_path / group["mesh"]
            dst_mesh_path = self._mesh_output_path / group["mesh"]
            dst_mesh_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(source_mesh_path, dst_mesh_path)
            shutil.copy(source_mesh_path.with_suffix(".mtl"), dst_mesh_path.with_suffix(".mtl"))
            uri = self._mesh_uri_prefix + f"/{dst_mesh_path.name}"
            geometry = etree.Element("geometry")
            geometry.append(etree.Element("mesh", filename=uri))
            visual_tag.append(geometry)
            link_tag.append(visual_tag)
        
        inertial_tag = etree.Element("inertial")
        inertial_tag.append(etree.Element("mass", value=self._str(group["mass"])))
        xx, yy, zz, xy, yz, xz = group["inertia"]
        inertia_tag = etree.Element(
            "inertia",
            ixx=self._str(xx),
            iyy=self._str(yy),
            izz=self._str(zz),
            ixy=self._str(xy),
            iyz=self._str(yz),
            ixz=self._str(xz),
        )
        inertial_tag.append(inertia_tag)
        link_tag.append(inertial_tag)
        
        if collision_id:
            for coll_obj in self._cad_collisions[collision_id]:
                collision_tag = etree.Element("collision")
                origin = etree.Element(
                    "origin",
                    xyz=self._str(coll_obj["center"]),
                    rpy=self._str(coll_obj["rotation"]),
                )
                collision_tag.append(origin)
                geom_tag = etree.Element("geometry")
                if coll_obj["type"] == "cylinder":
                    geom_tag.append(etree.Element(
                        "cylinder", 
                        radius=self._str(coll_obj["radius"]), 
                        length=self._str(coll_obj["length"]),
                    ))
                if coll_obj["type"] == "box":
                    geom_tag.append(etree.Element(
                        "box", 
                        size=self._str(coll_obj["size"]),
                    ))
                collision_tag.append(geom_tag)
                link_tag.append(collision_tag)

        self._link_tags.append(link_tag)
        return link_tag

    def register_joint(self, name: str, joint_id: str, parent_link: str, child_link: str):
        world_offset = self._get_world_offset(parent_link)
        # print(parent_link, "->", child_link, "world offset:", world_offset)
        joint = self._cad_joints[joint_id]
        joint_tag = etree.Element("joint", name=name)
        joint_tag.append(etree.Element("parent", link=parent_link))
        joint_tag.append(etree.Element("child", link=child_link))
        if joint["type"] in ("revolute", "continuous"):
            origin = [x - dx for x, dx in zip(joint["origin"], world_offset)]
            # print("original:", joint["origin"], "fixed:", origin)
            joint_tag.append(etree.Element("axis", xyz=self._str(joint["axis"])))
            joint_tag.append(etree.Element("origin", xyz=self._str(origin)))
            joint_tag.attrib["type"] = joint["type"]
            if joint["type"] == "revolute":
                joint_tag.append(
                    etree.Element(
                        "limit",
                        effort="30",  # TODO
                        velocity="1.0",  # TODO
                        lower=self._str(joint["limits"][0]),
                        upper=self._str(joint["limits"][1]),
                    )
                )
        else:
            raise RuntimeError(f"Unsupported joint type: {joint['type']!r}")

        self._link_meta[child_link] = {
            "parent": parent_link,
            "translation": origin,
        }
        self._joint_tags.append(joint_tag)
        return joint_tag

    def write_urdf(self, path: Path):
        for link in self._link_tags:
            self._fix_offsets(link)
        root = etree.Element("robot", name=self._robot_name)
        root.append(etree.Comment(" LINKS SECTION "))
        root.extend(self._link_tags)
        root.append(etree.Comment(" JOINTS SECTION "))
        root.extend(self._joint_tags)
        etree.indent(root, " " * 2)
        content = etree.tostring(root, pretty_print=True)
        with open(path, "wb") as file:
            file.write(b'<?xml version="1.0"?>\n')
            file.write(content)

    def _import_data(self):
        with open(self._cad_export_path / "export.json") as file:
            data = json.load(file)
        self._cad_components = {x["id"]: x for x in data["components"]}
        self._cad_groups = {x["id"]: x for x in data["groups"]}
        self._cad_joints = {x["id"]: x for x in data["joints"]}
        self._cad_collisions = defaultdict(list)
        for item in data["collisions"]:
            self._cad_collisions[item["id"]].append(item)

    def _str(self, *args):
        if len(args) == 1 and isinstance(args[0], Iterable):
            args = list(args[0])
        tokens = [str(round(x, self._precision)) for x in args]
        return " ".join(tokens)

    def _get_world_offset(self, link_name):
        offset_sum = [0, 0, 0]
        meta = self._link_meta.get(link_name)
        while meta is not None:
            parent_offset = meta["translation"]
            offset_sum = [x + dx for x, dx in zip(offset_sum, parent_offset)]
            meta = self._link_meta.get(meta["parent"])
        return offset_sum

    def _fix_offsets(self, link):
        link_name = link.attrib["name"]
        world_offset = self._get_world_offset(link_name)
        # print(f"offset for link {link_name}: {world_offset}")
        origin_tag = link.find("visual")
        if origin_tag is not None:
            origin_tag = origin_tag.find("origin")
            origin_tag.attrib["xyz"] = self._str([-x for x in world_offset])
        origin_tag = link.find("collision")
        if origin_tag is not None:
            origin_tags = origin_tag.findall("origin")
            for origin_tag in origin_tags:
                old_xyz = [float(x) for x in origin_tag.attrib["xyz"].split()]
                new_xyz = [x - dx for x, dx in zip(old_xyz, world_offset)]
                origin_tag.attrib["xyz"] = self._str(new_xyz)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", default="truck")
    parser.add_argument("--cad-export-path", default="hardware/cad_export")
    parser.add_argument("--urdf-output-path", default="hardware/model/model.urdf")
    parser.add_argument("--yaml-output-path", default="hardware/model/config.yaml")
    parser.add_argument("--mesh-output-path", default="hardware/model/meshes")
    parser.add_argument("--mesh-uri-prefix", default="file://hardware/model/meshes")
    parser.add_argument("--float-precision", default=5)
    args = parser.parse_args()

    urdf = URDFBuilder(
        robot_name=args.robot_name,
        cad_export_path=Path(args.cad_export_path),
        mesh_output_path=Path(args.mesh_output_path),
        mesh_uri_prefix=args.mesh_uri_prefix,
        float_precision=args.float_precision,
    )

    urdf.register_link("base", urdf.find_cad_group("Profile Base"), "Base Collision")
    urdf.register_link("front_left_cup", urdf.find_cad_group("Front Left Wheel", "Inner Cup"))
    urdf.register_link("front_right_cup", urdf.find_cad_group("Front Right Wheel", "Inner Cup"))
    urdf.register_link("front_left_wheel", urdf.find_cad_group("Front Left Wheel", "Generic Wheel"), "FL Wheel Collision")
    urdf.register_link("front_right_wheel", urdf.find_cad_group("Front Right Wheel", "Generic Wheel"), "FR Wheel Collision")
    urdf.register_link("rear_left_wheel", urdf.find_cad_group("Rear Left Wheel", "Generic Wheel"), "RL Wheel Collision")
    urdf.register_link("rear_right_wheel", urdf.find_cad_group("Rear Right Wheel", "Generic Wheel"), "RR Wheel Collision")
    
    urdf.register_joint("front_left_wheel_steering", "Front Left Wheel Steering", "base", "front_left_cup")
    urdf.register_joint("front_right_wheel_steering", "Front Right Wheel Steering", "base", "front_right_cup")
    urdf.register_joint("front_left_wheel_axle", "Front Left Wheel Spin", "front_left_cup", "front_left_wheel")
    urdf.register_joint("front_right_wheel_axle", "Front Right Wheel Spin", "front_right_cup", "front_right_wheel")
    urdf.register_joint("rear_left_wheel_axle", "Rear Left Wheel Spin", "base", "rear_left_wheel")
    urdf.register_joint("rear_right_wheel_axle", "Rear Right Wheel Spin", "base", "rear_right_wheel")

    front_left_wheel_collision = urdf.find_cad_collision("FL Wheel Collision")
    front_left_steering_joint = urdf.find_cad_joint("Front Left Wheel Steering")
    # In CAD all positions are in absolute coordinates
    jx, jy, _ = front_left_steering_joint["origin"]
    wx, wy, _ = front_left_wheel_collision[0]["center"]
    wheel_base_length = abs(wx) * 2
    wheel_base_width = abs(wy) * 2
    steering_width = abs(jy) * 2
    max_left_wheel_angle = max(front_left_steering_joint["limits"])
    min_turn_radius = wheel_base_length / math.tan(max_left_wheel_angle) + wheel_base_width / 2
    
    print("Wheel base length:", round(wheel_base_length * 1000), "mm")
    print("Wheel base width:", round(wheel_base_width * 1000), "mm")
    print("Max wheel angle:", round(max_left_wheel_angle * 180 / math.pi), "deg")
    print("Min turn radius:", round(min_turn_radius, 2), "m")

    urdf.write_urdf(Path(args.urdf_output_path))
    yaml_output_path = Path(args.yaml_output_path)
    data = {}
    if yaml_output_path.exists():
        with open(yaml_output_path) as file:
            data = yaml.load(file, Loader=yaml.Loader)
    data["wheel_base_length"] = round(wheel_base_length, args.float_precision)
    data["wheel_base_width"] = round(wheel_base_width, args.float_precision)
    data["min_turn_radius"] = round(min_turn_radius, args.float_precision)
    data["max_curvature"] = round(1 / min_turn_radius, args.float_precision)
    with open(yaml_output_path, "w") as file:
        yaml.dump(data, file, sort_keys=False)
