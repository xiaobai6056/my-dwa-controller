#!/usr/bin/env python3
"""Fetch two pinned visual assets and generate local Gazebo Classic worlds."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from functools import lru_cache, partial
from math import pi
from pathlib import Path
import re
from struct import unpack_from
from urllib.request import urlopen
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np

ROOT = Path(__file__).resolve().parent
CACHE = ROOT / ".cache"
MODELS = ROOT / "models"
RAW = "https://raw.githubusercontent.com"
GEMS = (f"{RAW}/npu-ius-lab/Gazebo-Models/"
        "12ff07b6ad6e7e6031b8227854dffeb6f02dffa0/models/Miscellaneous/human_female_1")
SUBJECT = f"{RAW}/gbionics/human-gazebo/223146b8ab48fc43c376fe0678e24c59090fa279"
PITCH = -0.445
CAMERA_X = 0.069
CAMERA_Y = -0.047
OPTICAL = "camera_rgb_optical_frame"
FRONT_YAW = {"gems": -pi / 2, "subject": pi}


def fetch(destination, url):
    if not destination.is_file():
        destination.parent.mkdir(parents=True, exist_ok=True)
        with urlopen(url, timeout=45) as response:
            data = response.read()
        temporary = destination.with_suffix(destination.suffix + ".part")
        temporary.write_bytes(data)
        temporary.replace(destination)
    return destination


def write_xml(element, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(element)
    ET.ElementTree(element).write(path, encoding="utf-8", xml_declaration=True)


def prepare_robot():
    share = Path(get_package_share_directory("turtlebot3_gazebo"))
    sdf = ET.parse(share / "models/turtlebot3_waffle/model.sdf").getroot()
    model = sdf.find("model")
    for link in model.findall("link"):
        for sensor in link.findall("sensor"):
            if sensor.get("type") != "camera":
                link.remove(sensor)
    camera = model.find(".//sensor[@type='camera']")
    camera.find("visualize").text = "false"
    camera.find("update_rate").text = "15"
    ET.SubElement(camera, "pose").text = f"0 0 0 0 {PITCH} 0"
    camera.find("camera/horizontal_fov").text = "1.57"
    camera.find("camera/image/width").text = "640"
    camera.find("camera/image/height").text = "480"
    camera.find("camera/clip/far").text = "30"
    plugin = camera.find("plugin")
    ET.SubElement(plugin.find("ros"), "namespace").text = "/visual_prototype"
    ET.SubElement(plugin, "camera_name").text = "camera"
    ET.SubElement(plugin, "frame_name").text = OPTICAL
    write_xml(sdf, CACHE / "waffle.sdf")

    urdf = ET.parse(share / "urdf/turtlebot3_waffle.urdf").getroot()
    urdf.find(".//joint[@name='camera_rgb_joint']/origin").set("rpy", f"0 {PITCH} 0")
    urdf.find(".//joint[@name='camera_rgb_optical_joint']/origin").set(
        "rpy", "-1.5707963267948966 0 -1.5707963267948966")
    write_xml(urdf, CACHE / "waffle.urdf")
    return model


def visual_model(name):
    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    return sdf, ET.SubElement(model, "link", name="body")


def prepare_gems():
    name = "human_female_1"
    directory = MODELS / name
    fetch(directory / "model.config", f"{GEMS}/model.config")
    original = ET.parse(fetch(directory / "upstream.sdf", f"{GEMS}/model.sdf"))
    sdf, link = visual_model(name)
    references = set()
    materials = set()
    for source in original.findall(".//link/visual"):
        visual = deepcopy(source)
        mesh = visual.find("geometry/mesh/uri")
        filename = Path(mesh.text).name
        # Upstream's belt URI points at a sibling with a byte-identical belt.
        mesh.text = f"model://{name}/meshes/{filename}"
        visual.set("name", Path(filename).stem)
        visual.find("pose").attrib.pop("frame", None)
        references.add(f"meshes/{filename}")
        material = visual.find("material/script/name")
        if material is not None and "/" not in material.text:
            materials.add(f"materials/scripts/{material.text}.material")
        link.append(visual)

    get = partial(fetch_gems_file, directory)
    with ThreadPoolExecutor(max_workers=8) as pool:
        files = list(pool.map(get, sorted(references | materials)))
        textures = set()
        for file in files:
            if file.suffix == ".material":
                textures.update(re.findall(r"\btexture\s+(\S+)", file.read_text()))
            else:
                for node in ET.parse(file).findall(".//{*}library_images/{*}image/{*}init_from"):
                    textures.add(Path(node.text).name)
        list(pool.map(get, sorted(f"materials/textures/{texture}" for texture in textures)))

    write_xml(sdf, directory / "model.sdf")
    return sdf.find("model")


def fetch_gems_file(directory, relative):
    return fetch(directory / relative, f"{GEMS}/{relative}")


def transform(xyz, rpy):
    sr, sp, sy = np.sin(rpy)
    cr, cp, cy = np.cos(rpy)
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = xyz
    return result


def numbers(value):
    return [float(part) for part in value.split()]


def origin(element):
    if element is None:
        return np.eye(4)
    return transform(numbers(element.get("xyz", "0 0 0")), numbers(element.get("rpy", "0 0 0")))


def pose_text(matrix, lift):
    rotation = matrix[:3, :3]
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(np.cos(pitch)) < 1e-9:
        roll = 0.0
        yaw = np.arctan2(-rotation[0, 1], rotation[1, 1])
    else:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    x, y, z = matrix[:3, 3]
    return " ".join(f"{value:.10g}" for value in (x, y, z + lift, roll, pitch, yaw))


def stl_vertices(path):
    data = path.read_bytes()
    triangles = unpack_from("<I", data, 80)[0]
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(data, dtype=dtype, count=triangles, offset=84)["vertices"].reshape(-1, 3)


def prepare_subject():
    name = "human_subject_with_meshes"
    directory = MODELS / name
    source = f"{SUBJECT}/humanSubjectWithMeshes"
    name = "humanSubjectWithMesh.urdf"
    urdf = ET.parse(fetch(directory / name, f"{source}/{name}"))
    fetch(directory / "LICENSE", f"{SUBJECT}/LICENSE")
    fetch(directory / "UPSTREAM-README.md", f"{SUBJECT}/README.md")
    fetch(directory / "MODEL-README.md", f"{source}/README.md")

    children = {joint.find("child").get("link"): joint for joint in urdf.findall("joint")}

    # Freeze the shoulders 15 degrees from vertical so the anatomical model stands naturally.
    shoulders = {"jRightShoulder_rotx": 5 * pi / 12, "jLeftShoulder_rotx": -5 * pi / 12}

    @lru_cache(maxsize=None)
    def standing_pose(link_name):
        joint = children.get(link_name)
        if joint is None:
            return np.eye(4)
        parent_pose = standing_pose(joint.find("parent").get("link"))
        arm_rotation = transform([0, 0, 0], [shoulders.get(joint.get("name"), 0), 0, 0])
        return parent_pose @ origin(joint.find("origin")) @ arm_rotation

    visuals = []
    for link in urdf.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            matrix = standing_pose(link.get("name")) @ origin(visual.find("origin"))
            filename = Path(mesh.get("filename")).name
            color = visual.find("material/color").get("rgba").strip()
            visuals.append(
                (visual.get("name"), filename, matrix, mesh.get("scale", "1 1 1"), color))

    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(lambda filename: fetch(directory / "meshes" / filename,
                                             f"{source}/meshes/{filename}"),
                      sorted({item[1] for item in visuals})))

    all_points = []
    for _, filename, matrix, scale, _ in visuals:
        points = stl_vertices(directory / "meshes" / filename) * numbers(scale)
        all_points.append(points @ matrix[:3, :3].T + matrix[:3, 3])
    points = np.concatenate(all_points)
    bounds = np.array([points.min(axis=0), points.max(axis=0)])
    print(f"{name}: dimensions xyz={np.round(np.ptp(bounds, axis=0), 4)} m; "
          f"ground offset={-bounds[0, 2]:.4f} m; {len(visuals)} visuals")

    sdf, link = visual_model(name)
    for label, filename, matrix, scale, color in visuals:
        visual = ET.SubElement(link, "visual", name=label)
        ET.SubElement(visual, "pose").text = pose_text(matrix, -bounds[0, 2])
        mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = f"model://{name}/meshes/{filename}"
        ET.SubElement(mesh, "scale").text = scale
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = color
        ET.SubElement(material, "diffuse").text = color
    write_xml(sdf, directory / "model.sdf")
    config = ET.Element("model")
    ET.SubElement(config, "name").text = name
    ET.SubElement(config, "version").text = "1.0"
    ET.SubElement(config, "sdf", version="1.6").text = "model.sdf"
    write_xml(config, directory / "model.config")
    return sdf.find("model")


def write_world(robot, subject, name):
    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world", name="vision_lab")
    ET.SubElement(ET.SubElement(world, "include"), "uri").text = "model://ground_plane"
    light = ET.SubElement(world, "light", name="key_light", type="directional")
    ET.SubElement(light, "direction").text = "0.55 -0.2 -0.82"
    ET.SubElement(light, "diffuse").text = "0.75 0.75 0.75 1"
    ET.SubElement(light, "specular").text = "0.15 0.15 0.15 1"
    ET.SubElement(light, "cast_shadows").text = "true"
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.52 0.52 0.52 1"
    ET.SubElement(scene, "background").text = "0.55 0.65 0.78 1"
    ET.SubElement(scene, "shadows").text = "true"
    plugin = ET.SubElement(world, "plugin", name="visual_state", filename="libgazebo_ros_state.so")
    ET.SubElement(ET.SubElement(plugin, "ros"), "namespace").text = "/visual_prototype"
    ET.SubElement(plugin, "update_rate").text = "15"
    world.append(deepcopy(robot))
    subject.set("name", "pedestrian")
    ET.SubElement(subject, "pose").text = f"{CAMERA_X + 2} {CAMERA_Y} 0 0 0 {FRONT_YAW[name]}"
    world.append(subject)
    write_xml(sdf, CACHE / f"{name}.world")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-only", action="store_true")
    args = parser.parse_args()
    robot = prepare_robot()
    if not args.robot_only:
        write_world(robot, prepare_gems(), "gems")
        write_world(robot, prepare_subject(), "subject")


if __name__ == "__main__":
    main()
