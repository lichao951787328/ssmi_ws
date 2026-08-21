#!/usr/bin/env python3
"""Generate an isolated grass / semantic-omni scenario from existing assets.

The source world and robot model are read-only inputs.  All changes are written
to new files so the original single-camera scenario remains reproducible.
"""

import argparse
import math
import os
import random
import xml.etree.ElementTree as ET


DIRECTIONS = (
    ("front", 0.0),
    ("left", math.pi / 2.0),
    ("rear", math.pi),
    ("right", -math.pi / 2.0),
)


def indent_xml(node, level=0):
    whitespace = "\n" + level * "  "
    child_whitespace = "\n" + (level + 1) * "  "
    if len(node):
        if not node.text or not node.text.strip():
            node.text = child_whitespace
        for child in node:
            indent_xml(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = child_whitespace
        node[-1].tail = whitespace


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def parse_with_comments(path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser=parser)


def add_camera_pair(base_link, model, direction, yaw):
    frame_name = "camera_mount" if direction == "front" else "camera_mount_{}".format(direction)
    rgbd_name = "rgbd_camera" if direction == "front" else "rgbd_{}_camera".format(direction)
    semantic_name = (
        "semantic_segmentation_camera"
        if direction == "front"
        else "semantic_{}_camera".format(direction)
    )
    rgbd_topic = "rgbd_camera" if direction == "front" else "rgbd_{}".format(direction)
    semantic_topic = "semantic" if direction == "front" else "semantic_{}".format(direction)

    frame = element(model, "frame", name=frame_name, attached_to=base_link.get("name"))
    # A top-centred ring approximates a spinning lidar origin.  A slight
    # downward pitch retains enough ground observations for terrain mapping.
    element(
        frame,
        "pose",
        "0 0 0.4 0 0.30 {:.12f}".format(yaw),
        relative_to=base_link.get("name"),
    )

    for sensor_name, sensor_type, topic in (
        (rgbd_name, "rgbd_camera", rgbd_topic),
        (semantic_name, "segmentation", semantic_topic),
    ):
        sensor = element(base_link, "sensor", name=sensor_name, type=sensor_type)
        element(sensor, "pose", "0 0 0 0 0 0", relative_to=frame_name)
        if sensor_type == "segmentation":
            element(sensor, "topic", topic)
        camera = element(sensor, "camera")
        if sensor_type == "segmentation":
            element(camera, "segmentation_type", "semantic")
        # 100 degrees gives 10 degrees overlap between adjacent cameras and
        # avoids blind seams around the nominal 360-degree ring.
        element(camera, "horizontal_fov", "1.745329252")
        image = element(camera, "image")
        element(image, "width", "480")
        element(image, "height", "270")
        clip = element(camera, "clip")
        element(clip, "near", "0.10")
        element(clip, "far", "30.0")
        lens = element(camera, "lens")
        intrinsics = element(lens, "intrinsics")
        # 480 / (2*tan(50 deg)) = 201.384...
        for name, value in (
            ("fx", "201.384"),
            ("fy", "201.384"),
            ("cx", "240"),
            ("cy", "135"),
            ("skew", "0"),
        ):
            element(intrinsics, name, value)
        element(sensor, "always_on", "1")
        element(sensor, "update_rate", "10")
        element(sensor, "visualize", "false")
        if sensor_type == "rgbd_camera":
            element(sensor, "topic", topic)


def generate_robot(source_path, output_path):
    tree = parse_with_comments(source_path)
    root = tree.getroot()
    model = root if root.tag == "model" else root.find("model")
    if model is None:
        raise RuntimeError("source robot SDF contains no model")
    model.set("name", "go2_semantic_omni")
    base_link = model.find("link[@name='base']")
    if base_link is None:
        raise RuntimeError("source robot SDF contains no base link")

    camera_sensor_names = {
        "rgbd_camera",
        "semantic_segmentation_camera",
    }
    for sensor in list(base_link.findall("sensor")):
        if sensor.get("name") in camera_sensor_names:
            base_link.remove(sensor)
    for frame in list(model.findall("frame")):
        if frame.get("name") == "camera_mount":
            model.remove(frame)

    # The derived model lives in another directory.  Resolve meshes through
    # the untouched source model instead of copying them.
    for uri in model.findall(".//mesh/uri"):
        if uri.text and uri.text.startswith("meshes/"):
            uri.text = "model://go2_semantic/{}".format(uri.text)

    for direction, yaw in DIRECTIONS:
        add_camera_pair(base_link, model, direction, yaw)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    indent_xml(root)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def add_grass_model(world):
    grass = ET.Element("model", {"name": "irregular_grass_islands"})
    element(grass, "static", "true")
    link = element(grass, "link", name="grass_visuals")
    rng = random.Random(20260818)
    # Several separated islands made from overlapping, deterministic random
    # primitives.  They have no collision: the original ground plane remains
    # the physical walking surface.
    island_centres = (
        (-20.0, 21.0),
        (10.0, 15.0),
        (24.0, -18.0),
        (-23.0, -25.0),
        (29.0, 29.0),
    )
    visual_index = 0
    for centre_x, centre_y in island_centres:
        for _ in range(9):
            visual = element(link, "visual", name="grass_{:02d}".format(visual_index))
            visual_index += 1
            x = centre_x + rng.uniform(-3.2, 3.2)
            y = centre_y + rng.uniform(-2.5, 2.5)
            yaw = rng.uniform(-math.pi, math.pi)
            element(visual, "pose", "{:.3f} {:.3f} 0.008 0 0 {:.5f}".format(x, y, yaw))
            geometry = element(visual, "geometry")
            if rng.random() < 0.55:
                box = element(geometry, "box")
                element(
                    box,
                    "size",
                    "{:.3f} {:.3f} 0.016".format(
                        rng.uniform(1.5, 4.0), rng.uniform(0.8, 2.5)
                    ),
                )
            else:
                cylinder = element(geometry, "cylinder")
                element(cylinder, "radius", "{:.3f}".format(rng.uniform(0.8, 2.0)))
                element(cylinder, "length", "0.016")
            material = element(visual, "material")
            green = rng.uniform(0.28, 0.55)
            element(material, "ambient", "0.06 {:.3f} 0.05 1".format(green))
            element(material, "diffuse", "0.08 {:.3f} 0.06 1".format(green + 0.10))
            element(material, "specular", "0.01 0.02 0.01 1")

    label_plugin = element(
        grass,
        "plugin",
        filename="ignition-gazebo-label-system",
        name="ignition::gazebo::systems::Label",
    )
    element(label_plugin, "label", "8")
    return grass


def generate_world(source_path, output_path):
    tree = parse_with_comments(source_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("source SDF contains no world")

    # Obstacles 7 and 8 retain their periodic motion systems but are perceived
    # as class 5 (static wall).  This deliberately simulates semantic false
    # negatives rather than physically stopping the models.
    for model_name in ("dynamic_obstacle_7", "dynamic_obstacle_8"):
        model = world.find("model[@name='{}']".format(model_name))
        if model is None:
            raise RuntimeError("source world is missing {}".format(model_name))
        label = model.find("plugin[@filename='ignition-gazebo-label-system']/label")
        if label is None:
            raise RuntimeError("{} has no semantic label plugin".format(model_name))
        label.text = "5"

    robot_include = None
    for include in world.findall("include"):
        uri = include.find("uri")
        if uri is not None and uri.text == "model://go2_semantic":
            robot_include = include
            break
    if robot_include is None:
        raise RuntimeError("source world has no go2_semantic include")
    robot_include.find("uri").text = "model://go2_semantic_omni"
    robot_include.find("name").text = "go2_semantic_omni"

    insert_at = list(world).index(robot_include)
    world.insert(
        insert_at,
        ET.Comment(
            " Label 8 grass: visual-only irregular islands on the original ground collision. "
        ),
    )
    world.insert(insert_at + 1, add_grass_model(world))

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    indent_xml(root)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-world", required=True)
    parser.add_argument("--source-model", required=True)
    parser.add_argument("--output-world", required=True)
    parser.add_argument("--output-model", required=True)
    args = parser.parse_args()
    generate_robot(args.source_model, args.output_model)
    generate_world(args.source_world, args.output_world)


if __name__ == "__main__":
    main()
