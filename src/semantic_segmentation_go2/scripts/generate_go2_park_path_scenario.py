#!/usr/bin/env python3
"""Derive a large two-lawn park with a curved path from the omni world."""

import argparse
import math
import os
import random
import xml.etree.ElementTree as ET


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


def add_material(visual, ambient, diffuse):
    material = element(visual, "material")
    element(material, "ambient", ambient)
    element(material, "diffuse", diffuse)
    element(material, "specular", "0.01 0.01 0.01 1")


def replace_first_island_with_large_park(world):
    grass = world.find("model[@name='irregular_grass_islands']")
    if grass is None:
        raise RuntimeError("source world has no irregular_grass_islands model")
    link = grass.find("link[@name='grass_visuals']")
    if link is None:
        raise RuntimeError("grass model has no grass_visuals link")

    # The first nine deterministic primitives form the old small island near
    # (-20, 21).  Replace only that island and preserve the other four.
    old_names = {"grass_{:02d}".format(index) for index in range(9)}
    removed = 0
    for visual in list(link.findall("visual")):
        if visual.get("name") in old_names:
            link.remove(visual)
            removed += 1
    if removed != len(old_names):
        raise RuntimeError("expected to replace 9 grass primitives, found {}".format(removed))

    # A 26 x 19 m lawn occupies the large north-west maze compartment.  The
    # path added below visually and semantically divides it into two lawns.
    base = element(link, "visual", name="park_lawn_base")
    element(base, "pose", "-20 23.5 0.008 0 0 0")
    geometry = element(base, "geometry")
    box = element(geometry, "box")
    element(box, "size", "26 19 0.016")
    add_material(base, "0.04 0.38 0.04 1", "0.06 0.50 0.06 1")

    # Uneven perimeter lobes keep the park from looking like a perfect box.
    rng = random.Random(20260819)
    perimeter = []
    for index in range(7):
        perimeter.append((-32.8, 15.0 + index * 2.8 + rng.uniform(-0.35, 0.35)))
        perimeter.append((-7.2, 15.0 + index * 2.8 + rng.uniform(-0.35, 0.35)))
    for index in range(7):
        perimeter.append((-31.0 + index * 3.65 + rng.uniform(-0.35, 0.35), 14.1))
        perimeter.append((-31.0 + index * 3.65 + rng.uniform(-0.35, 0.35), 32.9))
    for index, (x, y) in enumerate(perimeter):
        visual = element(link, "visual", name="park_edge_{:02d}".format(index))
        element(visual, "pose", "{:.3f} {:.3f} 0.008 0 0 0".format(x, y))
        geometry = element(visual, "geometry")
        cylinder = element(geometry, "cylinder")
        element(cylinder, "radius", "{:.3f}".format(rng.uniform(1.15, 1.75)))
        element(cylinder, "length", "0.016")
        green = rng.uniform(0.36, 0.50)
        add_material(
            visual,
            "0.04 {:.3f} 0.04 1".format(green),
            "0.06 {:.3f} 0.06 1".format(green + 0.08),
        )


def curved_path_points(count=33):
    """Return the park path rotated 90 degrees to run west-to-east."""
    points = []
    for index in range(count):
        t = float(index) / float(count - 1)
        # Extend one metre beyond each lawn edge so the path fully separates
        # the north and south lawns.  The former x deflection becomes the y
        # deflection after the 90-degree rotation around the park centre.
        x = -34.0 + 28.0 * t
        y = 22.5 + 4.0 * math.sin(2.0 * math.pi * t - math.pi / 2.0) + 4.0 * t
        points.append((x, y))
    return points


def add_curved_path(world):
    path_model = ET.Element("model", {"name": "curved_park_path"})
    element(path_model, "static", "true")
    link = element(path_model, "link", name="path_visuals")
    points = curved_path_points()
    path_width = 2.4

    # Overlapping tangent boxes form the body of the path.  Round joints hide
    # chord corners and make the border read as a continuous curve.
    for index, ((x0, y0), (x1, y1)) in enumerate(zip(points[:-1], points[1:])):
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy) + 0.28
        yaw = math.atan2(dy, dx)
        visual = element(link, "visual", name="path_segment_{:02d}".format(index))
        element(
            visual,
            "pose",
            "{:.4f} {:.4f} 0.022 0 0 {:.7f}".format(
                0.5 * (x0 + x1), 0.5 * (y0 + y1), yaw
            ),
        )
        geometry = element(visual, "geometry")
        box = element(geometry, "box")
        element(box, "size", "{:.4f} {:.3f} 0.012".format(length, path_width))
        add_material(visual, "0.46 0.39 0.27 1", "0.68 0.58 0.40 1")

    for index, (x, y) in enumerate(points):
        visual = element(link, "visual", name="path_joint_{:02d}".format(index))
        element(visual, "pose", "{:.4f} {:.4f} 0.022 0 0 0".format(x, y))
        geometry = element(visual, "geometry")
        cylinder = element(geometry, "cylinder")
        element(cylinder, "radius", "{:.3f}".format(0.5 * path_width))
        element(cylinder, "length", "0.012")
        add_material(visual, "0.46 0.39 0.27 1", "0.68 0.58 0.40 1")

    plugin = element(
        path_model,
        "plugin",
        filename="ignition-gazebo-label-system",
        name="ignition::gazebo::systems::Label",
    )
    # Label 0 is the existing ground/free semantic class.  The visuals have no
    # collision, so the original flat ground remains the physical surface.
    element(plugin, "label", "0")

    grass = world.find("model[@name='irregular_grass_islands']")
    insert_at = list(world).index(grass) + 1
    world.insert(
        insert_at,
        ET.Comment(" Label 0 curved park path over the large label 8 lawn. "),
    )
    world.insert(insert_at + 1, path_model)


def generate(source_path, output_path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source_path, parser=parser)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("source SDF contains no world")
    replace_first_island_with_large_park(world)
    add_curved_path(world)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    indent_xml(root)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-world", required=True)
    parser.add_argument("--output-world", required=True)
    args = parser.parse_args()
    generate(args.source_world, args.output_world)


if __name__ == "__main__":
    main()
