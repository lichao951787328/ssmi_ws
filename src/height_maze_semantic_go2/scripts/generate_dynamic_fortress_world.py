#!/usr/bin/env python3
"""Create a dynamic-obstacle variant of the generated Fortress height maze.

The base height_maze_semantic_go2.sdf remains unchanged.  Six collision-enabled
models are added to a separate world and move deterministically with the same
PeriodicMotionSystem used by semantic_segmentation_go2.
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_BASE_WORLD = PACKAGE_DIR / "worlds" / "height_maze_semantic_go2.sdf"
DEFAULT_OUTPUT_WORLD = PACKAGE_DIR / "worlds" / "height_maze_semantic_go2_dynamic.sdf"
HEIGHTMAP_PATH = PACKAGE_DIR / "worlds" / "assets" / "quad_height.png"
HEIGHTMAP_SIZE_X = 51.2
HEIGHTMAP_SIZE_Y = 51.2
HEIGHTMAP_SIZE_Z = 2.5
BASE_GROUND_TOP = 0.015
CONTACT_CLEARANCE = 0.005


DYNAMIC_OBSTACLES = (
    {
        "name": "dynamic_spawn_box",
        "start": (-5.0, -14.0),
        "end": (3.0, -14.0),
        "kind": "box",
        "dims": (0.75, 0.75, 1.05),
        "color": (0.96, 0.12, 0.64, 1.0),
        "period": 22.0,
        "phase": 0.0,
    },
    {
        "name": "dynamic_southwest_long_bar",
        "start": (-22.0, -14.0),
        "end": (-10.0, -14.0),
        "kind": "box",
        "dims": (1.80, 0.35, 0.60),
        "color": (0.86, 0.16, 0.72, 1.0),
        "period": 34.0,
        "phase": 8.0,
    },
    {
        "name": "dynamic_west_cylinder",
        "start": (-25.5, 2.0),
        "end": (-25.5, 10.0),
        "kind": "cylinder",
        "dims": (0.38, 1.15),
        "color": (0.92, 0.20, 0.70, 1.0),
        "period": 26.0,
        "phase": 4.0,
    },
    {
        "name": "dynamic_east_box",
        "start": (24.0, -12.0),
        "end": (24.0, 2.0),
        "kind": "box",
        "dims": (0.72, 0.72, 1.00),
        "color": (0.98, 0.18, 0.58, 1.0),
        "period": 32.0,
        "phase": 13.0,
    },
    {
        "name": "dynamic_north_sphere",
        "start": (6.0, 14.0),
        "end": (10.0, 14.0),
        "kind": "sphere",
        "dims": (0.50,),
        "color": (0.80, 0.10, 0.82, 1.0),
        "period": 18.0,
        "phase": 3.0,
    },
    {
        "name": "dynamic_southeast_cylinder",
        "start": (6.0, -10.0),
        "end": (10.0, -10.0),
        "kind": "cylinder",
        "dims": (0.40, 1.20),
        "color": (0.90, 0.14, 0.76, 1.0),
        "period": 20.0,
        "phase": 9.0,
    },
)


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, {key: str(value) for key, value in attributes.items()})
    if text is not None:
        node.text = str(text)
    return node


def indent_xml(node, level=0):
    """Pretty-print an ElementTree on Python 3.8."""
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


def add_pose(parent, values):
    element(parent, "pose", " ".join("{:.6f}".format(float(value)) for value in values))


def add_geometry(parent, kind, dims):
    geometry = element(parent, "geometry")
    shape = element(geometry, kind)
    if kind == "box":
        element(shape, "size", " ".join("{:.6f}".format(float(value)) for value in dims))
    elif kind == "cylinder":
        element(shape, "radius", "{:.6f}".format(float(dims[0])))
        element(shape, "length", "{:.6f}".format(float(dims[1])))
    elif kind == "sphere":
        element(shape, "radius", "{:.6f}".format(float(dims[0])))
    else:
        raise ValueError("Unsupported dynamic obstacle primitive: {}".format(kind))


def add_friction(collision):
    surface = element(collision, "surface")
    friction = element(surface, "friction")
    ode = element(friction, "ode")
    element(ode, "mu", 1.1)
    element(ode, "mu2", 1.1)


def add_material(visual, color):
    rgba = " ".join(str(value) for value in color)
    material = element(visual, "material")
    element(material, "ambient", rgba)
    element(material, "diffuse", rgba)
    element(material, "specular", "0.04 0.04 0.04 1")


def add_label(model, label):
    plugin = element(
        model,
        "plugin",
        filename="ignition-gazebo-label-system",
        name="ignition::gazebo::systems::Label",
    )
    element(plugin, "label", int(label))


def bilinear_height(heightmap, x, y):
    """Sample the Fortress heightmap using its world coordinate convention."""
    rows, columns = heightmap.shape
    pixel_x = np.clip((x / HEIGHTMAP_SIZE_X + 0.5) * (columns - 1), 0.0, columns - 1)
    # The first image row is the positive world-Y edge.
    pixel_y = np.clip((0.5 - y / HEIGHTMAP_SIZE_Y) * (rows - 1), 0.0, rows - 1)
    x0 = int(math.floor(pixel_x))
    y0 = int(math.floor(pixel_y))
    x1 = min(x0 + 1, columns - 1)
    y1 = min(y0 + 1, rows - 1)
    dx = pixel_x - x0
    dy = pixel_y - y0
    gray = (
        heightmap[y0, x0] * (1.0 - dx) * (1.0 - dy)
        + heightmap[y0, x1] * dx * (1.0 - dy)
        + heightmap[y1, x0] * (1.0 - dx) * dy
        + heightmap[y1, x1] * dx * dy
    )
    return float(gray) / 255.0 * HEIGHTMAP_SIZE_Z


def terrain_surface_height(heightmap, x, y):
    # The thin blue ground occupies z=[-0.015, 0.015] below the heightmap.
    return max(BASE_GROUND_TOP, bilinear_height(heightmap, x, y))


def primitive_half_height(kind, dims):
    if kind == "box":
        return dims[2] / 2.0
    if kind == "cylinder":
        return dims[1] / 2.0
    if kind == "sphere":
        return dims[0]
    raise ValueError("Unsupported dynamic obstacle primitive: {}".format(kind))


def trajectory_heights(heightmap, start, end):
    """Return endpoint heights and a small lift preventing terrain penetration."""
    start_height = terrain_surface_height(heightmap, *start)
    end_height = terrain_surface_height(heightmap, *end)
    maximum_residual = 0.0
    maximum_absolute_error = 0.0
    for progress in np.linspace(0.0, 1.0, 201):
        x = start[0] + (end[0] - start[0]) * progress
        y = start[1] + (end[1] - start[1]) * progress
        surface = terrain_surface_height(heightmap, x, y)
        linear_surface = start_height + (end_height - start_height) * progress
        maximum_residual = max(maximum_residual, surface - linear_surface)
        maximum_absolute_error = max(maximum_absolute_error, abs(surface - linear_surface))
    return start_height, end_height, maximum_residual + CONTACT_CLEARANCE, maximum_absolute_error


def add_dynamic_obstacle(world, heightmap, specification):
    name = specification["name"]
    start = specification["start"]
    end = specification["end"]
    kind = specification["kind"]
    dims = specification["dims"]
    half_height = primitive_half_height(kind, dims)
    surface0, surface1, lift, track_error = trajectory_heights(heightmap, start, end)
    z0 = surface0 + half_height + lift
    z1 = surface1 + half_height + lift

    model = ET.Element("model", {"name": name})
    element(model, "static", "true")
    add_pose(model, (start[0], start[1], z0, 0.0, 0.0, 0.0))
    link = element(model, "link", name="body")
    collision = element(link, "collision", name="body_collision")
    add_geometry(collision, kind, dims)
    add_friction(collision)
    visual = element(link, "visual", name="body_visual")
    add_geometry(visual, kind, dims)
    add_material(visual, specification["color"])
    add_label(model, 6)

    plugin = element(
        model,
        "plugin",
        filename="libssmi_periodic_motion_system.so",
        name="semantic_segmentation_husky::PeriodicMotionSystem",
    )
    element(
        plugin,
        "offset",
        "{:.6f} {:.6f} {:.6f}".format(
            end[0] - start[0], end[1] - start[1], z1 - z0
        ),
    )
    element(plugin, "period", "{:.6f}".format(specification["period"]))
    element(plugin, "phase", "{:.6f}".format(specification["phase"]))
    return model, track_error, lift


def generate_dynamic_world(base_world, output_world):
    if not base_world.is_file():
        raise FileNotFoundError("Base Fortress world does not exist: {}".format(base_world))
    if not HEIGHTMAP_PATH.is_file():
        raise FileNotFoundError("Heightmap asset does not exist: {}".format(HEIGHTMAP_PATH))

    tree = ET.parse(base_world)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("Base SDF does not contain a world")
    existing_names = {model.get("name") for model in world.findall("model")}
    collisions = existing_names.intersection(item["name"] for item in DYNAMIC_OBSTACLES)
    if collisions:
        raise ValueError("Base world already contains dynamic models: {}".format(sorted(collisions)))

    heightmap = np.asarray(Image.open(HEIGHTMAP_PATH).convert("L"), dtype=np.float64)
    include_index = next(
        (index for index, child in enumerate(list(world)) if child.tag == "include"),
        len(world),
    )
    reports = []
    for specification in DYNAMIC_OBSTACLES:
        model, track_error, lift = add_dynamic_obstacle(world, heightmap, specification)
        world.insert(include_index, model)
        include_index += 1
        reports.append((specification["name"], track_error, lift))

    indent_xml(root)
    output_world.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_world, encoding="utf-8", xml_declaration=True)
    print("Generated {}".format(output_world))
    print("Added {} collision-enabled semantic ID 6 obstacles".format(len(reports)))
    for name, track_error, lift in reports:
        print("  {:32s} terrain linearization error={:.4f} m, lift={:.4f} m".format(
            name, track_error, lift
        ))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-world", type=Path, default=DEFAULT_BASE_WORLD)
    parser.add_argument("--output-world", type=Path, default=DEFAULT_OUTPUT_WORLD)
    arguments = parser.parse_args()
    generate_dynamic_world(arguments.base_world.resolve(), arguments.output_world.resolve())


if __name__ == "__main__":
    main()
