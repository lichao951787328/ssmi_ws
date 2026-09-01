#!/usr/bin/env python3
"""Generate a deterministic multi-terrain semantic world for Gazebo Fortress."""

import argparse
import math
from pathlib import Path
from xml.dom import minidom
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageFilter


SEED = 20260825
WORLD_SIZE = 70.0
MAP_PIXELS = 257
HEIGHT_OFFSET = -1.0
HEIGHT_SCALE = 5.0


def terrain_height(x, y):
    """Continuous terrain containing flat, rolling, medium and steep zones."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    h = 0.10 * np.sin(2.0 * math.pi * (x + 2.0) / 21.0)
    h += 0.08 * np.cos(2.0 * math.pi * (y - 4.0) / 17.0)
    h += 0.06 * np.sin(2.0 * math.pi * (x + y) / 13.0)
    h += 0.16 * x / 35.0

    features = (
        (-25.0, 18.0, 1.75, 8.0, 5.5),
        (-18.0, -20.0, 1.55, 5.5, 7.0),
        (20.0, 19.0, 2.10, 5.5, 4.5),
        (24.0, -17.0, 1.60, 7.0, 4.0),
        (-5.0, 25.0, -0.72, 6.5, 4.0),
        (8.0, -18.0, -0.62, 4.5, 7.0),
        (29.0, 2.0, 1.40, 3.5, 8.0),
        (-29.0, -1.0, 1.10, 3.2, 6.0),
    )
    for cx, cy, amplitude, sx, sy in features:
        h += amplitude * np.exp(
            -0.5 * (((x - cx) / sx) ** 2 + ((y - cy) / sy) ** 2)
        )

    # Local medium-frequency undulation makes the southwest and northeast
    # sectors more demanding while retaining smooth, continuous collision.
    sw_mask = np.exp(-0.5 * (((x + 21.0) / 8.0) ** 2 + ((y + 18.0) / 8.0) ** 2))
    ne_mask = np.exp(-0.5 * (((x - 21.0) / 9.0) ** 2 + ((y - 17.0) / 8.0) ** 2))
    h += sw_mask * 0.18 * np.sin(2.0 * math.pi * x / 5.5)
    h += ne_mask * 0.22 * np.sin(2.0 * math.pi * (x + y) / 6.0)

    # Rotated elongated ridges add diagonal faces, so the slope directions are
    # not limited to the world X / Y axes.
    diagonal_u = (x + y) / math.sqrt(2.0)
    diagonal_v = (x - y) / math.sqrt(2.0)
    h += 1.15 * np.exp(-0.5 * (((diagonal_u - 24.0) / 5.0) ** 2 +
                               ((diagonal_v + 3.0) / 13.0) ** 2))
    h -= 0.58 * np.exp(-0.5 * (((diagonal_u + 22.0) / 4.0) ** 2 +
                               ((diagonal_v - 6.0) / 12.0) ** 2))

    # Small local hill immediately southwest (lower-left in the top view) of
    # the robot spawn. The elongated diagonal footprint provides a clear but
    # still traversable approach without disturbing the flat spawn disk.
    local_u = (x + y + 14.0) / math.sqrt(2.0)
    local_v = (x - y) / math.sqrt(2.0)
    h += 0.62 * np.exp(-0.5 * ((local_u / 3.2) ** 2 + (local_v / 4.2) ** 2))

    # A mesa in the north provides a real hilltop for the staircase. Its west
    # face is intentionally steeper; the other three sides create additional
    # +X, -X, +Y and -Y slope directions around the summit.
    mesa_radius = (
        np.abs((x - 6.25) / 8.5) ** 4 + np.abs((y - 15.5) / 7.5) ** 4
    ) ** 0.25
    mesa = 0.5 * (1.0 - np.tanh((mesa_radius - 0.88) / 0.18))
    h += 1.70 * mesa

    # Flat spawn disk with a long blend, avoiding an artificial steep ring.
    radius = np.hypot(x, y)
    blend = np.clip((radius - 2.8) / 9.0, 0.0, 1.0)
    blend = blend * blend * (3.0 - 2.0 * blend)
    return np.clip(h * blend, HEIGHT_OFFSET + 0.03, HEIGHT_OFFSET + HEIGHT_SCALE - 0.03)


def height_and_tilt(x, y):
    eps = 0.08
    z = float(terrain_height(x, y))
    gx = float(terrain_height(x + eps, y) - terrain_height(x - eps, y)) / (2.0 * eps)
    gy = float(terrain_height(x, y + eps) - terrain_height(x, y - eps)) / (2.0 * eps)
    return z, math.atan(gy), -math.atan(gx)


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, {key: str(value) for key, value in attributes.items()})
    if text is not None:
        node.text = str(text)
    return node


def add_pose(parent, xyz, rpy=(0.0, 0.0, 0.0)):
    values = (*xyz, *rpy)
    element(parent, "pose", " ".join(f"{value:.6f}" for value in values))


def add_label(model, label):
    plugin = element(
        model,
        "plugin",
        filename="ignition-gazebo-label-system",
        name="ignition::gazebo::systems::Label",
    )
    element(plugin, "label", label)


def add_geometry(parent, kind, dims):
    geometry = element(parent, "geometry")
    shape = element(geometry, kind)
    if kind == "box":
        element(shape, "size", " ".join(str(value) for value in dims))
    elif kind == "cylinder":
        element(shape, "radius", dims[0])
        element(shape, "length", dims[1])
    elif kind == "sphere":
        element(shape, "radius", dims[0])
    else:
        raise ValueError(f"unsupported primitive: {kind}")


def add_material(visual, color):
    material = element(visual, "material")
    rgba = " ".join(str(value) for value in (*color, 1.0))
    element(material, "ambient", rgba)
    element(material, "diffuse", rgba)
    element(material, "specular", "0.04 0.04 0.04 1")


def add_primitive(
    link,
    name,
    kind,
    dims,
    xyz,
    color,
    rpy=(0.0, 0.0, 0.0),
    collision=True,
):
    if collision:
        collision_node = element(link, "collision", name=f"{name}_collision")
        add_pose(collision_node, xyz, rpy)
        add_geometry(collision_node, kind, dims)
        surface = element(collision_node, "surface")
        friction = element(element(surface, "friction"), "ode")
        element(friction, "mu", "1.1")
        element(friction, "mu2", "1.1")

    visual = element(link, "visual", name=f"{name}_visual")
    add_pose(visual, xyz, rpy)
    add_geometry(visual, kind, dims)
    add_material(visual, color)


def add_model(world, name, label, static=True):
    model = element(world, "model", name=name)
    element(model, "static", str(static).lower())
    link = element(model, "link", name=f"{name}_link")
    add_label(model, label)
    return model, link


def add_systems(world):
    systems = (
        ("ignition-gazebo-physics-system", "ignition::gazebo::systems::Physics"),
        ("ignition-gazebo-user-commands-system", "ignition::gazebo::systems::UserCommands"),
        ("ignition-gazebo-scene-broadcaster-system", "ignition::gazebo::systems::SceneBroadcaster"),
    )
    for filename, name in systems:
        element(world, "plugin", filename=filename, name=name)
    sensors = element(
        world,
        "plugin",
        filename="ignition-gazebo-sensors-system",
        name="ignition::gazebo::systems::Sensors",
    )
    element(sensors, "render_engine", "ogre2")
    element(
        world,
        "plugin",
        filename="ignition-gazebo-imu-system",
        name="ignition::gazebo::systems::Imu",
    )


def add_environment(world):
    physics = element(world, "physics", name="1ms", type="ignored")
    element(physics, "max_step_size", "0.001")
    element(physics, "real_time_factor", "1.0")
    element(physics, "gravity", "0 0 -9.8")
    scene = element(world, "scene")
    element(scene, "ambient", "0.43 0.45 0.40 1")
    element(scene, "background", "0.60 0.73 0.86 1")
    element(scene, "shadows", "true")
    light = element(world, "light", type="directional", name="sun")
    element(light, "cast_shadows", "true")
    add_pose(light, (0.0, 0.0, 35.0))
    element(light, "diffuse", "0.88 0.84 0.74 1")
    element(light, "specular", "0.18 0.18 0.16 1")
    attenuation = element(light, "attenuation")
    element(attenuation, "range", "1000")
    element(attenuation, "constant", "0.9")
    element(attenuation, "linear", "0.01")
    element(attenuation, "quadratic", "0.001")
    element(light, "direction", "-0.45 0.25 -0.86")


def add_heightmap(world, heightmap_vertical_size):
    # Fortress' ODE heightfield collision and Ogre2 heightmap rendering use
    # different gray normalization and offset paths.  Keep them in separate
    # static models so each backend can be calibrated to the same world-space
    # terrain_height() surface without one backend moving the other.
    collision_model = element(
        world, "model", name="bare_earth_sloped_terrain_collision"
    )
    element(collision_model, "static", "true")
    add_pose(collision_model, (0.0, 0.0, HEIGHT_OFFSET))
    collision_link = element(collision_model, "link", name="terrain_collision_link")
    collision = element(collision_link, "collision", name="terrain_collision")
    geometry = element(collision, "geometry")
    heightmap = element(geometry, "heightmap")
    element(heightmap, "uri", "file://complex_terrain_heightmap.png")
    element(
        heightmap,
        "size",
        f"{WORLD_SIZE} {WORLD_SIZE} {heightmap_vertical_size:.6f}",
    )
    element(heightmap, "pos", "0 0 0")
    surface = element(collision, "surface")
    ode = element(element(surface, "friction"), "ode")
    element(ode, "mu", "1.15")
    element(ode, "mu2", "1.15")

    visual_model = element(world, "model", name="bare_earth_sloped_terrain")
    element(visual_model, "static", "true")
    visual_link = element(visual_model, "link", name="terrain_visual_link")
    visual = element(visual_link, "visual", name="terrain_visual")
    geometry = element(visual, "geometry")
    heightmap = element(geometry, "heightmap")
    element(heightmap, "use_terrain_paging", "false")
    texture = element(heightmap, "texture")
    element(texture, "diffuse", "file://complex_terrain_surface.png")
    element(texture, "normal", "file://complex_terrain_flat_normal.png")
    element(texture, "size", "5")
    element(heightmap, "uri", "file://complex_terrain_heightmap.png")
    element(heightmap, "size", f"{WORLD_SIZE} {WORLD_SIZE} {HEIGHT_SCALE}")
    element(heightmap, "pos", f"0 0 {HEIGHT_OFFSET}")
    add_label(visual_model, 5)


def add_boundary(world):
    _, link = add_model(world, "terrain_boundary", 4)
    color = (0.31, 0.28, 0.22)
    for name, xyz, size in (
        ("north", (0, 34.6, 2.0), (70, 0.5, 5.5)),
        ("south", (0, -34.6, 2.0), (70, 0.5, 5.5)),
        ("east", (34.6, 0, 2.0), (0.5, 70, 5.5)),
        ("west", (-34.6, 0, 2.0), (0.5, 70, 5.5)),
    ):
        add_primitive(link, name, "box", size, xyz, color)


def add_surface_tile(link, name, x, y, size, color, z_offset):
    z, roll, pitch = height_and_tilt(x, y)
    add_primitive(
        link,
        name,
        "box",
        (size, size, 0.018),
        (x, y, z + z_offset),
        color,
        (roll, pitch, 0.0),
        collision=False,
    )


def grass_region(x, y):
    """Return the union mask for the expanded grass and summit fields."""
    main_field = ((x + 16.0) / 19.0) ** 2 + ((y - 18.0) / 15.0) ** 2
    summit_field = ((x - 7.0) / 15.0) ** 2 + ((y - 25.0) / 8.5) ** 2
    return (main_field <= 1.0) | (summit_field <= 1.0)


def add_grass(world):
    _, link = add_model(world, "grass_terrain", 2)
    visual = element(link, "visual", name="height_conforming_grass_visual")
    geometry = element(visual, "geometry")
    mesh = element(geometry, "mesh")
    element(mesh, "uri", "file://complex_grass_surface.obj")
    element(mesh, "scale", "1 1 1")
    material = element(visual, "material")
    element(material, "ambient", "0.18 0.48 0.14 1")
    element(material, "diffuse", "0.18 0.48 0.14 1")
    element(material, "specular", "0.02 0.02 0.02 1")
    metal = element(element(material, "pbr"), "metal")
    element(metal, "albedo_map", "file://complex_grass_texture.png")
    element(metal, "roughness", "0.92")
    element(metal, "metalness", "0.0")


def add_gravel(world, rng):
    _, link = add_model(world, "gravel_terrain", 3)
    index = 0
    for x in np.arange(9.0, 31.0, 1.30):
        for y in np.arange(-29.0, -7.0, 1.30):
            ellipse = ((x - 20.0) / 11.5) ** 2 + ((y + 18.0) / 11.5) ** 2
            if ellipse <= 1.0:
                add_surface_tile(link, f"gravel_tile_{index}", x, y, 1.48, (0.48, 0.46, 0.41), 0.027)
                index += 1

    # Physical pebbles create small 2--11 cm disturbances in the gravel zone.
    for pebble in range(105):
        angle = rng.uniform(0.0, 2.0 * math.pi)
        radius_scale = math.sqrt(rng.uniform(0.0, 1.0))
        x = 20.0 + 10.2 * radius_scale * math.cos(angle)
        y = -18.0 + 10.2 * radius_scale * math.sin(angle)
        radius = rng.uniform(0.025, 0.105)
        z = float(terrain_height(x, y))
        add_primitive(
            link,
            f"pebble_{pebble}",
            "sphere",
            (radius,),
            (x, y, z + radius * 0.55),
            (0.39 + rng.uniform(-0.05, 0.05),) * 3,
        )


def interpolate_polyline(points, spacing=0.75):
    samples = []
    for start, end in zip(points[:-1], points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        count = max(2, int(math.hypot(dx, dy) / spacing) + 1)
        for step in range(count - 1):
            t = step / (count - 1)
            samples.append((start[0] + t * dx, start[1] + t * dy))
    samples.append(points[-1])
    return samples


def add_trail(world):
    _, link = add_model(world, "winding_dirt_trail", 1)
    points = [(-34, -5), (-27, -7), (-20, -5), (-13, -1), (-6, 1), (0, 0),
              (7, 4), (14, 3), (21, 8), (28, 7), (34, 10)]
    for index, (x, y) in enumerate(interpolate_polyline(points)):
        z, roll, pitch = height_and_tilt(x, y)
        add_primitive(
            link,
            f"trail_{index}",
            "cylinder",
            (1.15, 0.020),
            (x, y, z + 0.038),
            (0.40, 0.27, 0.12),
            (roll, pitch, 0.0),
            collision=False,
        )


def add_stairs(world):
    _, link = add_model(world, "terrain_stair_bridge", 7)
    x0, y0, yaw = -12.0, 15.0, 0.0
    tread, width, step_count = 0.55, 3.2, 21
    platform_length = 4.0
    platform_center_x = 1.5

    start_ground = float(terrain_height(x0, y0))
    platform_samples = [
        float(terrain_height(x, y))
        for x in np.linspace(platform_center_x - platform_length / 2.0,
                             platform_center_x + platform_length / 2.0, 21)
        for y in np.linspace(y0 - width / 2.0, y0 + width / 2.0, 9)
    ]
    platform_top = max(platform_samples) + 0.08
    riser = (platform_top - start_ground) / step_count

    for index in range(step_count):
        along = (index + 0.5) * tread
        x = x0 + along * math.cos(yaw)
        y = y0 + along * math.sin(yaw)
        ground = float(terrain_height(x, y))
        top = start_ground + (index + 1) * riser
        bottom = min(ground - 0.15, top - 0.10)
        add_primitive(
            link,
            f"step_{index}",
            "box",
            (tread, width, top - bottom),
            (x, y, 0.5 * (top + bottom)),
            (0.58, 0.50, 0.34),
            (0.0, 0.0, yaw),
        )

    platform_bottom = min(platform_samples) - 0.15
    add_primitive(
        link,
        "summit_platform",
        "box",
        (platform_length, width, platform_top - platform_bottom),
        (platform_center_x, y0, 0.5 * (platform_top + platform_bottom)),
        (0.52, 0.45, 0.30),
    )


def add_wall_run(link, name, start, end, wall_height=1.65, thickness=0.28):
    """Subdivide a maze wall so every piece sits on its local sloped ground."""
    x0, y0 = start
    x1, y1 = end
    length = math.hypot(x1 - x0, y1 - y0)
    count = max(1, int(math.ceil(length / 1.8)))
    segment_length = length / count + 0.04
    yaw = math.atan2(y1 - y0, x1 - x0)
    for index in range(count):
        t = (index + 0.5) / count
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = float(terrain_height(x, y))
        add_primitive(
            link,
            f"{name}_{index}",
            "box",
            (segment_length, thickness, wall_height),
            (x, y, z + wall_height / 2.0 - 0.03),
            (0.48, 0.40, 0.29),
            (0.0, 0.0, yaw),
        )


def add_maze_corridors(world):
    _, link = add_model(world, "hillside_maze_corridor_walls", 13)
    wall_runs = (
        ("west_a", (-27, -6), (-27, 10)),
        ("west_b", (-27, 10), (-18, 10)),
        ("west_c", (-18, 10), (-18, 25)),
        ("west_d", (-31, 26), (-22, 26)),
        ("south_a", (-29, -24), (-12, -24)),
        ("south_b", (-12, -24), (-12, -15)),
        ("south_c", (-12, -15), (-3, -15)),
        ("center_a", (-8, -9), (8, -9)),
        ("center_b", (8, -9), (8, -1)),
        ("center_c", (8, -1), (18, -1)),
        ("east_a", (22, 7), (22, 25)),
        ("east_b", (12, 7), (22, 7)),
        ("east_c", (12, 7), (12, 19)),
        ("north_a", (-2, 27), (12, 27)),
        ("north_b", (12, 27), (12, 32)),
        ("diag_a", (24, -7), (31, -1)),
        ("diag_b", (20, -28), (29, -22)),
    )
    for name, start, end in wall_runs:
        add_wall_run(link, name, start, end)


def add_static_obstacles(world, rng):
    # Long bars and fallen trunks: semantic ID 9.
    _, long_link = add_model(world, "long_static_obstacles", 9)
    for index, (x, y, length, yaw, cylinder) in enumerate((
        (-18, 3, 5.2, 0.30, True), (8, 13, 4.4, -0.65, True),
        (23, 3, 5.8, 0.90, False), (-25, -12, 6.0, -0.20, False),
        (2, -20, 4.8, 0.55, True),
    )):
        z = float(terrain_height(x, y))
        if cylinder:
            add_primitive(long_link, f"long_{index}", "cylinder", (0.22, length),
                          (x, y, z + 0.23), (0.34, 0.18, 0.07),
                          (0.0, math.pi / 2.0, yaw))
        else:
            add_primitive(long_link, f"long_{index}", "box", (length, 0.32, 0.42),
                          (x, y, z + 0.22), (0.72, 0.34, 0.08), (0.0, 0.0, yaw))

    # Rectangular crates / blocks: semantic ID 10.
    _, box_link = add_model(world, "rectangular_static_obstacles", 10)
    for index, (x, y, sx, sy, sz, yaw) in enumerate((
        (-11, -9, 1.2, 0.9, 1.1, 0.2), (4, 9, 1.8, 0.8, 0.8, -0.4),
        (16, 14, 0.9, 1.3, 1.4, 0.6), (-27, 25, 1.6, 1.1, 0.7, 0.1),
        (28, -2, 1.1, 1.1, 1.1, -0.2), (-5, -27, 2.0, 0.7, 0.6, 0.8),
    )):
        z = float(terrain_height(x, y))
        add_primitive(box_link, f"box_{index}", "box", (sx, sy, sz),
                      (x, y, z + sz / 2.0), (0.13, 0.31, 0.72), (0.0, 0.0, yaw))

    # Cylinders and spheres: semantic ID 11.
    _, round_link = add_model(world, "round_static_obstacles", 11)
    for index, (x, y, radius, height) in enumerate((
        (-4, 7, 0.45, 1.2), (12, -6, 0.55, 1.0), (-22, -26, 0.38, 1.4),
        (27, 22, 0.50, 1.1), (5, 27, 0.42, 1.3),
    )):
        z = float(terrain_height(x, y))
        add_primitive(round_link, f"cylinder_{index}", "cylinder", (radius, height),
                      (x, y, z + height / 2.0), (0.82, 0.68, 0.08))
    for index, (x, y, radius) in enumerate(((-15, 25, 0.68), (18, 26, 0.55), (30, 13, 0.75))):
        z = float(terrain_height(x, y))
        add_primitive(round_link, f"sphere_{index}", "sphere", (radius,),
                      (x, y, z + radius * 0.72), (0.78, 0.58, 0.07))

    # Natural boulders: semantic ID 8.
    _, rock_link = add_model(world, "natural_rocks", 8)
    rock_sites = [(-31, 14), (-14, 8), (10, 21), (22, 15), (-29, -20),
                  (-13, -18), (7, -11), (29, -25), (31, 27), (-3, 31)]
    for index, (x, y) in enumerate(rock_sites):
        radius = rng.uniform(0.45, 1.05)
        z = float(terrain_height(x, y))
        add_primitive(rock_link, f"rock_{index}", "sphere", (radius,),
                      (x, y, z + radius * 0.62), (0.30, 0.29, 0.27))

    # Trees and bushes: semantic ID 12.
    _, vegetation_link = add_model(world, "vegetation_obstacles", 12)
    for index, (x, y, height, radius) in enumerate((
        (-29, 28, 3.0, 0.27), (-18, 22, 3.4, 0.30), (-9, 29, 2.8, 0.24),
        (13, 29, 3.2, 0.28), (29, 18, 3.5, 0.31), (-31, -29, 3.1, 0.26),
        (2, -31, 2.9, 0.25), (31, -10, 3.3, 0.29),
    )):
        z = float(terrain_height(x, y))
        add_primitive(vegetation_link, f"trunk_{index}", "cylinder", (radius, height),
                      (x, y, z + height / 2.0), (0.29, 0.16, 0.065))
        add_primitive(vegetation_link, f"crown_{index}", "sphere", (1.05,),
                      (x, y, z + height + 0.55), (0.08, 0.31, 0.07), collision=False)


def add_dynamic_obstacle(world, name, start, end, kind, dims, color, period, phase):
    x0, y0 = start
    x1, y1 = end
    if kind == "box":
        half_height = dims[2] / 2.0
    elif kind == "cylinder":
        half_height = dims[1] / 2.0
    else:
        half_height = dims[0]
    z0 = float(terrain_height(x0, y0)) + half_height + 0.035
    z1 = float(terrain_height(x1, y1)) + half_height + 0.035
    model = element(world, "model", name=name)
    element(model, "static", "true")
    add_pose(model, (x0, y0, z0))
    link = element(model, "link", name="body")
    add_primitive(link, "body", kind, dims, (0.0, 0.0, 0.0), color)
    add_label(model, 6)
    plugin = element(
        model,
        "plugin",
        filename="libssmi_periodic_motion_system.so",
        name="semantic_segmentation_husky::PeriodicMotionSystem",
    )
    element(plugin, "offset", f"{x1 - x0:.4f} {y1 - y0:.4f} {z1 - z0:.4f}")
    element(plugin, "period", period)
    element(plugin, "phase", phase)


def add_dynamic_obstacles(world):
    add_dynamic_obstacle(world, "dynamic_box", (-8, -2), (7, -2), "box",
                         (0.8, 0.8, 1.1), (0.95, 0.12, 0.65), 28, 0)
    add_dynamic_obstacle(world, "dynamic_cylinder", (5, 6), (5, 14), "cylinder",
                         (0.42, 1.2), (0.92, 0.20, 0.72), 22, 5)
    add_dynamic_obstacle(world, "dynamic_sphere", (-16, -11), (-8, -11), "sphere",
                         (0.55,), (0.82, 0.12, 0.78), 18, 3)
    add_dynamic_obstacle(world, "dynamic_long_bar", (14, 0), (14, 8), "box",
                         (2.4, 0.35, 0.55), (0.88, 0.18, 0.62), 32, 8)
    add_dynamic_obstacle(world, "dynamic_grass_box", (-25, 11), (-14, 11), "box",
                         (0.7, 1.1, 1.0), (0.96, 0.16, 0.70), 38, 11)
    add_dynamic_obstacle(world, "dynamic_gravel_cylinder", (15, -20), (26, -20), "cylinder",
                         (0.48, 1.0), (0.90, 0.14, 0.68), 35, 7)


def add_robot(world):
    include = element(world, "include")
    element(include, "uri", "model://go2_semantic_omni")
    element(include, "name", "go2_semantic_omni")
    element(include, "pose", "0 0 0.34 0 0 0")


def generate_world(output_path, heightmap_vertical_size):
    rng = np.random.default_rng(SEED + 10)
    root = ET.Element("sdf", {"version": "1.8"})
    world = element(root, "world", name="semantic_segmentation_world")
    add_systems(world)
    add_environment(world)
    add_heightmap(world, heightmap_vertical_size)
    add_grass(world)
    add_gravel(world, rng)
    add_trail(world)
    add_stairs(world)
    add_maze_corridors(world)
    add_static_obstacles(world, rng)
    add_dynamic_obstacles(world)
    add_boundary(world)
    add_robot(world)
    rough = ET.tostring(root, encoding="utf-8")
    pretty = minidom.parseString(rough).toprettyxml(indent="  ", encoding="utf-8")
    output_path.write_bytes(pretty)


def generate_grass_mesh(output_dir, encoded_height):
    """Create grass triangles on the exact quantized heightmap sample grid."""
    axis = np.linspace(-WORLD_SIZE / 2.0, WORLD_SIZE / 2.0, MAP_PIXELS)
    quantized_height = HEIGHT_OFFSET + encoded_height.astype(np.float64) / 255.0 * HEIGHT_SCALE
    vertex_indices = {}
    vertices = []
    normals = []
    faces = []

    def vertex(row, column):
        key = (row, column)
        if key not in vertex_indices:
            vertex_indices[key] = len(vertices) + 1
            vertices.append(
                (axis[column], axis[row], quantized_height[row, column] + 0.015)
            )
            left = max(0, column - 1)
            right = min(MAP_PIXELS - 1, column + 1)
            lower = max(0, row - 1)
            upper = min(MAP_PIXELS - 1, row + 1)
            dz_dx = (quantized_height[row, right] - quantized_height[row, left]) / (
                axis[right] - axis[left]
            )
            dz_dy = (quantized_height[upper, column] - quantized_height[lower, column]) / (
                axis[upper] - axis[lower]
            )
            normal = np.asarray((-dz_dx, -dz_dy, 1.0), dtype=float)
            normal /= np.linalg.norm(normal)
            normals.append(tuple(normal))
        return vertex_indices[key]

    for row in range(MAP_PIXELS - 1):
        center_y = 0.5 * (axis[row] + axis[row + 1])
        for column in range(MAP_PIXELS - 1):
            center_x = 0.5 * (axis[column] + axis[column + 1])
            if not bool(grass_region(center_x, center_y)):
                continue
            lower_left = vertex(row, column)
            lower_right = vertex(row, column + 1)
            upper_right = vertex(row + 1, column + 1)
            upper_left = vertex(row + 1, column)
            faces.append((lower_left, lower_right, upper_right))
            faces.append((lower_left, upper_right, upper_left))

    lines = [
        "# Height-conforming semantic grass generated from complex_terrain_heightmap.png",
        "mtllib complex_grass_surface.mtl",
        "o complex_grass_surface",
        "usemtl semantic_grass",
    ]
    lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices)
    lines.extend(
        f"vt {(x + WORLD_SIZE / 2.0) / 3.0:.6f} "
        f"{(y + WORLD_SIZE / 2.0) / 3.0:.6f}"
        for x, y, _ in vertices
    )
    lines.extend(f"vn {x:.6f} {y:.6f} {z:.6f}" for x, y, z in normals)
    lines.extend(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}" for a, b, c in faces)
    (output_dir / "complex_grass_surface.obj").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    (output_dir / "complex_grass_surface.mtl").write_text(
        "newmtl semantic_grass\n"
        "Ka 0.18 0.48 0.14\n"
        "Kd 0.18 0.48 0.14\n"
        "Ks 0.04 0.04 0.04\n"
        "Ns 5.0\n"
        "d 1.0\n"
        "illum 2\n"
        "map_Ka complex_grass_texture.png\n"
        "map_Kd complex_grass_texture.png\n",
        encoding="utf-8",
    )
    print(f"grass mesh vertices={len(vertices)}, triangles={len(faces)}")


def generate_assets(output_dir):
    axis = np.linspace(-WORLD_SIZE / 2.0, WORLD_SIZE / 2.0, MAP_PIXELS)
    xx, yy = np.meshgrid(axis, axis)
    height = terrain_height(xx, yy)
    normalized = np.clip((height - HEIGHT_OFFSET) / HEIGHT_SCALE, 0.0, 1.0)
    encoded_height = np.round(normalized * 255.0).astype(np.uint8)
    Image.fromarray(encoded_height, mode="L").save(
        str(output_dir / "complex_terrain_heightmap.png")
    )
    generate_grass_mesh(output_dir, encoded_height)

    rng = np.random.default_rng(SEED + 2)
    noise = np.clip(128.0 + 42.0 * rng.normal(size=(256, 256)), 0.0, 255.0).astype(np.uint8)
    smooth = Image.fromarray(noise, mode="L").filter(ImageFilter.GaussianBlur(2.5))
    values = (np.asarray(smooth, dtype=np.float32) - 128.0) / 128.0
    texture = np.empty((256, 256, 3), dtype=np.float32)
    texture[:, :, 0] = 112.0 + 28.0 * values
    texture[:, :, 1] = 92.0 + 22.0 * values
    texture[:, :, 2] = 58.0 + 15.0 * values
    Image.fromarray(np.clip(texture, 0, 255).astype(np.uint8), mode="RGB").save(
        str(output_dir / "complex_terrain_surface.png")
    )

    grass_rng = np.random.default_rng(SEED + 3)
    grass_noise = grass_rng.normal(0.0, 1.0, (256, 256))
    grass_u8 = np.clip(128.0 + 48.0 * grass_noise, 0.0, 255.0).astype(np.uint8)
    grass_smooth = Image.fromarray(grass_u8, mode="L").filter(ImageFilter.GaussianBlur(1.2))
    grass_values = (np.asarray(grass_smooth, dtype=np.float32) - 128.0) / 128.0
    grass_texture = np.empty((256, 256, 3), dtype=np.float32)
    grass_texture[:, :, 0] = 42.0 + 18.0 * grass_values
    grass_texture[:, :, 1] = 126.0 + 42.0 * grass_values
    grass_texture[:, :, 2] = 32.0 + 14.0 * grass_values
    Image.fromarray(
        np.clip(grass_texture, 0, 255).astype(np.uint8), mode="RGB"
    ).save(str(output_dir / "complex_grass_texture.png"))
    normal = np.zeros((8, 8, 3), dtype=np.uint8)
    normal[:, :, :] = (128, 128, 255)
    Image.fromarray(normal, mode="RGB").save(str(output_dir / "complex_terrain_flat_normal.png"))

    spacing = WORLD_SIZE / (MAP_PIXELS - 1)
    gy, gx = np.gradient(height, spacing, spacing)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    bins = [(0, 3), (3, 6), (6, 10), (10, 15), (15, 20), (20, 25), (25, 90)]
    distribution = ", ".join(
        f"{low}-{high}deg={100.0 * np.mean((slope >= low) & (slope < high)):.1f}%"
        for low, high in bins
    )
    print(
        f"terrain height={height.min():.3f}..{height.max():.3f} m, "
        f"p95={np.percentile(slope, 95):.1f} deg, max={slope.max():.1f} deg"
    )
    print(distribution)
    # Gazebo Fortress normalizes an 8-bit heightmap by the largest value that
    # is actually present in the image, rather than by 255.  Scale the SDF
    # height to that same occupied gray range so pixel heights remain equal to
    # HEIGHT_OFFSET + pixel / 255 * HEIGHT_SCALE.  Without this correction the
    # flat spawn disk is about 0.13 m too high and the collision surface slowly
    # pushes the robot into the air.
    return float(encoded_height.max()) / 255.0 * HEIGHT_SCALE


def main():
    parser = argparse.ArgumentParser()
    default_dir = Path(__file__).resolve().parents[1] / "sdf"
    parser.add_argument("--output-dir", type=Path, default=default_dir)
    parser.add_argument("--world", type=Path, default=None)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    world_path = args.world or args.output_dir / "world_complex_semantic_terrain_omni.sdf"
    heightmap_vertical_size = generate_assets(args.output_dir)
    generate_world(world_path, heightmap_vertical_size)
    print(f"wrote {world_path}")


if __name__ == "__main__":
    main()
