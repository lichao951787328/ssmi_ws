#!/usr/bin/env python3
"""Generate an independent Fortress semantic version of the Classic height maze.

The source Gazebo Classic files are read-only inputs.  All generated assets and
the SDF 1.8 world are written inside height_maze_semantic_go2.
"""

import argparse
import copy
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MAPS_DIR = Path("/home/yanaibo/mapless_navigation/sim_scence/src/gazebo_maps")
DEFAULT_TREE_MODELS = Path("/home/yanaibo/.gazebo/models")
WORLD_NAME = "semantic_segmentation_world"
CLASSIC_HEIGHT_SIZE = 2.5
HEIGHTMAP_SIZE_XY = 51.2
SURFACE_OFFSET = 0.002
BASE_GROUND_TOP = 0.015
PATCH_SEED = 20260831
GRAVEL_ROCK_SEED = 20260901

# A 10 m stairway aligned with the left hill's main gradient.  This corridor
# stays clear of the maze walls and the trees, and the vector points from the
# low north ground to the high south ground.
STAIR_CENTER = np.asarray((-10.0, 3.5), dtype=np.float64)
STAIR_YAW = math.radians(270.0)
STAIR_LENGTH = 10.0
STAIR_WIDTH = 2.8
STAIR_COUNT = 48
MINIMUM_STAIR_RISE = 0.005

# Dynamic paths are excluded from random surface patches so the physical
# gravel stones do not obstruct the deterministic moving models.
DYNAMIC_PATH_SEGMENTS = (
    ((-5.0, -14.0), (3.0, -14.0)),
    ((-22.0, -14.0), (-10.0, -14.0)),
    ((-25.5, 2.0), (-25.5, 10.0)),
    ((24.0, -12.0), (24.0, 2.0)),
    ((6.0, 14.0), (10.0, 14.0)),
    ((6.0, -10.0), (10.0, -10.0)),
)


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, {key: str(value) for key, value in attributes.items()})
    if text is not None:
        node.text = str(text)
    return node


def add_pose(parent, values):
    return element(parent, "pose", " ".join("{:.6f}".format(float(value)) for value in values))


def parse_pose(text):
    values = [float(value) for value in (text or "0 0 0 0 0 0").split()]
    return (values + [0.0] * 6)[:6]


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


def add_label(model, label):
    plugin = element(
        model,
        "plugin",
        filename="ignition-gazebo-label-system",
        name="ignition::gazebo::systems::Label",
    )
    element(plugin, "label", int(label))


def add_simple_material(visual, diffuse, ambient=None, specular=(0.04, 0.04, 0.04, 1.0)):
    if ambient is None:
        ambient = tuple(channel * 0.72 for channel in diffuse[:3]) + (diffuse[3],)
    material = element(visual, "material")
    element(material, "ambient", " ".join(str(value) for value in ambient))
    element(material, "diffuse", " ".join(str(value) for value in diffuse))
    element(material, "specular", " ".join(str(value) for value in specular))
    return material


def add_box_geometry(parent, size):
    geometry = element(parent, "geometry")
    box = element(geometry, "box")
    element(box, "size", " ".join("{:.6f}".format(float(value)) for value in size))


def add_sphere_geometry(parent, radius):
    geometry = element(parent, "geometry")
    sphere = element(geometry, "sphere")
    element(sphere, "radius", "{:.6f}".format(float(radius)))


def add_mesh_geometry(parent, uri, submesh=None):
    geometry = element(parent, "geometry")
    mesh = element(geometry, "mesh")
    element(mesh, "uri", uri)
    if submesh:
        sub = element(mesh, "submesh")
        element(sub, "name", submesh)


def add_friction(collision, mu=1.0):
    surface = element(collision, "surface")
    friction = element(surface, "friction")
    ode = element(friction, "ode")
    element(ode, "mu", mu)
    element(ode, "mu2", mu)


def terrain_height(heightmap, x, y):
    """Bilinearly sample the heightmap in Fortress world coordinates."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    rows, columns = heightmap.shape
    pixel_x = np.clip((x / HEIGHTMAP_SIZE_XY + 0.5) * (columns - 1), 0.0, columns - 1)
    # The first image row is the positive world-Y edge.
    pixel_y = np.clip((0.5 - y / HEIGHTMAP_SIZE_XY) * (rows - 1), 0.0, rows - 1)
    x0 = np.floor(pixel_x).astype(int)
    y0 = np.floor(pixel_y).astype(int)
    x1 = np.minimum(x0 + 1, columns - 1)
    y1 = np.minimum(y0 + 1, rows - 1)
    dx = pixel_x - x0
    dy = pixel_y - y0
    gray = (
        heightmap[y0, x0] * (1.0 - dx) * (1.0 - dy)
        + heightmap[y0, x1] * dx * (1.0 - dy)
        + heightmap[y1, x0] * (1.0 - dx) * dy
        + heightmap[y1, x1] * dx * dy
    )
    return gray / 255.0 * CLASSIC_HEIGHT_SIZE


def physical_surface_height(heightmap, x, y):
    return np.maximum(terrain_height(heightmap, x, y), BASE_GROUND_TOP)


def terrain_normal(heightmap, x, y):
    epsilon = 0.05
    dzdx = float(terrain_height(heightmap, x + epsilon, y) - terrain_height(heightmap, x - epsilon, y)) / (2.0 * epsilon)
    dzdy = float(terrain_height(heightmap, x, y + epsilon) - terrain_height(heightmap, x, y - epsilon)) / (2.0 * epsilon)
    normal = np.asarray((-dzdx, -dzdy, 1.0), dtype=np.float64)
    return normal / np.linalg.norm(normal)


def copy_assets(maps_dir, tree_models):
    assets_dir = PACKAGE_DIR / "worlds" / "assets"
    models_dir = PACKAGE_DIR / "models"
    assets_dir.mkdir(parents=True, exist_ok=True)
    models_dir.mkdir(parents=True, exist_ok=True)

    heightmap_source = maps_dir / "height_maze" / "quad_height" / "materials" / "textures" / "quad_height.png"
    ground_source = maps_dir / "height_maze" / "quad_ground" / "materials" / "textures" / "blue_ground.jpg"
    required = [
        heightmap_source,
        ground_source,
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("Missing source assets: {}".format(", ".join(missing)))

    shutil.copy2(heightmap_source, assets_dir / "quad_height.png")
    shutil.copy2(ground_source, assets_dir / "blue_ground.jpg")
    for tree_name in ("pine_tree", "oak_tree"):
        source = tree_models / tree_name
        if not source.is_dir():
            raise FileNotFoundError("Missing tree model directory: {}".format(source))
        shutil.copytree(source, models_dir / tree_name, dirs_exist_ok=True)

    heightmap = np.asarray(Image.open(heightmap_source).convert("L"), dtype=np.uint8)
    normalized = heightmap.astype(np.float32) / 255.0
    # A neutral earth texture makes the grass and gravel semantic overlays clear.
    low = np.asarray([76.0, 91.0, 58.0], dtype=np.float32)
    high = np.asarray([164.0, 126.0, 78.0], dtype=np.float32)
    rgb = low[None, None, :] * (1.0 - normalized[:, :, None]) + high[None, None, :] * normalized[:, :, None]
    Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8), mode="RGB").save(assets_dir / "terrain_surface.png")
    normal = np.zeros((8, 8, 3), dtype=np.uint8)
    normal[:, :, :] = (128, 128, 255)
    Image.fromarray(normal, mode="RGB").save(assets_dir / "terrain_flat_normal.png")
    return heightmap


def maze_wall_layout(maps_dir):
    maze_root = ET.parse(maps_dir / "height_maze" / "quad_maze" / "model.sdf").getroot()
    wall_lengths = {"long_wall": 32.0, "unit_wall": 4.0, "half_wall": 2.0}
    outer_x, outer_y, outer_yaw = -12.0, 0.0, 1.5708
    walls = []
    for index, include in enumerate(maze_root.findall(".//include")):
        name = include.findtext("name") or "wall_{}".format(index)
        wall_type = (include.findtext("uri") or "").replace("model://", "")
        if wall_type not in wall_lengths:
            continue
        local_x, local_y, _, _, _, local_yaw = parse_pose(include.findtext("pose"))
        x = outer_x + math.cos(outer_yaw) * local_x - math.sin(outer_yaw) * local_y
        y = outer_y + math.sin(outer_yaw) * local_x + math.cos(outer_yaw) * local_y
        yaw = outer_yaw + local_yaw
        walls.append((name, (x, y, 1.25, 0.0, 0.0, yaw), (wall_lengths[wall_type], 0.15, 2.5)))
    return walls


def tree_positions(maps_dir):
    map_root = ET.parse(maps_dir / "height_maze" / "quad_map" / "model.sdf").getroot()
    positions = []
    for include in map_root.findall(".//include"):
        tree_type = (include.findtext("uri") or "").replace("model://", "")
        if tree_type not in ("pine_tree", "oak_tree"):
            continue
        pose = parse_pose(include.findtext("pose"))
        positions.append((pose[0], pose[1]))
    return positions


def stair_axis_and_endpoints():
    axis = np.asarray((math.cos(STAIR_YAW), math.sin(STAIR_YAW)), dtype=np.float64)
    low = STAIR_CENTER - axis * (STAIR_LENGTH / 2.0)
    high = STAIR_CENTER + axis * (STAIR_LENGTH / 2.0)
    return axis, low, high


def distance_to_segment(point, start, end):
    direction = end - start
    fraction = np.clip(np.dot(point - start, direction) / np.dot(direction, direction), 0.0, 1.0)
    return float(np.linalg.norm(point - (start + fraction * direction)))


def distance_to_wall(point, pose, size):
    dx = point[0] - pose[0]
    dy = point[1] - pose[1]
    cosine = math.cos(pose[5])
    sine = math.sin(pose[5])
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    outside_x = max(abs(local_x) - size[0] / 2.0, 0.0)
    outside_y = max(abs(local_y) - size[1] / 2.0, 0.0)
    return math.hypot(outside_x, outside_y)


def inside_irregular_ellipse(x, y, center, radii, phase):
    normalized_x = (x - center[0]) / radii[0]
    normalized_y = (y - center[1]) / radii[1]
    angle = math.atan2(normalized_y, normalized_x)
    boundary = 1.0 + 0.065 * math.sin(5.0 * angle + phase) + 0.035 * math.sin(9.0 * angle)
    return math.hypot(normalized_x, normalized_y) <= boundary


def generate_surface_patch_layout(maps_dir):
    """Generate separated small patches using a repeatable random sequence."""
    rng = np.random.default_rng(PATCH_SEED)
    walls = maze_wall_layout(maps_dir)
    trees = tree_positions(maps_dir)
    _, stair_low, stair_high = stair_axis_and_endpoints()
    excluded_segments = [(stair_low, stair_high)] + [
        (np.asarray(start, dtype=np.float64), np.asarray(end, dtype=np.float64))
        for start, end in DYNAMIC_PATH_SEGMENTS
    ]
    sequence = ["grass", "gravel"] * 8 + ["grass", "grass", "gravel"]
    accepted = []
    by_kind = {"grass": [], "gravel": []}
    for kind in sequence:
        for _ in range(10000):
            center = np.asarray((rng.uniform(-24.0, 24.0), rng.uniform(-14.2, 14.2)))
            radii = (float(rng.uniform(0.75, 1.45)), float(rng.uniform(0.65, 1.25)))
            phase = float(rng.uniform(0.0, 2.0 * math.pi))
            radius = max(radii)
            if abs(center[0]) + radius > 25.0 or abs(center[1]) + radius > 15.25:
                continue
            if min(distance_to_wall(center, pose, size) for _, pose, size in walls) < radius + 0.35:
                continue
            if min(math.hypot(center[0] - x, center[1] - y) for x, y in trees) < radius + 1.5:
                continue
            if math.hypot(center[0], center[1] + 12.0) < radius + 1.5:
                continue
            if any(distance_to_segment(center, start, end) < radius + 1.0 for start, end in excluded_segments):
                continue
            if any(
                np.linalg.norm(center - previous["center"])
                < radius + max(previous["radii"]) + 0.55
                for previous in accepted
            ):
                continue
            patch = {"center": center, "radii": radii, "phase": phase}
            accepted.append(patch)
            by_kind[kind].append(patch)
            break
        else:
            raise RuntimeError("Could not place all random {} patches".format(kind))
    return by_kind


def write_multi_patch_mesh(path, heightmap, patches, material_color):
    """Write disconnected, height-conforming islands into one semantic mesh."""
    spacing = 0.2
    vertex_lines = []
    texture_lines = []
    normal_lines = []
    face_lines = []
    for patch_index, patch in enumerate(patches):
        center = patch["center"]
        radii = patch["radii"]
        phase = patch["phase"]
        x_min = math.floor((center[0] - radii[0] * 1.12) / spacing) * spacing
        x_max = math.ceil((center[0] + radii[0] * 1.12) / spacing) * spacing
        y_min = math.floor((center[1] - radii[1] * 1.12) / spacing) * spacing
        y_max = math.ceil((center[1] + radii[1] * 1.12) / spacing) * spacing
        xs = np.arange(x_min, x_max + spacing * 0.5, spacing)
        ys = np.arange(y_min, y_max + spacing * 0.5, spacing)
        cells = []
        used = set()
        for iy in range(len(ys) - 1):
            for ix in range(len(xs) - 1):
                cell_x = (xs[ix] + xs[ix + 1]) * 0.5
                cell_y = (ys[iy] + ys[iy + 1]) * 0.5
                if not inside_irregular_ellipse(cell_x, cell_y, center, radii, phase):
                    continue
                corners = ((ix, iy), (ix + 1, iy), (ix + 1, iy + 1), (ix, iy + 1))
                cells.append(corners)
                used.update(corners)
        ordered = sorted(used, key=lambda item: (item[1], item[0]))
        first_index = len(vertex_lines) + 1
        indices = {grid_index: first_index + index for index, grid_index in enumerate(ordered)}
        for ix, iy in ordered:
            x = float(xs[ix])
            y = float(ys[iy])
            z = float(terrain_height(heightmap, x, y)) + SURFACE_OFFSET
            vertex_lines.append("v {:.6f} {:.6f} {:.6f}".format(x, y, z))
            texture_lines.append("vt {:.6f} {:.6f}".format((x - center[0]) * 0.5, (y - center[1]) * 0.5))
            normal_lines.append("vn {:.7f} {:.7f} {:.7f}".format(*terrain_normal(heightmap, x, y)))
        face_lines.append("g {}_island_{:02d}".format(path.stem, patch_index + 1))
        for a, b, c, d in cells:
            ia, ib, ic, id_ = (indices[a], indices[b], indices[c], indices[d])
            face_lines.append("f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}".format(ia, ib, ic))
            face_lines.append("f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}".format(ia, ic, id_))
    header = [
        "# Deterministic small terrain-conforming semantic islands",
        "mtllib {}".format(path.with_suffix(".mtl").name),
        "o {}".format(path.stem),
        "usemtl {}_material".format(path.stem),
    ]
    path.write_text("\n".join(header + vertex_lines + texture_lines + normal_lines + face_lines) + "\n", encoding="utf-8")
    path.with_suffix(".mtl").write_text(
        "newmtl {0}_material\nKa {1:.4f} {2:.4f} {3:.4f}\n"
        "Kd {1:.4f} {2:.4f} {3:.4f}\nKs 0.02 0.02 0.02\nNs 4.0\n".format(
            path.stem, *material_color
        ),
        encoding="utf-8",
    )
    return len(vertex_lines), sum(1 for line in face_lines if line.startswith("f "))


def generate_surface_patch_assets(heightmap, layout):
    assets_dir = PACKAGE_DIR / "worlds" / "assets"
    grass_stats = write_multi_patch_mesh(
        assets_dir / "grass_small_patches.obj", heightmap, layout["grass"], (0.18, 0.48, 0.10)
    )
    gravel_stats = write_multi_patch_mesh(
        assets_dir / "gravel_small_patches.obj", heightmap, layout["gravel"], (0.46, 0.43, 0.38)
    )
    return grass_stats, gravel_stats


def add_world_systems(world):
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


def add_scene(world):
    physics = element(world, "physics", name="1ms", type="ignored")
    element(physics, "max_step_size", 0.001)
    element(physics, "real_time_factor", 1.0)
    element(physics, "gravity", "0 0 -9.8")
    scene = element(world, "scene")
    element(scene, "ambient", "0.60 0.60 0.60 1")
    element(scene, "background", "0.55 0.68 0.78 1")
    element(scene, "shadows", "true")
    light = element(world, "light", type="directional", name="sun")
    element(light, "cast_shadows", "true")
    add_pose(light, (0, 0, 30, 0, 0, 0))
    element(light, "diffuse", "0.90 0.86 0.76 1")
    element(light, "specular", "0.20 0.20 0.18 1")
    attenuation = element(light, "attenuation")
    element(attenuation, "range", 1000)
    element(attenuation, "constant", 0.9)
    element(attenuation, "linear", 0.01)
    element(attenuation, "quadratic", 0.001)
    element(light, "direction", "-0.45 0.25 -0.86")


def add_heightmap(world, heightmap):
    # On the installed Fortress 6.17.1 / ODE stack, both collision and Ogre2
    # rendering map the full 8-bit range (0..255) to size.Z.  The source PNG's
    # maximum is only 208, so reducing size.Z by 208/255 would scale the
    # collision surface twice and put the robot below the rendered slope.
    collision_height = CLASSIC_HEIGHT_SIZE
    collision_model = element(world, "model", name="heightmap_terrain_collision")
    element(collision_model, "static", "true")
    collision_link = element(collision_model, "link", name="terrain_collision_link")
    collision = element(collision_link, "collision", name="terrain_collision")
    geometry = element(collision, "geometry")
    height = element(geometry, "heightmap")
    element(height, "uri", "file://assets/quad_height.png")
    element(height, "size", "51.2 51.2 {:.9f}".format(collision_height))
    element(height, "pos", "0 0 0")
    add_friction(collision, 1.1)

    visual_model = element(world, "model", name="heightmap_terrain")
    element(visual_model, "static", "true")
    visual_link = element(visual_model, "link", name="terrain_visual_link")
    visual = element(visual_link, "visual", name="terrain_visual")
    geometry = element(visual, "geometry")
    height = element(geometry, "heightmap")
    element(height, "use_terrain_paging", "false")
    texture = element(height, "texture")
    element(texture, "diffuse", "file://assets/terrain_surface.png")
    element(texture, "normal", "file://assets/terrain_flat_normal.png")
    element(texture, "size", 8)
    element(height, "uri", "file://assets/quad_height.png")
    element(height, "size", "51.2 51.2 {:.9f}".format(CLASSIC_HEIGHT_SIZE))
    element(height, "pos", "0 0 0")
    add_label(visual_model, 5)
    return collision_height


def add_base_ground(world):
    model = element(world, "model", name="flat_ground")
    element(model, "static", "true")
    link = element(model, "link", name="ground_link")
    for tag in ("collision", "visual"):
        node = element(link, tag, name="ground_{}".format(tag))
        add_box_geometry(node, (56.0, 32.0, 0.03))
        if tag == "visual":
            material = add_simple_material(node, (0.22, 0.38, 0.65, 1.0))
            pbr = element(material, "pbr")
            metal = element(pbr, "metal")
            element(metal, "albedo_map", "file://assets/blue_ground.jpg")
            element(metal, "roughness", 0.9)
            element(metal, "metalness", 0.0)
        else:
            add_friction(node, 1.0)
    add_label(model, 1)


def add_surface_patch(world, model_name, mesh_name, label, color):
    model = element(world, "model", name=model_name)
    element(model, "static", "true")
    link = element(model, "link", name="{}_link".format(model_name))
    visual = element(link, "visual", name="{}_visual".format(model_name))
    add_mesh_geometry(visual, "file://assets/{}".format(mesh_name))
    add_simple_material(visual, color)
    element(visual, "cast_shadows", "false")
    add_label(model, label)


def add_gravel_rocks(world, heightmap, gravel_patches):
    """Distribute physical stones over the separated gravel islands."""
    rng = np.random.default_rng(GRAVEL_ROCK_SEED)
    model = element(world, "model", name="natural_rocks")
    element(model, "static", "true")
    link = element(model, "link", name="natural_rocks_link")
    count = 0
    for patch_index, patch in enumerate(gravel_patches):
        center = patch["center"]
        radii = patch["radii"]
        for local_index in range(6):
            angle = float(rng.uniform(0.0, 2.0 * math.pi))
            radial = math.sqrt(float(rng.uniform(0.0, 1.0))) * 0.72
            x = float(center[0] + radii[0] * radial * math.cos(angle))
            y = float(center[1] + radii[1] * radial * math.sin(angle))
            radius = float(rng.uniform(0.045, 0.10))
            z = float(physical_surface_height(heightmap, x, y)) + SURFACE_OFFSET + radius * 0.45
            name = "patch_{:02d}_stone_{:02d}".format(patch_index + 1, local_index + 1)
            collision = element(link, "collision", name="{}_collision".format(name))
            add_pose(collision, (x, y, z, 0.0, 0.0, 0.0))
            add_sphere_geometry(collision, radius)
            add_friction(collision, 1.0)
            visual = element(link, "visual", name="{}_visual".format(name))
            add_pose(visual, (x, y, z, 0.0, 0.0, 0.0))
            add_sphere_geometry(visual, radius)
            shade = float(rng.uniform(0.28, 0.48))
            tint = float(rng.uniform(-0.025, 0.025))
            add_simple_material(
                visual,
                (shade + tint, shade, shade - tint, 1.0),
            )
            count += 1
    add_label(model, 8)
    return count


def add_traversable_staircase(world, heightmap):
    """Add a stair path whose first and last treads meet the adjacent terrain."""
    axis, low_edge, high_edge = stair_axis_and_endpoints()
    across = np.asarray((-axis[1], axis[0]), dtype=np.float64)
    run = STAIR_LENGTH / STAIR_COUNT
    edge_samples = np.linspace(-STAIR_WIDTH / 2.0, STAIR_WIDTH / 2.0, 21)
    low_ground = max(
        float(physical_surface_height(heightmap, *(low_edge + across * offset)))
        for offset in edge_samples
    )
    high_ground = max(
        float(physical_surface_height(heightmap, *(high_edge + across * offset)))
        for offset in edge_samples
    )
    nominal_rise = (high_ground - low_ground) / STAIR_COUNT

    model = element(world, "model", name="staircase")
    element(model, "static", "true")
    link = element(model, "link", name="staircase_link")
    previous_top = None
    tops = []
    for index in range(STAIR_COUNT):
        center_xy = low_edge + axis * ((index + 0.5) * run)
        footprint = []
        for along in np.linspace(-run / 2.0, run / 2.0, 7):
            for offset in np.linspace(-STAIR_WIDTH / 2.0, STAIR_WIDTH / 2.0, 15):
                point = center_xy + axis * along + across * offset
                footprint.append(float(physical_surface_height(heightmap, *point)))
        desired_top = low_ground + (index + 1) * nominal_rise
        top = max(desired_top, max(footprint) + 0.008)
        if previous_top is not None:
            # Keep every tread non-descending without forcing a rise larger
            # than the Go2 teleop controller's default 0.06 m step height.
            top = max(top, previous_top + MINIMUM_STAIR_RISE)
        bottom = min(footprint) - 0.04
        height = top - bottom
        center_z = (top + bottom) / 2.0
        size = (run + 0.015, STAIR_WIDTH, height)
        pose = (center_xy[0], center_xy[1], center_z, 0.0, 0.0, STAIR_YAW)
        collision = element(link, "collision", name="step_{:02d}_collision".format(index + 1))
        add_pose(collision, pose)
        add_box_geometry(collision, size)
        add_friction(collision, 1.2)
        visual = element(link, "visual", name="step_{:02d}_visual".format(index + 1))
        add_pose(visual, pose)
        add_box_geometry(visual, size)
        shade = 0.48 + index * 0.006
        add_simple_material(visual, (shade, shade * 0.88, shade * 0.68, 1.0))
        previous_top = top
        tops.append(top)
    add_label(model, 7)
    return {
        "low_edge": low_edge,
        "high_edge": high_edge,
        "low_ground": low_ground,
        "high_ground": high_ground,
        "first_top": tops[0],
        "last_top": tops[-1],
        "maximum_rise": max(np.diff([low_ground] + tops)),
    }


def copy_addition_link(world, source_model, source_link_name, model_name, label):
    source_link = source_model.find("link[@name='{}']".format(source_link_name))
    if source_link is None:
        raise ValueError("Missing source link {}".format(source_link_name))
    model = element(world, "model", name=model_name)
    element(model, "static", "true")
    link = element(model, "link", name="{}_link".format(model_name))
    for child in source_link:
        if child.tag in ("collision", "visual"):
            link.append(copy.deepcopy(child))
    add_label(model, label)


def add_maze_walls(world, maps_dir):
    model = element(world, "model", name="maze_walls")
    element(model, "static", "true")
    link = element(model, "link", name="maze_wall_link")
    count = 0
    for name, pose, size in maze_wall_layout(maps_dir):
        collision = element(link, "collision", name="{}_collision".format(name))
        add_pose(collision, pose)
        add_box_geometry(collision, size)
        add_friction(collision, 1.0)
        visual = element(link, "visual", name="{}_visual".format(name))
        add_pose(visual, pose)
        add_box_geometry(visual, size)
        add_simple_material(visual, (0.76, 0.76, 0.72, 1.0))
        count += 1
    if count != 73:
        raise RuntimeError("Expected 73 maze walls, generated {}".format(count))
    add_label(model, 13)
    return count


def add_tree_group(world, maps_dir, tree_type, label=12):
    map_root = ET.parse(maps_dir / "height_maze" / "quad_map" / "model.sdf").getroot()
    entries = []
    for include in map_root.findall(".//include"):
        uri = (include.findtext("uri") or "").replace("model://", "")
        if uri == tree_type:
            entries.append((include.findtext("name"), parse_pose(include.findtext("pose"))))
    model = element(world, "model", name="{}_group".format(tree_type))
    element(model, "static", "true")
    for name, pose in entries:
        link = element(model, "link", name=name)
        add_pose(link, pose)
        collision = element(link, "collision", name="{}_collision".format(name))
        add_mesh_geometry(collision, "model://{0}/meshes/{0}.dae".format(tree_type))
        add_friction(collision, 1.0)
        for submesh, color in (("Branch", (0.16, 0.45, 0.12, 1.0)), ("Bark", (0.34, 0.20, 0.10, 1.0))):
            visual = element(link, "visual", name="{}_{}_visual".format(name, submesh.lower()))
            add_mesh_geometry(visual, "model://{0}/meshes/{0}.dae".format(tree_type), submesh=submesh)
            add_simple_material(visual, color)
    add_label(model, label)
    return len(entries)


def add_robot(world):
    include = element(world, "include")
    element(include, "uri", "model://go2_semantic_omni")
    element(include, "name", "go2_semantic_omni")
    # Flat, wall-free start area; 0.36 m keeps the nominal feet just clear.
    add_pose(include, (0.0, -12.0, 0.36, 0.0, 0.0, 0.0))


def generate_world(maps_dir, tree_models):
    heightmap = copy_assets(maps_dir, tree_models)
    patch_layout = generate_surface_patch_layout(maps_dir)
    grass_stats, gravel_stats = generate_surface_patch_assets(heightmap, patch_layout)

    sdf = ET.Element("sdf", version="1.8")
    world = element(sdf, "world", name=WORLD_NAME)
    add_world_systems(world)
    add_scene(world)
    fortress_height = add_heightmap(world, heightmap)
    add_base_ground(world)
    add_surface_patch(
        world, "grass_surface", "grass_small_patches.obj", 2, (0.18, 0.48, 0.10, 1.0)
    )
    add_surface_patch(
        world, "gravel_surface", "gravel_small_patches.obj", 3, (0.46, 0.43, 0.38, 1.0)
    )
    rock_count = add_gravel_rocks(world, heightmap, patch_layout["gravel"])
    stair_report = add_traversable_staircase(world, heightmap)
    wall_count = add_maze_walls(world, maps_dir)
    pine_count = add_tree_group(world, maps_dir, "pine_tree")
    oak_count = add_tree_group(world, maps_dir, "oak_tree")
    add_robot(world)

    indent_xml(sdf)
    output = PACKAGE_DIR / "worlds" / "height_maze_semantic_go2.sdf"
    ET.ElementTree(sdf).write(output, encoding="utf-8", xml_declaration=True)
    print("Generated {}".format(output))
    print("Fortress collision and visual size.Z: {:.9f} m (PNG max {}, surface max {:.9f} m)".format(fortress_height, int(heightmap.max()), fortress_height * float(heightmap.max()) / 255.0))
    print(
        "Small terrain islands: {} grass ({} vertices / {} triangles), "
        "{} gravel ({} vertices / {} triangles)".format(
            len(patch_layout["grass"]), *grass_stats,
            len(patch_layout["gravel"]), *gravel_stats
        )
    )
    print(
        "Traversable stairs: {} steps, low ground {:.3f} -> first top {:.3f} m; "
        "last top {:.3f} -> high ground {:.3f} m; maximum rise {:.3f} m".format(
            STAIR_COUNT,
            stair_report["low_ground"],
            stair_report["first_top"],
            stair_report["last_top"],
            stair_report["high_ground"],
            stair_report["maximum_rise"],
        )
    )
    print(
        "Semantic models: terrain, ground, grass, gravel, {} rocks, {} steps, "
        "{} walls, {} pines, {} oaks".format(
            rock_count, STAIR_COUNT, wall_count, pine_count, oak_count
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gazebo-maps", type=Path, default=DEFAULT_MAPS_DIR)
    parser.add_argument("--tree-models", type=Path, default=DEFAULT_TREE_MODELS)
    arguments = parser.parse_args()
    generate_world(arguments.gazebo_maps.resolve(), arguments.tree_models.resolve())


if __name__ == "__main__":
    main()
