# Height-maze semantic Go2 (Gazebo Fortress)

This package is an independent Gazebo Fortress 6 / ROS 1 version of
`quad_hill_high_surface_demo.world`. It does not modify either the Gazebo
Classic scene or `semantic_segmentation_go2`.

The base world keeps the heightmap, 73 maze walls, four pine trees and three
oak trees. Instead of two large terrain overlays, it deterministically scatters
10 small grass islands and nine small gravel islands through wall-free parts of
the scene. The gravel islands contain 54 small physical stones. A 10 m long,
2.8 m wide, 48-step staircase forms a physical route between the lower and
upper surfaces of the left hill. Each category is a separate Fortress model
with an integer Label. The included `go2_semantic_omni` carries four co-located
RGB-D / semantic camera pairs. A separate dynamic variant adds six
collision-enabled moving obstacles with semantic ID 6 without changing the
base world.

The grass and gravel are generated as disconnected irregular islands using a
fixed random seed, so regenerating the package gives the same layout. Every
overlay vertex is sampled from the same heightmap as the collision terrain and
placed only 2 mm above it; the overlays therefore follow the local slope rather
than forming flat plates or floating blocks. Patch placement also excludes the
maze walls, trees, robot spawn, staircase and all six dynamic-obstacle paths.

The staircase center is `(-10.0, 3.5)`, running north-to-south from the low edge
`(-10.0, 8.5)` to the high edge `(-10.0, -1.5)`. Its first tread is about
0.040 m above the lower surface, the maximum generated rise is about 0.061 m,
and the final transition to the upper surface is about 0.008 m. Every tread has
collision geometry and semantic ID 7, so the staircase is a bidirectional
physical route, not a visual-only layer.

## Build

```bash
cd /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws
catkin_make
source devel/setup.bash
```

The generated world and assets are already present. Regenerate only after the
source Classic demo changes:

```bash
rosrun height_maze_semantic_go2 generate_fortress_world.py
rosrun height_maze_semantic_go2 generate_dynamic_fortress_world.py
```

## Run

```bash
source /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws/devel/setup.bash
roslaunch height_maze_semantic_go2 height_maze_semantic_go2.launch
```

Run the independent dynamic-obstacle variant with:

```bash
roslaunch height_maze_semantic_go2 height_maze_semantic_go2_dynamic.launch
```

Its moving box, long bar, two cylinders, east-side box and sphere use the same
`libssmi_periodic_motion_system.so` plugin as `semantic_segmentation_go2`.
They move deterministically along six wall-free corridors, carry physical
collision geometry, and are labeled `dynamic_obstacle` (ID 6, canonical color
`[255, 0, 255]`). The original `height_maze_semantic_go2.sdf` remains the
no-dynamic-obstacle version.

| Model | Shape | Start -> end (m) | Period (s) |
| --- | --- | --- | ---: |
| `dynamic_spawn_box` | box | `(-5,-14) -> (3,-14)` | 22 |
| `dynamic_southwest_long_bar` | long box | `(-22,-14) -> (-10,-14)` | 34 |
| `dynamic_west_cylinder` | cylinder | `(-25.5,2) -> (-25.5,10)` | 26 |
| `dynamic_east_box` | box | `(24,-12) -> (24,2)` | 32 |
| `dynamic_north_sphere` | sphere | `(6,14) -> (10,14)` | 18 |
| `dynamic_southeast_cylinder` | cylinder | `(6,-10) -> (10,-10)` | 20 |

As in the reference package, these are scripted kinematic obstacles: the
collision body follows the commanded periodic pose, but external forces do not
change its trajectory. This provides repeatable dynamic occlusion, semantic
observations and collision checking rather than free rigid-body dynamics.

For a server-only run without RViz:

```bash
roslaunch height_maze_semantic_go2 height_maze_semantic_go2.launch \
  headless:=true start_rviz:=false
```

Important ROS topics:

- `/semantic/labels_map` and the corresponding left/rear/right topics;
- `/rgbd_camera/depth_image` and the corresponding left/rear/right topics;
- `/semantic_pcl/front`, `/semantic_pcl/left`, `/semantic_pcl/rear`, `/semantic_pcl/right`;
- `/semantic_pcl/semantic_pcl`, the four-view fused `sensor_msgs/PointCloud2`.

Semantic IDs are defined in `config/label_colors_height_maze.yaml`: flat ground
1, grass 2, gravel 3, bare heightmap 5, stairs 7, rocks 8, vegetation 12 and
maze walls 13.

On the installed Fortress 6.17.1 / ODE stack, collision and Ogre2 rendering
both use the full 8-bit range for `size.Z`. The generator gives both layers the
same 2.5 m Z size, so their surface is `gray / 255 * 2.5 m`, exactly matching
the grass and gravel meshes. They remain separate models only so the visual
terrain can carry a semantic Label without affecting collision configuration.
