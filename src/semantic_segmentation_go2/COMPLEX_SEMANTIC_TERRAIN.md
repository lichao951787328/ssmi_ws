# Complex semantic terrain

This independent Gazebo Fortress world combines several physical terrain and
obstacle types while preserving pixel-aligned RGB, depth and semantic output
from the existing four-view Go2 camera rig.

The heightmap contains slopes facing all eight principal / diagonal directions.
Most terrain remains below 20 degrees, with selected approach and summit faces
reaching approximately 37 degrees. The north summit is rounded rather than a
rectangular platform: a 21-step, 3.2 m wide staircase climbs its west face and
terminates at a 4.0 x 3.2 m collision platform seated on the hilltop.

## Semantic IDs

| ID | Class | Geometry / region |
|---:|---|---|
| 1 | dirt trail | Winding 2.3 m path across the map |
| 2 | grass | Expanded north and summit grass fields |
| 3 | gravel | South-east gravel surface and 105 physical pebbles |
| 4 | boundary | Four outer retaining walls |
| 5 | bare earth slope | Continuous heightmap terrain |
| 6 | dynamic obstacle | Six moving boxes, cylinders, sphere and long bar |
| 7 | stairs | Wide 21-step mountain stair and hilltop platform |
| 8 | natural rock | Ten boulders |
| 9 | long static | Logs and beam obstacles |
| 10 | rectangular static | Crates and blocks |
| 11 | round static | Cylinders and spheres |
| 12 | vegetation | Tree trunks and crowns |
| 13 | maze corridor wall | Seventeen wall runs subdivided to follow slopes |

The RGB-D image, depth image and semantic camera use matching poses and camera
intrinsics. The ID image repeats the uint8 class ID across its RGB channels;
the colored semantic image uses `config/label_colors_complex_terrain.yaml`.

Run:

```bash
cd /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_go2 seg_go2_complex_terrain_omni.launch
```

Topics for the front view are `/rgbd_camera/image`,
`/rgbd_camera/depth_image`, `/semantic/labels_map` and
`/semantic/colored_map`. Left, rear and right views use their existing suffixes.

Regenerate the deterministic world and assets:

```bash
python3 src/semantic_segmentation_go2/scripts/generate_complex_semantic_terrain.py
```

For the local 3D semantic voxel mapper, use:

```bash
config=$(rospack find semantic_segmentation_go2)/config/semantic_voxel_map_complex_terrain.yaml
```
