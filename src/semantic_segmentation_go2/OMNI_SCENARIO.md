# Grass + false-static movers + omnidirectional semantic view

This is an isolated scenario variant.  It does not modify
`world_large_dynamic.sdf`, `models/go2_semantic/model.sdf`, or the original
launch/config files.

## What changes

- `world_large_dynamic_grass_omni.sdf` adds five irregular grass islands made
  from deterministic random, overlapping visual primitives.  They use semantic
  label 8 and have no extra collision; the original floor is still the walking
  surface.
- `dynamic_obstacle_7` and `dynamic_obstacle_8` keep their periodic motion
  plugins but use label 5.  Their canonical RGB is gray, so the mapper treats
  them as static obstacles.  This is an intentional semantic false negative.
- `go2_semantic_omni` has front/left/rear/right RGB-D and semantic cameras.
  Each has a 100-degree horizontal FOV, giving adjacent views 10 degrees of
  overlap and no nominal horizontal blind sector.
- The four point clouds are synchronized, transformed into `base_link`, and
  published once on `/semantic_pcl/semantic_pcl`.  One merged timestamp avoids
  losing views when downstream code rejects duplicate acquisition stamps.

This is a **semantic 360-degree observation approximation**, not a physical
spinning-lidar scan model.  A normal Gazebo lidar supplies geometry but no
per-return semantic label.  The camera ring preserves the existing pure-RGB
semantic pipeline and gives the local/global map the same all-around
information shape expected from a semantic lidar.  Motion distortion,
vertical-channel layout, beam sparsity, and lidar noise are not simulated.

## Run

```bash
cd /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_go2 seg_go2_park_path_omni.launch \
  start_semantic_cloud:=true start_control:=true start_rviz:=true
```

Start the local voxel map with the matching full configuration:

```bash
roslaunch local3d_semantic_voxel_map semantic_voxel_map.launch \
  config:=$(rospack find semantic_segmentation_go2)/config/semantic_voxel_map_go2_omni.yaml \
  input_topic:=/semantic_pcl/semantic_pcl
```

The important output remains `/semantic_pcl/semantic_pcl`; existing local
voxel admission and SemanticOcTree launch files can stay downstream of it.
This cloud has no traversability field. The supplied voxel-map configuration
therefore enables terrain-height inference: labels 0/1/9 with neighboring
height differences greater than 0.15 m are raised to cost 1.0 and encoded as
obstacles downstream. Ordinary terrain remains in the non-dynamic admission
snapshot at low cost, while `/grids_points` configurations keep this inference
disabled and use their measured traversability.

## Regenerate the derived SDF files

The included generated files make normal launch independent of generation.
When the source world/model changes, regenerate only the derived variants:

```bash
rosrun semantic_segmentation_go2 generate_go2_omni_scenario.py \
  --source-world $(rospack find semantic_segmentation_go2)/sdf/world_large_dynamic.sdf \
  --source-model $(rospack find semantic_segmentation_go2)/models/go2_semantic/model.sdf \
  --output-world $(rospack find semantic_segmentation_go2)/sdf/world_large_dynamic_grass_omni.sdf \
  --output-model $(rospack find semantic_segmentation_go2)/models/go2_semantic_omni/model.sdf
```
