# Semantic-Octomap
Semantic 3-D OctoMap implementation for building probabilistic multi-class maps of an environment using semantic pointcloud input.

## Dependencies
* ROS Melodic or Noetic
* Catkin
* PCL
* OctoMap
* octomap_msgs
* octomap_rviz_plugins
* scikit-image

## How to use
1. `launch/semantic_octomap.launch` starts ROS nodes `semantic_cloud` and `semantic_octomap_node`.
2. `semantic_cloud` takes three **aligned** images, i.e. RBG image, depth image, and semantic segmentation image. The output is a sematic pointcloud topic of type `sensor_msgs/PointCloud2`.
3. `semantic_octomap_node` receives the generated semantic pointcloud and updates the semantic OctoMap. This node internally maintains a semantic OcTree where each node stores the probability of each object class. Two types of semantic OctoMap topic are published as instances of `octomap_msgs/Octomap` message: `octomap_full` and `octomap_color`. `octomap_full` contains the full probability distribution over the object classes, while `octomap_color` only stores the maximmum likelihood semantic OcTree (with probabilistic occupancies). A probabilistic 2-D occupancy map topic is additionally published via projection of the OctoMap on the ground plane.
4. Note that `octomap_rviz_plugins` can only visualize `octomap_color`, whereas visualizing `octomap_full` causes Rviz to crash.
5. `params/semantic_cloud.yaml` stores camera intrinsic parameters. This should be set to the values used by your camera.
6. `params/octomap_generator.yaml` stores parameters for the semantic OctoMap such as minimum grid size (`resolution`), log-odds increments (`psi` and `phi`), and path for saving the final map.

### Local semantic and traversability grid input

For an upstream `/grids_points` cloud containing `x`, `y`, `z`,
`semantic_lable` and `traversability`, launch:

```bash
roslaunch semantic_octomap semantic_octomap_grid.launch \
  world_frame_id:=world obstacle_threshold:=0.6
```

The launch starts `grid_semantic_adapter_node` and
`semantic_octomap_node`; it does not start the RGB-D semantic sensor node.
The adapter preserves existing static obstacle classes, maps Cityscapes
dynamic labels 11--18 to magenta, and maps high-cost road, sidewalk, terrain
or unknown points to the existing wall class. It publishes the SSMI fields
`x y z rgb semantic_color` on `/semantic_pcl/semantic_pcl`.

`/grids_points` is treated as an already-fused local grid. The accompanying
configuration sets `input_mode: local_grid` and disables free-space ray
casting, so missing cells or cells outside the moving local window are not
interpreted as free observations. A timestamp-compatible transform from the
selected `world_frame_id` to the input cloud frame (normally `wuba_base`) must
exist.

Mapping rules and topic names are configurable in
`params/semantic_octomap_grid.yaml`.
