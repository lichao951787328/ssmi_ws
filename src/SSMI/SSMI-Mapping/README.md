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

### Admitted global semantic grid input

The persistent tree only consumes the strict global admission cloud produced
by the local 3-D semantic voxel node. Launch with:

```bash
roslaunch semantic_octomap semantic_octomap_grid.launch \
  world_frame_id:=map_start
```

The launch starts `grid_semantic_adapter_node` and
`semantic_octomap_node`; it does not start the RGB-D semantic sensor node.
The adapter subscribes to
`/local_3d_semantic_voxel_map/global_semantic_admission_grid` and publishes
`x y z rgb semantic_color` on `/semantic_pcl/global_admitted`. It admits only
terrain labels 0, 1 and 9 and confirmed static-obstacle labels 2 through 8.
Dynamic, unknown and any other unexpected labels are counted and dropped.
The adapter deliberately does not read traversability or confidence: the
local voxel map owns those decisions and must convert any confirmed,
persistent high-cost geometry (for example a stair riser) to an admitted
static-obstacle label before publishing this cloud. No label is remapped to
wall inside the adapter.

The admission cloud is already in `map_start`. The configuration fixes the
tree resolution at 0.4 m, sets `input_mode: local_grid`, disables raycast
clearing, and uses `world_frame_id: map_start` with
`use_initial_pose_reference: false`. Therefore SemanticOcTree does not publish
another `map -> map_start`; that transform belongs exclusively to the local
voxel node. Missing cells and dynamic occlusion are not free evidence. In this
conservative phase the tree only inserts admitted endpoints and retains old
static evidence unless repeated explicit semantic evidence reclassifies it.

Mapping rules and topic names are configurable in
`params/semantic_octomap_grid.yaml`.

For the recorded localization bag used by this workspace, run the complete
adapter-to-OctoMap visualization pipeline with:

```bash
roslaunch semantic_octomap semantic_octomap_grid_bag.launch
```

This launch expects the bag to contain the admitted cloud and the local voxel
node's `map -> map_start` transform. SemanticOcTree publishes `/octomap_full`
unchanged as `SemanticOcTree` with `frame_id=map_start`; FAR can continue to
query that full persistent semantic evidence. `/octomap_color` remains the
RViz-compatible view.

To replay continuously and verify automatic map reset after a time rewind:

```bash
roslaunch semantic_octomap semantic_octomap_grid_bag.launch \
  bag_play_args:="--clock --delay=2 --loop"
```

RViz must display `/octomap_color`; `/octomap_full` contains SSMI's custom
semantic tree payload and is not compatible with the standard OctoMap RViz
plugin.

The supplied RViz configuration also loads a dockable `SSMI 语义颜色图例`
panel. The adapter and SemanticOcTree log cumulative admitted static/terrain,
unexpected-label and TF-failure statistics for runtime verification.
