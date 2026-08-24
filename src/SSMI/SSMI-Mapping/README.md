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

### 共享语义配置

所有 label、颜色和全局地图准入规则集中在
[`params/semantic_schema.yaml`](params/semantic_schema.yaml)。启动文件把它加载到全局
ROS 参数 `/semantic_schema`，以下组件读取同一份配置：

- `grid_semantic_adapter_node`：把带整数 label 和 traversability 的旧式网格转换为
  `rgb/semantic_color` 点云；
- `semantic_octomap_node`：再次按颜色检查准入，防止动态、忽略或未知类别进入持久树；
- RViz 的 `SSMI 语义颜色图例`：直接显示配置中的名称、含义、角色和准入状态。

每个 `classes` 条目的字段含义如下：

| 字段 | 含义 | 修改上游时怎么填 |
|---|---|---|
| `label` | 上游整数类别 ID | 改成新感知端发布的 ID |
| `name` | 简短类别名 | 用于日志和图例 |
| `meaning` | 人可读说明 | 写清类别在当前项目中的实际含义 |
| `rgb` | 规范输出 RGB，同时也是有效输入 RGB | 使用 `[R, G, B]`，不要写成 BGR |
| `input_rgb_aliases` | 同一类别可接受的额外输入颜色 | 上游颜色与规范色不同时添加，可省略 |
| `role` | `terrain`、`static_obstacle`、`dynamic_obstacle` 或 `ignore` | 决定地图行为，不再依赖固定 label 范围 |
| `global_map` | 是否允许进入持久全局八叉树 | 地形/静态类按需设为 `true`；动态/忽略必须为 `false` |

例如，新感知端把“行人”改为 label 80、颜色 `[10, 20, 30]`，只需修改对应条目：

```yaml
- label: 80
  name: person
  meaning: pedestrian from the new perception model
  rgb: [10, 20, 30]
  role: dynamic_obstacle
  global_map: false
```

未知 label/颜色由 `unknown.policy` 统一处理：

- `exclude`（推荐）：直接丢弃，不污染全局地图；
- `map_to_fallback`：映射到 `fallback_label`。该回退 label 必须在 `classes`
  中存在，并且必须是允许进入全局图的地形或静态类别。

`traversability` 段控制高代价覆盖。启用 `override_semantics` 后，只有本来已经
允许进入全局图的类别，且代价达到 `obstacle_threshold`，才会编码成
`obstacle_label`。动态和未知类别不会因为代价高而被提升成静态墙。
`defaults.initial_floor_label` 指定可选的机器人初始地面补丁使用哪个地形类别，
也必须指向 `global_map: true` 的 `terrain` 条目。

修改配置后需要重启相关节点；当前不支持运行时热更新。若要保留默认文件并使用
另一份 schema，可在启动时传入：

```bash
roslaunch semantic_octomap semantic_octomap_grid.launch \
  semantic_schema:=/absolute/path/to/my_semantic_schema.yaml
```

节点会在启动时校验重复 label/颜色、RGB 范围、角色、回退类别及高代价障碍类别；
配置不合法会直接报错退出，避免带着错误语义继续建图。

### Filtered local semantic grid input

The persistent tree can consume the 0.10 m robot-frame non-dynamic snapshot
produced by the local 3-D semantic voxel node. Launch with:

```bash
roslaunch semantic_octomap semantic_octomap_grid.launch \
  world_frame_id:=map_start
```

The launch starts `semantic_octomap_node`; it does not start the RGB-D semantic
sensor node or an adapter by default. The local 3-D voxel node publishes the
SSMI-ready `x y z rgb semantic_color` cloud directly on
`/semantic_pcl/global_admitted`. It has already removed dynamic labels 11-18,
normalized the simulation and bag inputs to one semantic encoding, and encoded
every point at or above the configured traversability threshold as an obstacle.
Consequently a simulation stair boundary may retain its terrain label in
`voxel_cloud` while entering SemanticOcTree as an obstacle. `rgb` and
`semantic_color` carry identical packed bits.

`grid_semantic_adapter_node` remains installed only for older raw-grid
producers. Set `use_grid_adapter:=true` to restore that compatibility path; do
not enable it when the current local voxel node is publishing the direct topic.

The input cloud is in `wuba_base` for the bag profile or `base_link` for the
simulation profile, with coordinates frozen at its acquisition stamp. ROS
message filtering transforms it into `map_start` at that same stamp. The input
snapshot uses a 0.10 m local grid, while the persistent SemanticOcTree keeps a
0.4 m resolution for real-time operation. The configuration uses
`input_mode: local_grid`, disables raycast clearing, and sets
`use_initial_pose_reference: false`; SemanticOcTree therefore does not publish
another `map -> map_start`. Missing cells and dynamic occlusion are not free
evidence.

Explicit revocation subscriptions are enabled for the local node's confirmed
`revoked_free` stream. The local node requires repeated positive evidence and
never emits an event merely because a voxel disappeared, left the rolling
window, or was dynamically occluded. Simulation may use raw depth-ray traversal
to expose a stale false-static trail; the processed `/grids_points` profile uses
only direct low-cost terrain contradiction. Repeated timer publications with
the same acquisition stamp are deduplicated. Ordinary OctoMap raycast clearing
remains disabled because the admitted cloud itself is a fused local grid rather
than a raw scan.

Topic names and compatibility settings are configurable in
`launch/semantic_octomap_grid.launch`; OctoMap policy remains in
`params/semantic_octomap_grid.yaml`.

After starting the recorded-data local voxel profile, run the direct
local-voxel-to-OctoMap visualization pipeline with:

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
panel. SemanticOcTree logs insertion, deduplication, reset, and TF statistics
for runtime verification.
