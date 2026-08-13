# SSMI Semantic Octree 可通行性扩展设计记录

## 1. 文档目的

本文记录将连续可通行性（traversability）作为独立状态加入 SSMI 三维
Octree voxel 的讨论结果，供后续实现时直接参考。

本文是设计记录，不代表相关代码已经实现。实现时应重新核对上游字段、
规划器需求、地图分辨率以及旧地图兼容要求。

## 2. 当前输入与目标

当前上游话题 `/grids_points` 是 `sensor_msgs/PointCloud2`，主要字段为：

```text
x, y, z                 FLOAT32
intensity               FLOAT32
traversability          FLOAT32，范围 [0, 1]
semantic_lable          UINT32（保留上游现有拼写）
```

其中：

- `traversability = 0` 表示容易通行；
- `traversability = 1` 表示不可通行或障碍；
- `semantic_lable = UINT32_MAX` 表示没有有效语义标签；
- 语义类别和可通行性是同一个点上的两个独立观测量。

长期目标是在同一个地图 voxel 中同时回答三个问题：

```text
1. 这个空间是否被占用？
2. 这个表面或物体是什么语义类别？
3. 机器人经过这里的代价或难度是多少？
```

不能长期用“高可通行代价覆盖成 obstacle 语义”代替独立状态，因为例如
`road + traversability 1.0` 与 `wall + traversability 1.0` 的语义含义不同。

## 3. 总体设计决定

占用、语义和可通行性必须相互独立：

```cpp
struct TraversabilityState
{
    // 叶节点直接测量并经过时间融合后的代价。
    float measured_cost;

    // 观测可靠程度；0 可同时表示 unknown。
    float confidence;
    std::uint32_t observations;
    bool valid;

    // 以下是内部节点的空间摘要；叶节点可令它们等于 measured_cost。
    float max_cost;
    float mean_cost;
};

struct SemanticTraversabilityOcTreeNode
{
    // OccupancyOcTreeBase / ColorOcTreeNode 已有状态。
    float occupancy_log_odds;
    SemanticsLogOdds semantics;
    TraversabilityState traversability;
};
```

实际实现可将 cost 和 confidence 量化为 `uint8_t`，以减少内存占用并避免
用浮点完全相等作为合并条件：

```text
0   -> 0.0
255 -> 1.0
confidence == 0 -> unknown
```

是否量化应在确定精度和序列化格式后决定。

## 4. 父节点摘要使用 max、mean 还是 median

三种统计量都可以计算，但用途不同，不能认为它们等价。

### 4.1 最大值 max

```text
parent.max_cost = max(valid child costs)
```

优点：

- 是子区域风险的安全上界；
- 粗分辨率碰撞检查不会漏掉小面积真实障碍；
- 与“只要 footprint 内存在一个障碍就不可通过”的规则一致。

缺点：

- 对单个噪声值敏感；
- 不适合直接表示整片区域的平均行驶难度。

### 4.2 平均值 mean

```text
parent.mean_cost = sum(valid child cost * weight) / sum(weight)
```

优点：

- 能表达区域整体通行难度；
- 适合路径代价排序或信息增益计算。

缺点：

- 会稀释小面积障碍。例如 `[0.1, ..., 0.1, 0.9]` 的平均值为
  `0.2`，不能据此判定整个父区域安全；
- 增量更新时需要维护权重或有效子节点数量。

内部节点建议按有效子节点覆盖体积进行加权。八个直接子节点体积相同，
且都有效时可以使用算术平均。若 confidence 代表可靠性，也可以另行保存
confidence 加权均值，但不能因此丢弃安全上界。

### 4.3 中位数 median

优点：

- 对离群噪声有较强鲁棒性。

缺点：

- 会直接隐藏占少数的真实障碍；
- 从一个中位数无法增量恢复父节点的分布；
- 合并后无法重建子节点状态。

中位数可以作为诊断或去噪辅助量，但不应成为粗层碰撞判断的唯一值。

### 4.4 当前推荐

父节点至少保留两种摘要：

```text
max_cost  -> 粗层碰撞、可达性和安全约束
mean_cost -> 区域总体难度、路径排序
```

如果只允许保存一个值，面向安全规划时选择 `max`。噪声问题应由同帧
聚合、置信度、时间滤波或多次确认解决，而不是用平均值掩盖风险。

未知子节点不能默认视为 cost 0。查询时可以使用参数化策略：

```text
effective_max = max(max_valid_child_cost, unknown_cost if any child unknown)
```

## 5. 同一最小叶 voxel 内的点融合

多个原始点落进同一个最小叶 voxel 时，已经没有更细的 child 可以保留，
必须在当前分辨率内聚合。这不是 Octree prune。

例如：

```text
semantic:       [road, road, road, wall]
traversability: [0.1,  0.1,  0.1,  0.9]
```

推荐分别聚合：

```text
semantic       -> 按类别置信度累计投票，保留 Top-K 语义证据
traversability -> max 或高分位数；当前近似二值数据优先使用 max
RGB            -> 最近点或平均颜色，只用于显示
```

最终叶节点可以是：

```text
dominant semantic = road
semantic evidence = road 较强、wall 较弱
traversability    = 0.9
```

语义颜色可能仍显示 road，但规划必须依据独立的 traversability 将该 voxel
视作高代价或不可通行。

现有 `semantic_octomap` 在一帧内对每个 key 只选择一个代表点。扩展后不能
继续用“最近点”同时代表所有属性，应建立每帧临时聚合结构，例如：

```cpp
struct ScanVoxel
{
    std::unordered_map<std::uint32_t, float> semantic_votes;
    float max_traversability;
    float traversability_sum;
    std::uint32_t traversability_samples;
    bool has_traversability;
};
```

## 6. 叶节点跨时间融合

不建议用一次最大值永久覆盖历史状态。推荐危险快速上升、安全缓慢下降：

```cpp
const float alpha = measured >= stored ? rise_alpha : fall_alpha;
stored += alpha * (measured - stored);
```

初始建议参数：

```yaml
traversability_rise_alpha: 0.70
traversability_fall_alpha: 0.15
```

如果规划要求绝对保守，也可以使用时间窗口最大值，但必须配套过期和恢复
机制，否则一次噪声会形成永久障碍。

需要保留：

```text
valid
confidence
observations
last_observed（若需要时间衰减）
```

从而区分“明确观测为 cost 0”和“从未观测”。

## 7. 父节点更新与真正合并（prune）的区别

必须区分：

```text
更新父节点摘要 != 删除八个子节点
```

即使子节点不能合并，父节点仍然可以维护：

```text
parent.max_cost
parent.mean_cost
parent.valid/unknown coverage
```

这些摘要供粗分辨率查询使用，但不代表允许删除 children。

例如：

```text
child 0..6 cost = 0.1
child 7    cost = 0.9
```

应得到：

```text
parent.max_cost  = 0.9
parent.mean_cost = 0.2
children         = 全部保留
```

不能删除 children，否则重新拆分时无法知道只有 child 7 是高代价。

## 8. 真正合并（prune）条件

只有以下条件全部满足时才允许删除八个子节点：

```text
1. 八个 child 都存在且本身没有 children；
2. occupancy 满足原有可折叠条件；
3. semantic evidence 满足原有可折叠条件；
4. traversability valid/unknown 状态一致；
5. 有效 child 的 max_cost - min_cost <= merge_epsilon；
6. 若保存时间和 confidence，它们也满足规定的兼容条件。
```

建议参数：

```yaml
traversability_merge_epsilon: 0.05
```

示例：

| 子节点状态 | 是否允许 prune |
|---|---:|
| 全部 road，cost 全部为 0.1 | 可以 |
| road 与 wall 混合 | 不可以 |
| 语义相同，cost 从 0.1 到 0.9 | 不可以 |
| 语义相同，cost 从 0.20 到 0.23 | 可以，取决于 epsilon |
| valid 与 unknown 混合 | 不可以 |
| 全部 unknown 且 occupancy/semantic 可合并 | 可以 |

真正合并时父节点的安全值取 child 最大值，mean 取加权平均。由于只有差异
很小的 child 才允许合并，未来拆分时继承父状态产生的误差受 epsilon 限制。

## 9. 拆分（expand）规则

当一个已折叠父叶节点收到更细观测而需要拆分时：

```text
1. 创建八个 child；
2. child 继承父节点 occupancy、semantic 和 traversability 状态；
3. 只对本次观测命中的 child 进行新融合；
4. 自底向上重新计算父节点 max/mean 摘要。
```

由于父节点只有在 child 状态足够一致时才允许 prune，继承不会把明显不同
的高代价错误扩散到所有 child。

如果曾经用 `max` 强行合并差异很大的 child，拆分无法恢复原分布。因此
“父节点使用 max 做摘要”绝不等于“可以使用 max 后删除 child”。

## 10. Occupancy free ray 与 traversability

两种更新必须分开：

```text
free-space ray endpoint 之前的空间 -> 只更新 occupancy
真实表面 endpoint              -> 更新 occupancy、semantic、traversability
```

射线穿过一个 voxel 只证明空间为空，不证明某个地表容易通行。因此禁止：

```cpp
if (free_ray)
    traversability = 0;
```

如果 voxel 被确认为空中自由空间，可以令 traversability `valid=false`，而
不是将其设为明确可通行的 0。

动态障碍消失后，traversability 的降低应来自：

- 新的低代价地表 endpoint 观测；
- 多次确认；
- 明确定义的时间衰减或动态清理逻辑。

## 11. SemanticOcTree 的扩展方式

不建议原地改变现有 `SemanticOcTree` 的二进制布局。建议新增树类型：

```text
SemanticTraversabilityOcTree
```

原因是当前节点序列化顺序为：

```text
occupancy + color + semantics
```

扩展后将包含：

```text
occupancy + color + semantics + traversability state
```

如果仍沿用 `SemanticOcTree` 类型名，旧程序会用旧布局读取新数据，造成
字节错位。新树类型应单独注册，并为旧 `.ot` 文件保留旧读取路径；如有
需要，再提供显式地图转换工具。

以下位置都需要同步扩展：

- Node 构造、复制和相等比较；
- leaf observation update；
- `updateInnerOccupancy()` 的父节点摘要；
- `isNodeCollapsible()` 和 `pruneNode()`；
- expand 后的状态继承；
- `readData()` / `writeData()`；
- 保存、加载和树类型注册；
- 单元测试与旧格式兼容测试。

## 12. PointCloud2 适配

新增点类型或直接按 `PointCloud2` 字段读取：

```text
x, y, z
semantic_lable
traversability
```

不要先订阅 `view_point_cloud` 输出的 `/semantic_lable` 或 `/traversability`
可视化点云，因为那里原始字段已经被转换成 RGB。

输入应直接订阅 `/grids_points`，并完整继承其 `header.stamp` 和
`header.frame_id`，通过 TF 转换到 world/global frame。不要写死
`wuba_base`。

额外注意：

- 分类字段不可用 PCL `VoxelGrid` 做数值平均；
- `semantic_lable` 的 `UINT32_MAX` 作为 invalid；
- traversability 必须检查 finite，并裁剪到 `[0,1]`；
- 同帧先按地图 key 聚合，再对每个 voxel 做一次时间更新。

## 13. 发布和序列化兼容

建议继续发布两个用途不同的话题：

```text
/octomap_full
    新树的完整 occupancy + semantic + traversability 数据

/octomap_color
    标准 ColorOcTree 兼容负载，只包含 occupancy + color
```

`/octomap_color` 不能夹带额外 traversability 字节，否则标准 RViz 插件会
按 `ColorOcTree` 错误解析。

另外建议发布便于调试和规划消费的显式点云或 Marker：

```text
/traversability_voxels
/semantic_voxels
/semantic_traversability_voxel_cloud
```

最后一个点云可包含：

```text
label
semantic_confidence
measured_traversability
max_traversability
mean_traversability
traversability_confidence
observations
```

## 14. SSMI RLE 与规划接口

当前 `GetRLE` 的 `LE.msg` 只是 `float64[] le`，实际内容依赖固定位置：

```text
run_length + 4 个 semantic log-odds
```

不能无版本地在数组末尾追加 traversability，否则旧规划器会误解消息。

推荐保留旧 `GetRLE`，新增明确的版本化接口，例如：

```text
TraversabilityLE.msg
    uint32 run_length
    float64[4] semantic_log_odds
    float32 max_traversability
    float32 mean_traversability
    float32 confidence
    bool valid
```

也可以将语义 RLE 与 traversability RLE 分成两个服务，使只关心其中一种
数据的消费者保持简单。

规划器推荐用法：

```text
硬碰撞/可达性判断 -> max_traversability
软路径代价排序    -> mean_traversability 或叶节点 measured_cost
未知区域策略      -> 参数 unknown_cost
```

## 15. 分辨率和内存影响

traversability 差异会使以下区域更难 prune：

- 道路与墙壁交界；
- 平地与坡地交界；
- 动态障碍附近；
- 崎岖程度连续变化的地表。

因此树节点数量和内存占用会增加。这不是实现错误，而是保留空间风险细节
的代价。

上游 `/grids_points` 约为 `0.1 m` 网格。若 SSMI 地图使用 `0.4 m` 叶分辨率，
多个语义和不同 traversability 会先在同一最小 voxel 内不可逆融合。需要
根据算力和规划 footprint 在 `0.1~0.2 m` 范围评估叶分辨率，不能依靠
Octree 父子结构恢复叶分辨率以下的细节。

## 16. 推荐实施顺序

1. 定义独立 `TraversabilityState` 和新树类型，不修改旧树格式。
2. 增加 `/grids_points` 字段读取和 TF 转换。
3. 实现同帧 voxel 聚合：语义投票、traversability max/mean。
4. 实现叶节点危险快升、安全慢降的时间融合。
5. 实现父节点 `max_cost`、`mean_cost` 和 unknown coverage 摘要。
6. 将 traversability 条件加入 collapsible/prune 判断。
7. 实现 expand 继承以及自底向上的摘要刷新。
8. 实现新树序列化、保存/加载与旧格式兼容测试。
9. 保持 `/octomap_color` 的标准 ColorOcTree 负载。
10. 新增 traversability 可视化和版本化查询接口。
11. 修改规划器：硬约束用 max，软代价用 mean/leaf cost。
12. 用录制的 `/grids_points` bag 做回放、性能、内存和规划安全测试。

## 17. 必须覆盖的测试

- 同 voxel 的语义投票不会平均或破坏离散标签；
- 同 voxel 存在 `0.1` 和 `0.9` 时保留高风险；
- 危险代价上升快、安全代价下降慢；
- unknown 与明确 cost 0 可以区分；
- `[0.1 x 7, 0.9]` 更新父摘要但不 prune；
- 相近 cost 在 epsilon 内允许 prune；
- prune 后 expand 的误差不超过 epsilon；
- free ray 不会把地表 traversability 清成 0；
- 新树 save/load 完整保留 traversability；
- 旧 `SemanticOcTree` 地图仍可由旧类型读取；
- `/octomap_color` 仍可被标准 RViz 插件显示；
- RLE 新旧接口不会互相误读；
- bag 回放下地图更新耗时、节点数和内存满足要求。

## 18. 当前结论

长期使用 SSMI 作为全局地图后端时，将 traversability 独立加入 voxel 是
合理方案。最终原则如下：

```text
叶节点：语义和 traversability 分别融合。
父节点：max 表示安全上界，mean 表示总体难度。
合并：只有 occupancy、semantic、valid 状态及 cost 都足够一致才 prune。
拆分：继承仅来自曾经安全合并的父节点，新观测只更新命中的 child。
free ray：只更新 occupancy，不代表地表可通行。
格式：使用新的 Octree 类型和版本化规划接口，避免破坏旧系统。
```

## 19. 实现状态与使用入口

该设计已在 `feature/traversability-octree` 分支实现，旧
`SemanticOcTree` 类型和序列化布局未修改。新增入口为：

```bash
roslaunch semantic_octomap semantic_traversability_octomap.launch
```

节点直接订阅 `/grids_points`，通过消息时间戳对应的 TF 转到
`world_frame_id`，并发布：

```text
/octomap_full                         SemanticTraversabilityOcTree 完整负载
/octomap_color                        标准 ColorOcTree 兼容负载
/semantic_voxels                      语义颜色点云
/traversability_voxels                可通行性颜色点云
/semantic_traversability_voxel_cloud  数值字段点云
/occupancy_map_2D                     占用投影
/traversability_map_2D                max 可通行代价投影
/traversability_mean_map_2D           mean 可通行代价投影
```

原有 `querry_RLE` 保持不变，新接口为 `query_traversability_rle`，每个
`TraversabilityLE` 都带 `version=CURRENT_VERSION`，避免旧规划器误读。

规划器参数位于 `SSMI-Planning/params/exploration_params.yaml`：

```yaml
planning:
  traversability:
    enabled: true
    hard_threshold: 0.65
    soft_weight: 2.0
    unknown_cost: 0.5
```

规划时，max 达到硬阈值的栅格先转为碰撞障碍；其余位置使用 mean
作为 A* 的连续边代价。若没有收到与占用图几何对齐的可通行地图，则
自动退回原有仅占用规划行为。
