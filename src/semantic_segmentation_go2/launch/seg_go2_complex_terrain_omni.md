# Go2 复杂语义地形全向仿真启动说明

本文档对应 [`seg_go2_complex_terrain_omni.launch`](seg_go2_complex_terrain_omni.launch)。它是一个基于 ROS 1 与 Gazebo Fortress（Ignition Gazebo 6）的场景入口，用于一次性启动复杂地形世界、Unitree Go2 四足机器人、四向 RGB-D/语义相机、ROS–Ignition 数据桥、语义点云生成与融合、可选的强化学习运动控制以及 RViz。

## 1. 设计目的与初衷

该启动方案的目标不是只展示一个可行走的 Gazebo 世界，而是提供一个可重复的“运动—感知—语义—建图”测试前端，便于验证复杂户外环境中的无地图导航和语义地图算法。

设计时主要考虑了以下问题：

1. **在同一场景内覆盖多种导航难点。** 70 m × 70 m 世界同时包含连续起伏地形、八个主要/对角坡向、约 37° 的局部陡坡、草地、碎石、弯曲土路、山顶台阶、坡地迷宫、静态障碍和周期运动障碍，可用于检查算法在几何变化与语义变化叠加时的表现。
2. **提供像素严格对齐的语义真值。** 每个方向的 RGB、深度和语义相机共用相同外参、分辨率、视场角和裁剪距离，避免由传感器标定误差干扰对建图链路本身的验证。
3. **近似提供 360° 语义观测。** 前、左、后、右四组相机各覆盖 100° 水平视场，相邻方向有约 10° 重叠，消除名义上的水平盲区，减少四足机器人转身过程中后方或侧方信息丢失。
4. **保持场景可复现。** 世界和贴图由固定随机种子生成，生成后的 SDF 与资源直接提交到工程中；正常启动不依赖临时生成过程，重复实验面对相同的地形与障碍布局。
5. **复用通用四相机框架。** 当前 launch 只选择复杂地形世界并覆盖场景专属语义颜色，其余桥接、TF、点云和控制编排复用 `seg_go2_grass_omni.launch`，避免多个场景复制同一套节点后逐渐失配。

## 2. 系统边界

这个启动文件负责的是**仿真和感知前端**。默认启动后可以得到四路图像、四路语义点云及一帧合并的全向语义点云，也可以控制 Go2 运动。

需要特别区分以下边界：

- 语义图像由 Gazebo 的 segmentation sensor 和场景标签直接生成，是仿真真值，不是神经网络实时推理结果。
- 全向观测是四个相机视图的组合，不是旋转式或多线激光雷达模型；它不模拟激光束稀疏性、扫描运动畸变、通道布局或激光噪声。
- launch 默认不会启动 `local3d_semantic_voxel_map`、持久化 SemanticOcTree、路径规划器或自主导航控制层。
- `start_control:=true` 会启动 RL 控制器和 Fortress 适配器，但不会启动键盘节点。机器人是否迈步仍由外部 `/quad_cmd_vel` 指令决定。
- 输出语义点云包含 `x/y/z/rgb/semantic_color`，本身没有测得的 traversability 字段；下游复杂地形配置根据语义类别和邻域高度差计算代价。

## 3. 启动文件的分层结构

顶层文件将参数传给通用四相机启动文件：

```text
seg_go2_complex_terrain_omni.launch
└── seg_go2_grass_omni.launch
    ├── ros_ign_gazebo/ign_gazebo.launch
    ├── ROS–Ignition parameter_bridge
    ├── 4 × dynamic_semantic_remap.py
    ├── dynamic_tf_go2.py
    ├── 4 × sdf_sensor_tf.py
    ├── 4 × semantic_sensor_node.py              [start_semantic_cloud=true]
    ├── omni_semantic_cloud_merger.py             [start_semantic_cloud=true]
    ├── go2_rl_control.launch                     [start_control=true]
    └── RViz                                      [start_rviz=true]
```

顶层固定选择：

- 世界：`../sdf/world_complex_semantic_terrain_omni.sdf`
- 机器人：`../models/go2_semantic_omni/model.sdf`
- 场景调色板：`../config/label_colors_complex_terrain.yaml`

通用 launch 会先声明普通全向场景的调色板，顶层 launch 随后加载复杂地形调色板。ROS launch 的 XML 加载阶段会先完成参数写入再启动节点，因此最终 `/labels` 是复杂地形的 0–13 类调色板，四个语义重映射节点读取到的也是这份配置。

## 4. 仿真引擎与场景内容

Gazebo Fortress 启动参数为 `-r -v 1`，即加载世界后直接运行，控制台使用较低日志等级。世界采用 Ogre 2 渲染，物理步长为 1 ms，目标实时因子为 1.0，重力为 `0 0 -9.8 m/s²`。ROS 参数 `/use_sim_time` 被设置为 `true`，所有基于 ROS 时间的节点跟随 Gazebo `/clock`。

`IGN_GAZEBO_RESOURCE_PATH` 指向本包的 `models` 和 `sdf` 目录，使世界能够找到 Go2 模型、高度图、纹理和草地网格。`IGN_GAZEBO_SYSTEM_PLUGIN_PATH` 还包含工作空间的 `devel/lib`，用于加载 Go2 effort-PD 插件和 `semantic_segmentation_husky` 包提供的周期运动插件。匿名 `IGN_PARTITION` 将本次 Ignition Transport 通信与其他并行仿真实例隔离。

主地形使用 257 × 257 的 8 位高度图。生成脚本把目标世界高度编码为 `-1 + gray / 255 × 5 m`。由于 Fortress 的 ODE 高度场碰撞与 Ogre2 高度图渲染采用不同的灰度归一化和偏移路径，世界把二者拆成两个静态模型：`bare_earth_sloped_terrain_collision` 只负责物理接触，按 PNG 实际最大灰度计算垂直尺寸（当前为 `4.431373 m`）并通过模型位姿下移 `1 m`；`bare_earth_sloped_terrain` 只负责可视表面和语义标签，保持 `5 m` 垂直尺寸并在高度图内部下移 `1 m`。两套参数最终映射到同一个 `terrain_height()` 世界表面，草地贴合网格也使用相同高度。这样机器人的足端接触到的正是画面中的山坡表层，不会被隐藏碰撞面顶向空中，也不会陷入可视地面下方。

世界中的语义类别如下：

| ID | 类别 | 主要对象或区域 | 规范 RGB |
|---:|---|---|---|
| 0 | 未标注背景 | 未命中有效模型的像素 | `[0, 0, 0]` |
| 1 | 土路 | 场地中的弯曲路径 | `[139, 90, 43]` |
| 2 | 草地 | 北侧及山顶草地区域 | `[34, 139, 34]` |
| 3 | 碎石 | 东南碎石区域及 105 个物理小石块 | `[190, 190, 170]` |
| 4 | 边界 | 四周挡墙 | `[90, 90, 90]` |
| 5 | 裸土坡面 | 连续高度图主地形 | `[160, 110, 70]` |
| 6 | 动态障碍 | 周期运动的箱体、圆柱、球体和长杆，共 6 个 | `[255, 0, 255]` |
| 7 | 台阶 | 21 级、3.2 m 宽的山坡楼梯及顶部平台 | `[255, 165, 0]` |
| 8 | 天然岩石 | 10 个岩石障碍 | `[105, 105, 105]` |
| 9 | 长条静态障碍 | 木桩和梁状障碍 | `[255, 80, 0]` |
| 10 | 矩形静态障碍 | 箱子和块状障碍 | `[0, 90, 255]` |
| 11 | 圆形静态障碍 | 圆柱和球形障碍 | `[255, 220, 0]` |
| 12 | 植被 | 树干和树冠 | `[0, 170, 100]` |
| 13 | 迷宫墙 | 随坡面分段布置的 17 段通道墙 | `[150, 75, 0]` |

六个动态物体虽然在 SDF 中声明为静态模型，但由 `libssmi_periodic_motion_system.so` 按固定起点、终点、周期和相位做运动。这种方式用于生成确定性的动态遮挡和占据变化，不代表物体受完整动力学驱动。

## 5. 传感器设计

四组 RGB-D 与语义相机安装在同一机身高度，方向分别为前、左、后、右：

| 参数 | 值 |
|---|---|
| 相对 `base_link` 位置 | `[0, 0, 0.4] m` |
| 俯角 | `0.30 rad` |
| 水平朝向 | `0° / 90° / 180° / -90°` |
| 水平视场角 | `100°` |
| 图像尺寸 | `480 × 270` |
| 内参 | `fx=fy=201.384, cx=240, cy=135` |
| 深度范围 | `0.10–30.0 m` |
| 发布频率 | `10 Hz` |
| 配置的深度噪声 | `0.0` |

RGB-D 和语义传感器使用完全相同的安装 frame 与相机参数，所以同一方向的 RGB、深度、label 图和 colored semantic 图可以逐像素组合。四路点云生成器还启用了深度边缘过滤，拒绝绝对跳变大于 0.15 m、相对跳变大于 3% 或跨越语义边界的可疑深度点，以减轻物体边缘“拉丝”。

## 6. 数据流

### 6.1 感知与建图前端

```mermaid
flowchart LR
    W[Gazebo Fortress 世界<br/>物理、标签、周期运动] --> S[四向 RGB-D + 语义相机]
    S --> B[ros_ign_bridge]
    B --> R[四路语义颜色规范化<br/>dynamic_semantic_remap]
    B --> P[RGB + Depth]
    R --> C[四路 Colored Semantic]
    P --> G[4 × semantic_sensor_node]
    C --> G
    TF[world→base_link 动态 TF<br/>base_link→camera→optical 静态 TF] --> M[omni_semantic_cloud_merger]
    G -->|front / left / rear / right| M
    M --> O[/semantic_pcl/semantic_pcl<br/>base_link，约 10 Hz]
    O -. 可选下游 .-> V[局部 3D 语义体素地图]
    V -. 可选下游 .-> T[持久 SemanticOcTree / 规划]
```

具体过程如下：

1. 世界中每类模型的 `Label` 系统给语义相机提供整数 ID。Gazebo Fortress 的 label 图为 `rgb8`，同一个 `uint8` ID 重复写入 R、G、B 三个通道。
2. `parameter_bridge` 将 Ignition 图像、时钟、IMU、模型姿态和关节状态转换为 ROS 1 消息。四路原始语义话题被重映射到 `raw_*` 名称，避免与规范化输出重名。
3. 四个 `dynamic_semantic_remap.py` 分别同步原始 label 图和 colored 图。节点按 `/labels` 把所有像素重新着色，并把源 label 6 明确规范为 label 6、洋红色 `[255,0,255]`。该节点只按已有标签重映射，不执行运动检测。
4. 四个 `semantic_sensor_node.py` 各自同步 RGB、深度和规范化语义图，根据相机内参反投影为相机光学坐标系下的 `PointCloud2`。
5. `omni_semantic_cloud_merger.py` 在 0.04 s 容差内近似同步四路点云，通过 TF 转换到 `base_link`，保留原有点字段并拼接数据，使用四帧中最晚的时间戳发布一次合并结果。这样可防止下游按采集时间去重时只保留四个方向中的一个。
6. 合并点云是一维非组织点云，但保持四幅 `480 × 270` 图像的顺序，总点位布局等价于 `480 × 1080`。配套复杂地形体素配置据此以 4 × 4 像素步长采样。

### 6.2 TF 数据流

TF 树的关键部分为：

```text
world
└── base_link                              动态：来自 /model/<model_name>/pose
    ├── rgbd_camera_link
    │   └── rgbd_camera_optical_frame
    ├── rgbd_left_link
    │   └── rgbd_left_optical_frame
    ├── rgbd_rear_link
    │   └── rgbd_rear_optical_frame
    └── rgbd_right_link
        └── rgbd_right_optical_frame
```

`world → base_link` 由机器人模型姿态实时广播；四组相机静态外参直接从 `model.sdf` 的 `camera_mount*` frame 读取。光学 frame 遵守 ROS 相机坐标约定（Z 向前、X 向右、Y 向下）。修改模型相机安装位姿后，RGB-D、语义相机和 ROS TF 会使用同一个 SDF 数据源。

### 6.3 运动控制数据流

```text
Gazebo joint_state + /trunk_imu + model pose
    ↓ ros_ign_bridge
go2_fortress_control_adapter.py
    ↓ 12 路 MotorState
robot_control（ONNX RL policy）
    ↓ 12 路 MotorCmd
go2_fortress_control_adapter.py
    ↓ 12 路 /model/<model_name>/joint/<joint>/cmd_pos
ros_ign_bridge → Go2EffortPdSystem → 关节力矩

外部键盘或导航控制器 → /quad_cmd_vel ────────────────┘
```

适配器以 500 Hz 发布一组一致的 12 关节目标，并负责起停插值、关节限位、指令/状态超时保护和 35° 机身倾角锁存急停。没有新鲜的 `/quad_cmd_vel` 运动请求时，机器人保持标称站姿。

## 7. 主要 ROS 话题

四个方向的话题对应关系如下：

| 数据 | 前 | 左 | 后 | 右 |
|---|---|---|---|---|
| RGB | `/rgbd_camera/image` | `/rgbd_left/image` | `/rgbd_rear/image` | `/rgbd_right/image` |
| Depth | `/rgbd_camera/depth_image` | `/rgbd_left/depth_image` | `/rgbd_rear/depth_image` | `/rgbd_right/depth_image` |
| CameraInfo | `/rgbd_camera/camera_info` | `/rgbd_left/camera_info` | `/rgbd_rear/camera_info` | `/rgbd_right/camera_info` |
| 原始 label | `/semantic/raw_labels_map` | `/semantic_left/raw_labels_map` | `/semantic_rear/raw_labels_map` | `/semantic_right/raw_labels_map` |
| 规范 label | `/semantic/labels_map` | `/semantic_left/labels_map` | `/semantic_rear/labels_map` | `/semantic_right/labels_map` |
| 规范语义颜色 | `/semantic/colored_map` | `/semantic_left/colored_map` | `/semantic_rear/colored_map` | `/semantic_right/colored_map` |
| 单向语义点云 | `/semantic_pcl/front` | `/semantic_pcl/left` | `/semantic_pcl/rear` | `/semantic_pcl/right` |

其他关键话题：

| 话题 | 类型/用途 |
|---|---|
| `/clock` | Gazebo 仿真时间 |
| `/trunk_imu` | Go2 机身 IMU |
| `/model/go2_semantic_omni/pose` | 机器人世界位姿 |
| `/world/semantic_segmentation_world/model/go2_semantic_omni/joint_state` | 12 关节反馈 |
| `/semantic_pcl/semantic_pcl` | `base_link` 下的四向合并语义点云，主要感知输出 |
| `/quad_cmd_vel` | Go2 键盘/上层速度指令输入 |

## 8. 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ign_partition` | 匿名唯一名称 | 隔离本次 Ignition Transport 网络，通常无需修改 |
| `model_name` | `go2_semantic_omni` | 同时影响姿态、关节、控制话题及模型 SDF 路径；不要在不修改世界模型名时单独更改 |
| `base_frame` | `base_link` | TF 和合并点云的目标机身坐标系 |
| `start_semantic_cloud` | `true` | 是否启动四路图像转点云和点云融合；设为 false 时图像仍会发布 |
| `start_rviz` | `true` | 是否启动预配置 RViz |
| `start_control` | `true` | 是否启动 Fortress 控制适配器和 RL 控制器 |

## 9. 使用方法

### 9.1 编译与环境

首次使用或修改 C++ 插件后，在工作空间根目录编译：

```bash
cd /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws
catkin_make
source devel/setup.bash
```

以后每个新终端都需要执行 `source devel/setup.bash`。

### 9.2 默认启动

```bash
cd /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_go2 seg_go2_complex_terrain_omni.launch
```

默认行为是：Gazebo 自动开始运行、四路语义点云开启、RL 控制链开启、RViz 开启。

### 9.3 常见裁剪启动

只检查世界和原始图像，降低 CPU/GPU 占用：

```bash
roslaunch semantic_segmentation_go2 seg_go2_complex_terrain_omni.launch \
  start_semantic_cloud:=false start_control:=false start_rviz:=false
```

生成点云，但不加载运动控制和 RViz：

```bash
roslaunch semantic_segmentation_go2 seg_go2_complex_terrain_omni.launch \
  start_semantic_cloud:=true start_control:=false start_rviz:=false
```

### 9.4 键盘控制

保持主 launch 运行，在另一个已经 source 工作空间的终端启动：

```bash
roslaunch robot_cmd keyboard_teleop.launch
```

常用按键为 `w/s` 前后、`a/d` 转向、`j/l` 横移、空格停止。每次按键改变 0.1，适配器运动阈值为 0.19，因此从零速开始通常需要连续按两次同一方向键达到 0.2。不要重复启动 `go2_rl_control.launch`，因为主 launch 在 `start_control:=true` 时已经启动它。

### 9.5 可选的局部语义体素地图

若环境中已经安装并 source 了 `local3d_semantic_voxel_map` 包，可在第二个终端启动复杂地形专属配置：

```bash
roslaunch local3d_semantic_voxel_map semantic_voxel_map.launch \
  config:=$(rospack find semantic_segmentation_go2)/config/semantic_voxel_map_complex_terrain.yaml \
  input_topic:=/semantic_pcl/semantic_pcl
```

该配置把 0–13 类映射为地形或障碍代价，在 0.10 m × 0.10 m × 0.05 m 体素中融合语义，并启用 0.15 m 邻域高度差推断。它可发布去除动态类后的 `/semantic_pcl/global_admitted`，供持久化 SemanticOcTree 使用。此下游包不属于当前 launch 的自动启动范围。

## 10. 运行检查与排错

启动后可先检查以下频率：

```bash
rostopic hz /clock
rostopic hz /rgbd_camera/depth_image
rostopic hz /semantic/colored_map
rostopic hz /semantic_pcl/front
rostopic hz /semantic_pcl/semantic_pcl
```

正常情况下，相机、单向点云和合并点云都接近 10 Hz。进一步检查：

```bash
rosparam get /labels
rostopic echo -n 1 /semantic_pcl/semantic_pcl/header
rosrun tf tf_echo world base_link
rosrun tf tf_echo base_link rgbd_camera_optical_frame
```

常见问题：

- **Gazebo 找不到模型、贴图或网格：** 确认从本工作空间 source 后使用 `roslaunch`，不要绕过 launch 直接加载 SDF；资源路径由 launch 设置。
- **周期运动插件或 Go2 PD 插件加载失败：** 重新执行 `catkin_make`，确认 `devel/lib` 中存在对应 `.so`，并检查 `semantic_segmentation_husky` 包是否可被 `rospack find` 找到。
- **有图像但没有合并点云：** 确认 `start_semantic_cloud` 为 true，并检查四个方向是否都在发布。融合器要求四路字段布局一致、TF 可用且时间差不超过 0.04 s；任一方向缺失都会跳过该次融合。
- **语义颜色不符合复杂地形类别：** `rosparam get /labels` 应包含 0–13；不要同时启动另一个会覆盖全局 `/labels` 的语义场景。
- **机器人站立但不走：** 主 launch 不启动键盘；检查 `/quad_cmd_vel` 是否持续更新。若倾角超过 35°，安全急停会锁存，需要重置仿真并重启控制 launch。
- **机器人悬空或陷入可视地面：** 重新运行第 11 节的场景生成脚本，确认世界文件同时包含独立的 `bare_earth_sloped_terrain_collision` 物理模型和 `bare_earth_sloped_terrain` 可视语义模型。物理层应使用自动计算的垂直尺寸和模型级 `-1 m` 偏移；可视层应使用 `5 m` 垂直尺寸和高度图内部 `-1 m` 偏移。修改后必须完全退出并重启 Gazebo，运行中的世界不会热加载 SDF。
- **系统负载过高：** 优先关闭 RViz；仅调试场景时关闭语义点云和控制。四组 RGB-D 加四组语义相机在 10 Hz 下仍会占用明显的 GPU 和 CPU。
- **同一 ROS Master 中数据互相污染：** 不要并行启动其他发布 `/clock`、同名相机话题或全局 `/labels` 的场景。`IGN_PARTITION` 只隔离 Ignition Transport，不隔离 ROS Master。

## 11. 场景再生成与维护

场景资源由固定种子脚本生成。修改地形函数、障碍布局或语义对象后，可在工作空间根目录执行：

```bash
python3 src/semantic_segmentation_go2/scripts/generate_complex_semantic_terrain.py
```

该命令会更新复杂地形 SDF、高度图、表面纹理、法线图和贴合高度的草地网格。生成后应重新启动 Gazebo，并至少核对以下一致性：

1. SDF 中模型的 label ID 与 `label_colors_complex_terrain.yaml` 一致；
2. `semantic_voxel_map_complex_terrain.yaml` 中的 RGB→label 映射与调色板一致；
3. 世界内机器人名称仍为 `go2_semantic_omni`；
4. 四组 RGB-D/语义相机参数和 `semantic_cloud_go2_omni.yaml` 内参一致；
5. 若改变相机数量、尺寸或点云顺序，同步修改点云融合器输入和下游 `input_image_width/input_image_height`。

关闭系统时，建议先停止规划/建图等下游节点，再停止键盘和控制，最后在主 launch 终端按 `Ctrl-C` 关闭 RViz 与 Gazebo。
