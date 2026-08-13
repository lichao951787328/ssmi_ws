# SSMI Husky 与 Go2 启动说明

本文档说明同一工作空间内 Husky 轮式机器人和 Unitree Go2 四足机器人
的启动方法。两套仿真配置彼此独立，请勿在同一个 ROS Master 中同时启动。

## 1. 编译与终端环境

首次使用或修改源码后，在一个终端中编译：

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
catkin_make
```

下面每打开一个新终端，都需要先执行：

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
```

如果上一次仿真没有正常退出，应先确认没有残留的 `roslaunch`、Gazebo
Fortress 或 ROS 节点，避免两个 `/clock`、TF 或语义图像发布者相互冲突。

## 2. Husky 全套启动方式

Husky 的完整系统使用四个终端。建议先等场景和相机正常发布，再依次启动
建图、规划和底盘控制。

### 终端 1：Husky 仿真、语义相机、动态障碍和 RViz

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_husky seg_husky.launch
```

### 终端 2：Husky 语义点云和语义八叉树

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_octomap semantic_octomap.launch
```

注意：正确的 ROS 包名是 `semantic_octomap`，不是 `semantioctomap`。

### 终端 3：语义探索和路径规划

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_info_gathering run_semantic_exploration.launch
```

### 终端 4：Husky 路径跟踪和轮式底盘控制

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_husky jackal_pd.launch
```

该节点将 `/planner/position_cmd` 转换为
`/model/husky_seg_cam/cmd_vel`。它只适用于 Husky，不得用于 Go2。

## 3. Go2 全套启动方式

Go2 使用独立的 Fortress 模型、TF、语义颜色和 OctoMap 参数，不会加载
Husky 的 `base_to_ground` 或轮式控制器。

### 终端 1：Go2 仿真、语义相机、动态障碍和 RViz

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_go2 seg_go2.launch
```

Go2 在 Fortress 中的生成高度为 `0.34 m`，稳定站立后的 `base_link`
离地约 `0.298 m`。相机安装在机身前部并向下倾斜 `0.20944 rad`。
该命令默认同时启动Go2 RL控制器和Fortress原生effort-PD适配层，避免机器人在
控制器启动前失去站姿。不要再重复运行`go2_rl_control.launch`。

### 终端 2：Go2 语义点云和语义八叉树

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_go2 semantic_octomap_go2.launch
```

该启动文件使用 Go2 专属参数：

- 八叉树全局坐标系：`world`
- 点云坐标系：`camera_optic`
- 脚底盲区参考坐标系：`base_link`
- 脚底覆盖范围：`1.2 x 0.8 m`
- `base_to_ground`：`0.298 m`
- 地面以上清空高度：`1.0 m`
- `align_voxel_top`：`true`

### 终端 3：语义探索和路径规划

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_info_gathering run_semantic_exploration.launch
```

探索节点可以读取 Go2 发布的地图并产生 `/planner/position_cmd`。当前新增的
适配层接收键盘话题 `/quad_cmd_vel`；规划器位置指令若要自动驱动 Go2，仍需
再增加 `/planner/position_cmd -> /quad_cmd_vel` 的导航控制层。

### Go2 RL控制器和Fortress适配层（已由终端1启动）

`seg_go2.launch`内部已经包含以下启动文件：

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_go2 go2_rl_control.launch
```

不要再次手工执行上述命令，否则会产生重名控制节点。如需仅调试场景、不启动
控制，可显式使用`seg_go2.launch start_control:=false`，随后再手动启动它。

适配层在速度为零时保持稳定站姿；开始和停止时对整组12关节同步插值，避免
破坏步态相位；键盘消息超时或控制器失联时保持安全姿态；机身滚转或俯仰超过
`35 deg`时锁存急停。触发姿态急停后，应先重置仿真，然后重启本启动文件。

不要为 Go2 运行 `semantic_segmentation_husky jackal_pd.launch`，它只会发布
Husky 的轮式 `/cmd_vel`。

### 终端 4：Go2 键盘控制

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch robot_cmd keyboard_teleop.launch
```

常用按键：`w/s` 前后、`a/d` 转向、`j/l` 横移、空格立即把三轴速度归零。
每按一次速度增减 `0.1`；安全适配层的运动阈值为 `0.19`，因此从静止开始需按
两次方向键达到 `0.2` 才开始迈步。建议首次测试只使用 `0.2` 的低速，停止时
按空格，不要直接关闭键盘终端。

## 4. 使用参数选择机器人

也可以通过统一入口选择场景机器人。默认仍为 Husky：

```bash
# Husky
roslaunch semantic_segmentation_go2 seg_robot.launch robot_type:=husky

# Go2
roslaunch semantic_segmentation_go2 seg_robot.launch robot_type:=go2
```

统一入口只替代两套系统的“终端 1”命令。终端 2 的 OctoMap 配置仍必须按
机器人分别选择，不能让 Go2 加载 Husky 的脚底高度参数。

## 5. 关键输出检查

场景和建图启动后，可使用以下命令检查数据链：

```bash
rostopic hz /rgbd_camera/depth_image
rostopic hz /semantic/colored_map
rostopic hz /semantic_pcl/semantic_pcl
rostopic echo -n 1 /octomap_color/id
rostopic echo -n 1 /octomap_full/id
```

预期结果：

```text
/rgbd_camera/depth_image    约 10 Hz
/semantic/colored_map       约 10 Hz
/semantic_pcl/semantic_pcl  约 10 Hz
/octomap_color/id           ColorOcTree
/octomap_full/id            SemanticOcTree
```

## 6. 正常关闭

分别在各个启动终端中按 `Ctrl-C`。应先关闭规划和控制，再关闭 OctoMap，最后
关闭仿真场景。Go2 OctoMap 默认在退出时保存到：

```text
/tmp/go2_semantic_map.ot
```
