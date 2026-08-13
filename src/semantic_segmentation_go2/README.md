# SSMI Go2 Fortress integration

Husky 与 Go2 的完整多终端启动顺序见工作空间源码根目录的
[`README.md`](../README.md)。本文件只保留 Go2 模型集成细节。

This package is parallel to `semantic_segmentation_husky`. It does not replace
the Husky model, launch files, TF script, palette, or OctoMap parameters.

## Build

```bash
cd ~/mapless_navigation/SSMI-example/ssmi_ws
catkin_make
source devel/setup.bash
```

## Start the Go2 semantic scene

```bash
roslaunch semantic_segmentation_go2 seg_go2.launch
```

Start the Go2-specific mapping configuration in another terminal:

```bash
source ~/mapless_navigation/SSMI-example/ssmi_ws/devel/setup.bash
roslaunch semantic_segmentation_go2 semantic_octomap_go2.launch
```

The selector launch keeps Husky as the default:

```bash
roslaunch semantic_segmentation_go2 seg_robot.launch robot_type:=husky
roslaunch semantic_segmentation_go2 seg_robot.launch robot_type:=go2
```

## Fortress RL control adapter

`seg_go2.launch` starts the supplied policy and Fortress adapter by default so
the effort-PD stance is active as soon as joint feedback becomes available.
Do not launch a second copy. For isolated debugging only, use:

```bash
roslaunch semantic_segmentation_go2 seg_go2.launch start_control:=false
roslaunch semantic_segmentation_go2 go2_rl_control.launch
```

Then start the original interactive keyboard in a separate terminal:

```bash
roslaunch robot_cmd keyboard_teleop.launch
```

The adapter converts Fortress joint feedback by joint name to the 12
`MotorState` topics used by `robot_control`. A native Fortress system evaluates
the original effort-PD equation every 1 ms physics step and applies bounded
joint torque; ROS only transports coherent target angles. It holds the nominal
stance at zero velocity, freezes
the measured pose when feedback is lost, synchronously blends all 12 joints
when starting or stopping, enforces physical joint limits, and latches a safety
stop if body tilt exceeds 35 degrees.
From zero, press a direction key twice to reach the 0.2 movement threshold;
press Space to stop.

## Standing geometry

- Spawn height: `0.34 m`, preventing initial foot / ground penetration.
- Settled base height: approximately `0.298 m`.
- Mapping `base_to_ground`: `0.298 m`.
- Floor alignment: `align_voxel_top=true`; occupied floor voxels stay below
  the physical surface and the first voxel above it is explicitly free.
- RGB-D and semantic cameras share the `camera_mount` frame. Change only the
  frame's `x y z roll pitch yaw` pose in `models/go2_semantic/model.sdf`; both
  sensors and the ROS optical TF use it automatically. The current pose is
  `[0.30, 0.0, 0.40] m`, pitched down by `0.40 rad`.

The model uses Fortress position controllers for all 12 joints. The separate
`go2_rl_control.launch` adapter supplies the Gazebo Classic-style motor-state,
IMU and motor-command interface expected by the original RL controller, while
leaving the Husky launch and control files unchanged.

## Regenerate the model

The committed `models/go2_semantic/model.sdf` was produced from the supplied
Go2 Xacro. Regenerate it after changing the source robot model with:

```bash
scripts/generate_go2_fortress_model.py \
  --xacro ../go2_gazebo-sim/Robot_ros/robots/go2_description/xacro/robot.xacro \
  --output models/go2_semantic/model.sdf \
  --ros-package-path ../go2_gazebo-sim/Robot_ros/robots:../velodyne_simulator
```
