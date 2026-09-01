# Go2 robot_lab 策略隔离适配产物

本目录是一次“只新增文件、不修改现有 ROS/C++/URDF/SDF”的适配和验证。完整结论见 [VALIDATION_REPORT.md](VALIDATION_REPORT.md)。

## 主要文件

- `source_policy.pt`：`fan-ziqi/rl_sar` 的原始 robot_lab Go2 TorchScript 策略。
- `source_config.yaml`：与该策略配套的部署配置。
- `policy_robot_lab_native.onnx`：原生 `[1,45] -> [1,12]` ONNX。
- `actor_robot_lab_legacy_compat.onnx`：旧控制器 ABI 兼容模型，接口为 `obs[1,45] + history_obs[1,10,45] -> actions[1,12]`。
- `manifest.json`：来源、哈希、模型接口和映射常量。
- `test_report.json` / `TEST_REPORT.md`：离线数值和 C++ Runtime 测试。
- `validation_summary.json` / `VALIDATION_REPORT.md`：Gazebo A/B 结果和部署结论。
- `build_models.py`、`test_models.py`：构建与离线复测脚本。
- `simulation_probe.py`：短时 Gazebo 位姿、姿态和关节探针。
- `shadow_ws*`、`shadow_prefix`：测试时让原二进制加载指定模型的临时路径，不用于正式部署。
- `pd20_kd05`、`friction1_08`：只新增的 SDF/世界参数对照副本。

## 当前结论

模型格式、观测变换、关节顺序、默认姿态、动作缩放和旧滤波补偿均已验证；候选策略也能稳定行走。但它在平地直行时出现约 1.68 m 横移和 61° 偏航，明显差于原策略，因此当前不建议把兼容 ONNX 覆盖到原 `actor.onnx`。

原策略、控制器源码和语义模型 SDF 的测试后 SHA-256 已写入 `validation_summary.json`。
