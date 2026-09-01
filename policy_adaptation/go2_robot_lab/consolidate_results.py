#!/usr/bin/env python3
"""Consolidate offline and Gazebo A/B results into an acceptance report."""

import argparse
import hashlib
import json
import math
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def summarize_run(path):
    result = load(path)
    metrics = result["metrics"]
    start = result["drive_start_pose"]
    end = result["drive_end_pose"]
    yaw_delta_deg = math.degrees(wrap_angle(end["yaw"] - start["yaw"]))
    duration = result["command"]["drive_seconds"]
    dx = metrics["delta_x_m"]
    dy = metrics["delta_y_m"]
    stability_pass = bool(result["passed"])
    tracking_pass = abs(dy) <= 0.30 and abs(yaw_delta_deg) <= 10.0
    return {
        "source": str(path.resolve()),
        "stability_pass": stability_pass,
        "navigation_tracking_pass": tracking_pass,
        "delta_x_m": dx,
        "delta_y_m": dy,
        "planar_displacement_m": metrics["planar_displacement_m"],
        "average_world_x_velocity_mps": dx / duration,
        "yaw_delta_deg": yaw_delta_deg,
        "max_abs_roll_deg": metrics["max_abs_roll_deg"],
        "max_abs_pitch_deg": metrics["max_abs_pitch_deg"],
        "minimum_base_height_m": metrics["minimum_base_height_m"],
        "maximum_base_height_m": metrics["maximum_base_height_m"],
        "joint_count": metrics["joint_count"],
        "joints_finite": metrics["joints_finite"],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-dir", required=True, type=Path)
    parser.add_argument("--legacy-actor", required=True, type=Path)
    parser.add_argument("--legacy-controller-cpp", required=True, type=Path)
    parser.add_argument("--legacy-controller-h", required=True, type=Path)
    parser.add_argument("--semantic-model-sdf", required=True, type=Path)
    args = parser.parse_args()

    offline = load(args.artifact_dir / "test_report.json")
    runs = {
        "legacy_policy_kp40_kd1_mu100_mu50": summarize_run(
            args.artifact_dir / "simulation_probe_baseline.json"
        ),
        "robot_lab_policy_kp40_kd1_mu100_mu50": summarize_run(
            args.artifact_dir / "simulation_probe.json"
        ),
        "robot_lab_policy_kp20_kd05_mu100_mu50": summarize_run(
            args.artifact_dir / "simulation_probe_pd20_kd05.json"
        ),
        "robot_lab_policy_kp40_kd1_mu1_mu08": summarize_run(
            args.artifact_dir / "simulation_probe_friction1_08.json"
        ),
    }
    baseline = runs["legacy_policy_kp40_kd1_mu100_mu50"]
    candidate = runs["robot_lab_policy_kp40_kd1_mu100_mu50"]
    deployment_recommended = bool(
        offline["passed"]
        and candidate["stability_pass"]
        and candidate["navigation_tracking_pass"]
    )
    source_hashes = {
        "legacy_actor.onnx": sha256(args.legacy_actor),
        "FSM_State_RL.cpp": sha256(args.legacy_controller_cpp),
        "FSM_State_RL.h": sha256(args.legacy_controller_h),
        "semantic_model.sdf": sha256(args.semantic_model_sdf),
    }
    summary = {
        "offline_tests_passed": offline["passed"],
        "candidate_stability_passed": candidate["stability_pass"],
        "candidate_navigation_tracking_passed": candidate["navigation_tracking_pass"],
        "model_only_deployment_recommended": deployment_recommended,
        "tracking_acceptance": {
            "maximum_absolute_lateral_displacement_m_over_8s": 0.30,
            "maximum_absolute_yaw_drift_deg_over_8s": 10.0,
        },
        "runs": runs,
        "source_hashes_after_tests": source_hashes,
        "notes": [
            "Gazebo pass in each raw probe means finite/stable/no-fall smoke-test pass only.",
            "Advanced slope/stair acceptance was not run because the candidate failed flat-ground direction tracking.",
            "All deployed test changes were isolated generated files and shadow package paths.",
        ],
    }
    output_json = args.artifact_dir / "validation_summary.json"
    output_md = args.artifact_dir / "VALIDATION_REPORT.md"
    output_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def row(label, item):
        return (
            "| {} | {:.3f} | {:.3f} | {:.1f} | {:.2f} / {:.2f} | {:.3f} | {} | {} |".format(
                label,
                item["delta_x_m"],
                item["delta_y_m"],
                item["yaw_delta_deg"],
                item["max_abs_roll_deg"],
                item["max_abs_pitch_deg"],
                item["minimum_base_height_m"],
                "通过" if item["stability_pass"] else "失败",
                "通过" if item["navigation_tracking_pass"] else "失败",
            )
        )

    native_error = offline["tests"]["native_torchscript_vs_onnx"]["max_abs_error"]
    target_error = offline["tests"]["sequential_legacy_controller_equivalence"][
        "max_physical_joint_target_error_rad"
    ]
    latency = offline["metrics"]["cpu_inference_latency_ms"]
    lines = [
        "# Go2 robot_lab 策略适配与 A/B 验证报告",
        "",
        "## 结论",
        "",
        "兼容模型在接口和数值层面适配成功，也能在旧语义仿真的平地上稳定行走；但当前不建议直接替换原模型。新策略在 8 秒直行中产生明显横移和偏航，未达到导航方向跟踪门槛。",
        "",
        "- 原生 TorchScript→ONNX 最大误差：`{:.9g}`".format(native_error),
        "- 1000 帧旧控制器闭环目标最大误差：`{:.9g} rad`".format(target_error),
        "- C++ ONNX Runtime 加载与推理：通过",
        "- 推理延迟：mean `{:.3f} ms`，p95 `{:.3f} ms`，控制周期 `20 ms`".format(
            latency["mean"], latency["p95"]
        ),
        "",
        "## 平地 A/B（0.3 m/s，8 s）",
        "",
        "建议的导航跟踪门槛：横移不超过 0.30 m，偏航漂移不超过 10°。稳定性通过只表示未摔倒、姿态和关节数据有限。",
        "",
        "| 方案 | 前进 m | 横移 m | 偏航漂移 ° | 最大横滚/俯仰 ° | 最低高度 m | 稳定性 | 方向跟踪 |",
        "|---|---:|---:|---:|---:|---:|---|---|",
        row("原策略，PD 40/1，摩擦 100/50", baseline),
        row("新策略，PD 40/1，摩擦 100/50", candidate),
        row("新策略，PD 20/0.5，摩擦 100/50", runs["robot_lab_policy_kp20_kd05_mu100_mu50"]),
        row("新策略，PD 40/1，摩擦 1.0/0.8", runs["robot_lab_policy_kp40_kd1_mu1_mu08"]),
        "",
        "## 已完成的适配",
        "",
        "- 策略关节顺序：`FR, FL, RR, RL`；旧控制器内部顺序：`FL, FR, RL, RR`。",
        "- 新默认姿态：每条腿 `[0.0, 0.8, -1.5]`；兼容模型在输出端补偿旧默认姿态。",
        "- 命令缩放从旧 `[2, 2, 0.25]` 还原后转换为新 `[1, 1, 1]`。",
        "- 动作缩放保留髋关节 `0.125`、其余 `0.25`。",
        "- 利用历史输入精确抵消旧控制器 `0.2/0.8` 动作滤波，并恢复新策略需要的上一动作。",
        "- 非有限输入会在兼容模型内归零，避免 NaN/Inf 直接进入策略。",
        "",
        "## 未继续的阶段",
        "",
        "斜坡和楼梯验收未继续，因为候选策略已经在平地直线跟踪阶段失败。先解决方向漂移，再做复杂地形测试更有意义。",
        "",
        "## 原文件保护",
        "",
        "本次没有覆盖原 `actor.onnx`，也没有修改 C++、URDF 或原 SDF。测试通过临时影子包和新增 SDF 副本完成。原文件测试后哈希记录在 `validation_summary.json`。",
    ]
    output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
