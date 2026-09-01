#!/usr/bin/env python3
"""Offline verification for the native and legacy-compatible Go2 policies."""

import argparse
import hashlib
import json
import math
import subprocess
import time
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch


PERM = np.asarray([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8], dtype=np.int64)
OLD_DEFAULT = np.asarray(
    [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5],
    dtype=np.float32,
)
NEW_DEFAULT = np.asarray([0.0, 0.8, -1.5] * 4, dtype=np.float32)
SCALE_OLD = np.asarray([0.125, 0.25, 0.25] * 4, dtype=np.float32)
SCALE_NEW = np.asarray([0.125, 0.25, 0.25] * 4, dtype=np.float32)
POSTURE_OFFSET_OLD = (NEW_DEFAULT[PERM] - OLD_DEFAULT) / SCALE_OLD


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def safe_clip(value):
    return np.clip(np.nan_to_num(value, nan=0.0, posinf=0.0, neginf=0.0), -100.0, 100.0).astype(np.float32)


def policy_observation_reference(old_obs, history_obs):
    old_obs = safe_clip(old_obs)
    history_obs = safe_clip(history_obs)
    batch = old_obs.shape[0]
    result = np.empty((batch, 45), dtype=np.float32)
    result[:, 0:6] = old_obs[:, 0:6]
    result[:, 6:9] = old_obs[:, 6:9] * np.asarray([0.5, 0.5, 4.0], dtype=np.float32)
    result[:, 9:21] = (old_obs[:, 9:21] + OLD_DEFAULT)[..., PERM] - NEW_DEFAULT
    result[:, 21:33] = old_obs[:, 21:33][..., PERM]

    z_previous = old_obs[:, 33:45]
    z_previous_previous = history_obs[:, -1, 33:45]
    filtered_previous = 0.2 * z_previous_previous + 0.8 * z_previous
    previous_policy_action = (filtered_previous - POSTURE_OFFSET_OLD)[..., PERM]
    startup = (
        np.sum(np.abs(z_previous), axis=1, keepdims=True)
        + np.sum(np.abs(z_previous_previous), axis=1, keepdims=True)
    ) < 1.0e-12
    result[:, 33:45] = np.where(startup, 0.0, previous_policy_action)
    return result


def compatibility_output_reference(old_obs, history_obs, native_session):
    clean_old_obs = safe_clip(old_obs)
    policy_obs = policy_observation_reference(old_obs, history_obs)
    policy_action = native_session.run(["output"], {"input": policy_obs})[0]
    policy_action = np.where(np.isfinite(policy_action), policy_action, 0.0).astype(np.float32)
    desired_old = policy_action[..., PERM] + POSTURE_OFFSET_OLD
    wrapper_action = (desired_old - 0.2 * clean_old_obs[:, 33:45]) / 0.8
    return wrapper_action.astype(np.float32), policy_obs, policy_action


def assert_close(name, actual, expected, atol, report):
    difference = np.abs(actual - expected)
    max_abs = float(np.max(difference))
    mean_abs = float(np.mean(difference))
    passed = bool(np.allclose(actual, expected, rtol=0.0, atol=atol))
    report[name] = {
        "passed": passed,
        "max_abs_error": max_abs,
        "mean_abs_error": mean_abs,
        "absolute_tolerance": atol,
    }
    if not passed:
        raise AssertionError("{} max error {} exceeds {}".format(name, max_abs, atol))


def interface(session):
    return {
        "inputs": [(x.name, x.shape, x.type) for x in session.get_inputs()],
        "outputs": [(x.name, x.shape, x.type) for x in session.get_outputs()],
    }


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-dir", required=True, type=Path)
    parser.add_argument("--source-policy", required=True, type=Path)
    parser.add_argument("--legacy-actor", required=True, type=Path)
    parser.add_argument("--cpp-smoke-binary", type=Path)
    args = parser.parse_args()

    native_path = args.artifact_dir / "policy_robot_lab_native.onnx"
    compat_path = args.artifact_dir / "actor_robot_lab_legacy_compat.onnx"
    manifest_path = args.artifact_dir / "manifest.json"
    report_path = args.artifact_dir / "test_report.json"
    markdown_path = args.artifact_dir / "TEST_REPORT.md"

    report = {"passed": False, "tests": {}, "metrics": {}}
    with open(manifest_path, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)

    if sha256(args.source_policy) != manifest["source"]["policy_sha256"]:
        raise AssertionError("Source policy hash differs from manifest")
    report["tests"]["source_policy_hash"] = {"passed": True, "sha256": sha256(args.source_policy)}
    report["tests"]["legacy_actor_unchanged"] = {
        "passed": sha256(args.legacy_actor) == "eff13a0b9219e30892c3b979251dea804030a0c874c2da7fd19a75cba67f9b93",
        "sha256": sha256(args.legacy_actor),
    }
    if not report["tests"]["legacy_actor_unchanged"]["passed"]:
        raise AssertionError("Legacy actor hash changed from recorded baseline")

    for path in [native_path, compat_path]:
        onnx.checker.check_model(onnx.load(str(path)))
    report["tests"]["onnx_checker"] = {"passed": True}

    native = ort.InferenceSession(str(native_path), providers=["CPUExecutionProvider"])
    compat = ort.InferenceSession(str(compat_path), providers=["CPUExecutionProvider"])
    expected_native_interface = {
        "inputs": [("input", [1, 45], "tensor(float)")],
        "outputs": [("output", [1, 12], "tensor(float)")],
    }
    expected_compat_interface = {
        "inputs": [
            ("obs", [1, 45], "tensor(float)"),
            ("history_obs", [1, 10, 45], "tensor(float)"),
        ],
        "outputs": [("actions", [1, 12], "tensor(float)")],
    }
    actual_native_interface = interface(native)
    actual_compat_interface = interface(compat)
    interface_ok = (
        actual_native_interface == expected_native_interface
        and actual_compat_interface == expected_compat_interface
    )
    report["tests"]["model_interfaces"] = {
        "passed": interface_ok,
        "native": actual_native_interface,
        "legacy_compatible": actual_compat_interface,
    }
    if not interface_ok:
        raise AssertionError("Unexpected ONNX interface")

    torch_policy = torch.jit.load(str(args.source_policy), map_location="cpu")
    torch_policy.eval()
    rng = np.random.default_rng(20260827)
    native_cases = [
        np.zeros((1, 45), dtype=np.float32),
        np.concatenate(
            [
                np.asarray([[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]], dtype=np.float32),
                np.zeros((1, 39), dtype=np.float32),
            ],
            axis=1,
        ),
    ]
    for _ in range(256):
        value = np.zeros((1, 45), dtype=np.float32)
        value[:, 0:3] = rng.uniform(-1.5, 1.5, (1, 3))
        gravity = rng.normal(size=(1, 3)).astype(np.float32)
        gravity /= np.linalg.norm(gravity, axis=1, keepdims=True)
        value[:, 3:6] = gravity
        value[:, 6:9] = rng.uniform(-1.0, 1.0, (1, 3))
        value[:, 9:21] = rng.uniform(-1.0, 1.0, (1, 12))
        value[:, 21:33] = rng.uniform(-2.0, 2.0, (1, 12))
        value[:, 33:45] = rng.uniform(-5.0, 5.0, (1, 12))
        native_cases.append(value)
    native_batch = np.concatenate(native_cases, axis=0)
    torch_outputs = []
    onnx_outputs = []
    with torch.no_grad():
        for row in native_batch:
            row_batch = row.reshape(1, 45)
            torch_outputs.append(torch_policy(torch.from_numpy(row_batch)).numpy())
            onnx_outputs.append(native.run(["output"], {"input": row_batch})[0])
    assert_close(
        "native_torchscript_vs_onnx",
        np.concatenate(onnx_outputs),
        np.concatenate(torch_outputs),
        2.0e-5,
        report["tests"],
    )

    old_obs_cases = []
    history_cases = []
    startup_obs = np.zeros((1, 45), dtype=np.float32)
    startup_obs[:, 3:6] = [0.0, 0.0, -1.0]
    old_obs_cases.append(startup_obs)
    history_cases.append(np.zeros((1, 10, 45), dtype=np.float32))
    for _ in range(256):
        old_obs = rng.uniform(-4.0, 4.0, (1, 45)).astype(np.float32)
        old_obs[:, 3:6] = rng.normal(size=(1, 3)).astype(np.float32)
        history = rng.uniform(-4.0, 4.0, (1, 10, 45)).astype(np.float32)
        old_obs_cases.append(old_obs)
        history_cases.append(history)

    compat_outputs = []
    compat_references = []
    for old_obs, history in zip(old_obs_cases, history_cases):
        compat_outputs.append(
            compat.run(["actions"], {"obs": old_obs, "history_obs": history})[0]
        )
        reference, _, _ = compatibility_output_reference(old_obs, history, native)
        compat_references.append(reference)
    assert_close(
        "compatibility_wrapper_vs_numpy_reference",
        np.concatenate(compat_outputs),
        np.concatenate(compat_references),
        3.0e-5,
        report["tests"],
    )

    # Sequentially reproduce the unchanged controller's observation/history,
    # filter, default pose, action scaling, and final physical joint remapping.
    z_previous = np.zeros((1, 12), dtype=np.float32)
    z_previous_previous = np.zeros((1, 12), dtype=np.float32)
    previous_policy_action = np.zeros((1, 12), dtype=np.float32)
    target_errors = []
    wrapper_errors = []
    previous_action_feature_errors = []
    maximum_wrapper_action = 0.0
    for step in range(1000):
        old_obs = np.zeros((1, 45), dtype=np.float32)
        old_obs[:, 0:3] = rng.uniform(-1.0, 1.0, (1, 3))
        gravity = rng.normal(size=(1, 3)).astype(np.float32)
        gravity /= np.linalg.norm(gravity, axis=1, keepdims=True)
        old_obs[:, 3:6] = gravity
        old_obs[:, 6:9] = rng.uniform(-2.0, 2.0, (1, 3))
        old_obs[:, 9:21] = rng.uniform(-0.8, 0.8, (1, 12))
        old_obs[:, 21:33] = rng.uniform(-1.5, 1.5, (1, 12))
        old_obs[:, 33:45] = z_previous
        history = np.zeros((1, 10, 45), dtype=np.float32)
        history[:, -1, 33:45] = z_previous_previous

        wrapper_output = compat.run(
            ["actions"], {"obs": old_obs, "history_obs": history}
        )[0]
        reference_output, transformed_obs, policy_action = compatibility_output_reference(
            old_obs, history, native
        )
        wrapper_errors.append(float(np.max(np.abs(wrapper_output - reference_output))))
        expected_previous_feature = (
            np.zeros_like(previous_policy_action) if step == 0 else previous_policy_action
        )
        previous_action_feature_errors.append(
            float(np.max(np.abs(transformed_obs[:, 33:45] - expected_previous_feature)))
        )

        filtered_old = 0.2 * z_previous + 0.8 * wrapper_output
        old_target = OLD_DEFAULT + SCALE_OLD * filtered_old
        physical_target = old_target[:, PERM]
        expected_physical_target = NEW_DEFAULT + SCALE_NEW * policy_action
        target_errors.append(float(np.max(np.abs(physical_target - expected_physical_target))))
        maximum_wrapper_action = max(maximum_wrapper_action, float(np.max(np.abs(wrapper_output))))

        z_previous_previous = z_previous.copy()
        z_previous = wrapper_output.copy()
        previous_policy_action = policy_action.copy()

    sequential_ok = (
        max(wrapper_errors) < 3.0e-5
        and max(previous_action_feature_errors) < 3.0e-5
        and max(target_errors) < 3.0e-5
    )
    report["tests"]["sequential_legacy_controller_equivalence"] = {
        "passed": sequential_ok,
        "steps": 1000,
        "max_wrapper_reference_error": max(wrapper_errors),
        "max_previous_action_feature_error": max(previous_action_feature_errors),
        "max_physical_joint_target_error_rad": max(target_errors),
        "maximum_absolute_wrapper_action": maximum_wrapper_action,
    }
    if not sequential_ok:
        raise AssertionError("Sequential compatibility test failed")

    abnormal_obs = np.zeros((1, 45), dtype=np.float32)
    abnormal_history = np.zeros((1, 10, 45), dtype=np.float32)
    abnormal_obs[0, 0] = np.nan
    abnormal_obs[0, 10] = np.inf
    abnormal_obs[0, 24] = -np.inf
    abnormal_history[0, -1, 33] = np.nan
    abnormal_output = compat.run(
        ["actions"], {"obs": abnormal_obs, "history_obs": abnormal_history}
    )[0]
    abnormal_ok = bool(np.all(np.isfinite(abnormal_output)))
    report["tests"]["nonfinite_input_guard"] = {
        "passed": abnormal_ok,
        "output_all_finite": abnormal_ok,
        "maximum_absolute_output": float(np.max(np.abs(abnormal_output))),
    }
    if not abnormal_ok:
        raise AssertionError("Non-finite input guard failed")

    if args.cpp_smoke_binary is not None:
        completed = subprocess.run(
            [str(args.cpp_smoke_binary.resolve()), str(compat_path.resolve())],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        cpp_ok = completed.returncode == 0 and "PASS C++ ONNX Runtime" in completed.stdout
        report["tests"]["cpp_onnxruntime_smoke"] = {
            "passed": cpp_ok,
            "return_code": completed.returncode,
            "output": completed.stdout.strip(),
        }
        if not cpp_ok:
            raise AssertionError("C++ ONNX Runtime smoke test failed")

    stress_max = 0.0
    for _ in range(200):
        old_obs = rng.uniform(-100.0, 100.0, (1, 45)).astype(np.float32)
        history = rng.uniform(-100.0, 100.0, (1, 10, 45)).astype(np.float32)
        output = compat.run(["actions"], {"obs": old_obs, "history_obs": history})[0]
        if not np.all(np.isfinite(output)):
            raise AssertionError("Stress test produced non-finite output")
        stress_max = max(stress_max, float(np.max(np.abs(output))))
    report["tests"]["finite_clipped_input_stress"] = {
        "passed": True,
        "samples": 200,
        "maximum_absolute_output_before_legacy_clip": stress_max,
        "note": "The unchanged C++ controller clips wrapper output to [-100,100].",
    }

    stationary_obs = np.zeros((1, 45), dtype=np.float32)
    stationary_obs[:, 3:6] = [0.0, 0.0, -1.0]
    stationary_history = np.zeros((1, 10, 45), dtype=np.float32)
    stationary_wrapper = compat.run(
        ["actions"], {"obs": stationary_obs, "history_obs": stationary_history}
    )[0]
    stationary_filtered = 0.8 * stationary_wrapper
    stationary_target = (OLD_DEFAULT + SCALE_OLD * stationary_filtered)[:, PERM]
    report["metrics"]["stationary_first_step"] = {
        "wrapper_output": stationary_wrapper.reshape(-1).tolist(),
        "physical_joint_target_rad": stationary_target.reshape(-1).tolist(),
        "all_finite": bool(np.all(np.isfinite(stationary_target))),
    }

    timing_obs = stationary_obs.copy()
    timing_history = stationary_history.copy()
    for _ in range(100):
        compat.run(["actions"], {"obs": timing_obs, "history_obs": timing_history})
    timings_ms = []
    for _ in range(2000):
        start = time.perf_counter()
        compat.run(["actions"], {"obs": timing_obs, "history_obs": timing_history})
        timings_ms.append((time.perf_counter() - start) * 1000.0)
    report["metrics"]["cpu_inference_latency_ms"] = {
        "samples": len(timings_ms),
        "mean": float(np.mean(timings_ms)),
        "p50": percentile(timings_ms, 50),
        "p95": percentile(timings_ms, 95),
        "p99": percentile(timings_ms, 99),
        "max": float(np.max(timings_ms)),
        "control_period_ms": 20.0,
    }

    report["artifacts"] = {
        "source_policy_sha256": sha256(args.source_policy),
        "native_onnx_sha256": sha256(native_path),
        "compatibility_onnx_sha256": sha256(compat_path),
        "legacy_actor_sha256": sha256(args.legacy_actor),
    }
    report["passed"] = all(
        value.get("passed", True) for value in report["tests"].values()
    )
    with open(report_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    latency = report["metrics"]["cpu_inference_latency_ms"]
    sequence = report["tests"]["sequential_legacy_controller_equivalence"]
    lines = [
        "# Go2 robot_lab 策略离线适配测试报告",
        "",
        "- 总结：{}".format("通过" if report["passed"] else "失败"),
        "- 原生 TorchScript→ONNX 最大误差：{:.9g}".format(
            report["tests"]["native_torchscript_vs_onnx"]["max_abs_error"]
        ),
        "- 兼容 ONNX→NumPy 参考实现最大误差：{:.9g}".format(
            report["tests"]["compatibility_wrapper_vs_numpy_reference"]["max_abs_error"]
        ),
        "- 1000 帧闭环物理关节目标最大误差：{:.9g} rad".format(
            sequence["max_physical_joint_target_error_rad"]
        ),
        "- 非有限输入防护：{}".format(
            "通过" if report["tests"]["nonfinite_input_guard"]["passed"] else "失败"
        ),
        "- CPU 推理延迟：mean={:.3f} ms, p95={:.3f} ms, max={:.3f} ms（控制周期 20 ms）".format(
            latency["mean"], latency["p95"], latency["max"]
        ),
        "",
        "本报告只验证模型格式、数值一致性、观测变换、关节顺序、默认姿态、动作缩放、旧滤波补偿和离线时序闭环。",
        "它不等价于 Gazebo/机器人动力学测试；在显式替换模型前，现有仿真不会加载这个新文件。",
    ]
    with open(markdown_path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")

    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
