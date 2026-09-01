#!/usr/bin/env python3
"""Build native and legacy-controller-compatible ONNX models for Go2.

This script only writes generated artifacts to the requested output directory.
It does not replace the existing controller model or edit any ROS source file.
"""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch


PERM = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
OLD_DEFAULT = [
    0.1, 0.8, -1.5,
    -0.1, 0.8, -1.5,
    0.1, 1.0, -1.5,
    -0.1, 1.0, -1.5,
]
NEW_DEFAULT = [
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
]
ACTION_SCALE_OLD_ORDER = [
    0.125, 0.25, 0.25,
    0.125, 0.25, 0.25,
    0.125, 0.25, 0.25,
    0.125, 0.25, 0.25,
]


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def describe_session(path):
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    return {
        "inputs": [
            {"name": item.name, "shape": item.shape, "type": item.type}
            for item in session.get_inputs()
        ],
        "outputs": [
            {"name": item.name, "shape": item.shape, "type": item.type}
            for item in session.get_outputs()
        ],
    }


def make_eager_actor(torchscript_policy):
    """Recreate the inspected 45-512-256-128-12 ELU actor exactly."""
    actor = torch.nn.Sequential(
        torch.nn.Linear(45, 512),
        torch.nn.ELU(),
        torch.nn.Linear(512, 256),
        torch.nn.ELU(),
        torch.nn.Linear(256, 128),
        torch.nn.ELU(),
        torch.nn.Linear(128, 12),
    )
    actor.load_state_dict(torchscript_policy.actor.state_dict(), strict=True)
    actor.eval()
    return actor


class LegacyControllerCompatibility(torch.nn.Module):
    """Adapt the old FL/FR/RL/RR two-input ABI to robot_lab FR/FL/RR/RL."""

    def __init__(self, actor):
        super().__init__()
        self.actor = actor
        self.register_buffer("perm", torch.tensor(PERM, dtype=torch.long))
        self.register_buffer(
            "old_default", torch.tensor(OLD_DEFAULT, dtype=torch.float32)
        )
        self.register_buffer(
            "new_default", torch.tensor(NEW_DEFAULT, dtype=torch.float32)
        )
        self.register_buffer(
            "action_scale_old",
            torch.tensor(ACTION_SCALE_OLD_ORDER, dtype=torch.float32),
        )

        # Target offset in raw-action units, represented in the old joint order.
        new_default_old = self.new_default.index_select(0, self.perm)
        posture_offset = (
            new_default_old - self.old_default
        ) / self.action_scale_old
        self.register_buffer("posture_offset_old", posture_offset)

    @staticmethod
    def finite_and_clipped(value):
        finite = torch.where(torch.isfinite(value), value, torch.zeros_like(value))
        return torch.clamp(finite, -100.0, 100.0)

    def forward(self, obs, history_obs):
        obs = self.finite_and_clipped(obs)
        history_obs = self.finite_and_clipped(history_obs)

        angular_velocity_and_gravity = obs[:, 0:6]
        commands = obs[:, 6:9] * obs.new_tensor([0.5, 0.5, 4.0])

        old_q_delta = obs[:, 9:21]
        old_q = old_q_delta + self.old_default
        new_q_delta = old_q.index_select(1, self.perm) - self.new_default

        old_dq_scaled = obs[:, 21:33]
        new_dq_scaled = old_dq_scaled.index_select(1, self.perm)

        # At time t, obs contains wrapper output z[t-1], while the newest
        # history item contains z[t-2]. This exactly reconstructs the action
        # that passed through the old controller's 0.2/0.8 filter at t-1.
        wrapper_action_previous = obs[:, 33:45]
        wrapper_action_previous_previous = history_obs[:, -1, 33:45]
        filtered_previous_old = (
            0.2 * wrapper_action_previous_previous
            + 0.8 * wrapper_action_previous
        )
        policy_action_previous = (
            filtered_previous_old - self.posture_offset_old
        ).index_select(1, self.perm)

        startup = (
            torch.sum(torch.abs(wrapper_action_previous), dim=1, keepdim=True)
            + torch.sum(
                torch.abs(wrapper_action_previous_previous), dim=1, keepdim=True
            )
        ) < 1.0e-12
        policy_action_previous = torch.where(
            startup, torch.zeros_like(policy_action_previous), policy_action_previous
        )

        policy_obs = torch.cat(
            [
                angular_velocity_and_gravity,
                commands,
                new_q_delta,
                new_dq_scaled,
                policy_action_previous,
            ],
            dim=1,
        )
        policy_action = self.actor(policy_obs)
        policy_action = torch.where(
            torch.isfinite(policy_action), policy_action, torch.zeros_like(policy_action)
        )

        desired_filtered_action_old = (
            policy_action.index_select(1, self.perm) + self.posture_offset_old
        )

        # Pre-compensate the filter in the unchanged legacy C++ controller:
        # 0.2*z[t-1] + 0.8*z[t] == desired_filtered_action_old.
        wrapper_action = (
            desired_filtered_action_old - 0.2 * wrapper_action_previous
        ) / 0.8
        return wrapper_action


def add_metadata(model_path, entries):
    model = onnx.load(str(model_path))
    del model.metadata_props[:]
    for key, value in entries.items():
        item = model.metadata_props.add()
        item.key = str(key)
        item.value = str(value)
    onnx.checker.check_model(model)
    onnx.save(model, str(model_path))


def ensure_new(paths, force):
    existing = [str(path) for path in paths if path.exists()]
    if existing and not force:
        raise FileExistsError(
            "Refusing to overwrite generated artifacts; pass --force: "
            + ", ".join(existing)
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-policy", required=True, type=Path)
    parser.add_argument("--source-config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--source-repo", default="https://github.com/fan-ziqi/rl_sar.git")
    parser.add_argument("--source-commit", default="unknown")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    if not args.source_policy.is_file():
        raise FileNotFoundError(args.source_policy)
    if not args.source_config.is_file():
        raise FileNotFoundError(args.source_config)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    source_policy_copy = args.output_dir / "source_policy.pt"
    source_config_copy = args.output_dir / "source_config.yaml"
    native_path = args.output_dir / "policy_robot_lab_native.onnx"
    compat_path = args.output_dir / "actor_robot_lab_legacy_compat.onnx"
    manifest_path = args.output_dir / "manifest.json"
    ensure_new(
        [source_policy_copy, source_config_copy, native_path, compat_path, manifest_path],
        args.force,
    )

    shutil.copy2(str(args.source_policy), str(source_policy_copy))
    shutil.copy2(str(args.source_config), str(source_config_copy))

    policy = torch.jit.load(str(args.source_policy), map_location="cpu")
    policy.eval()
    eager_actor = make_eager_actor(policy)
    example_obs = torch.zeros(1, 45, dtype=torch.float32)

    with torch.no_grad():
        example_output = policy(example_obs)
        eager_output = eager_actor(example_obs)
    if tuple(example_output.shape) != (1, 12):
        raise ValueError("Expected policy shape [1,12], got {}".format(example_output.shape))
    if not torch.equal(example_output, eager_output):
        raise ValueError("Eager actor is not exactly equal to the TorchScript policy")

    torch.onnx.export(
        eager_actor,
        example_obs,
        str(native_path),
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=["input"],
        output_names=["output"],
    )

    compatibility_model = LegacyControllerCompatibility(eager_actor)
    compatibility_model.eval()
    example_history = torch.zeros(1, 10, 45, dtype=torch.float32)
    torch.onnx.export(
        compatibility_model,
        (example_obs, example_history),
        str(compat_path),
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=["obs", "history_obs"],
        output_names=["actions"],
    )

    source_hash = sha256(args.source_policy)
    common_metadata = {
        "source_repo": args.source_repo,
        "source_commit": args.source_commit,
        "source_policy_sha256": source_hash,
        "source_policy_framework": "TorchScript",
        "policy_observation_count": 45,
        "policy_action_count": 12,
        "policy_joint_order": "FR,FL,RR,RL; hip,thigh,calf",
    }
    add_metadata(
        native_path,
        dict(common_metadata, model_role="native robot_lab policy export"),
    )
    add_metadata(
        compat_path,
        dict(
            common_metadata,
            model_role="legacy go2 controller compatibility wrapper",
            legacy_input_joint_order="FL,FR,RL,RR; hip,thigh,calf",
            legacy_filter_compensation="z=(desired-0.2*z_previous)/0.8",
            history_usage="latest history action reconstructs z_previous_previous",
            output_joint_order="FL,FR,RL,RR for unchanged controller remapping",
        ),
    )

    manifest = {
        "source": {
            "repository": args.source_repo,
            "commit": args.source_commit,
            "policy_original_path": str(args.source_policy.resolve()),
            "config_original_path": str(args.source_config.resolve()),
            "policy_sha256": source_hash,
            "config_sha256": sha256(args.source_config),
        },
        "generated": {
            native_path.name: {
                "sha256": sha256(native_path),
                "interface": describe_session(native_path),
            },
            compat_path.name: {
                "sha256": sha256(compat_path),
                "interface": describe_session(compat_path),
            },
        },
        "mapping": {
            "legacy_joint_order": ["FL", "FR", "RL", "RR"],
            "policy_joint_order": ["FR", "FL", "RR", "RL"],
            "permutation": PERM,
            "legacy_default_position": OLD_DEFAULT,
            "policy_default_position": NEW_DEFAULT,
            "legacy_effective_action_scale": ACTION_SCALE_OLD_ORDER,
            "legacy_command_scale_to_policy": [0.5, 0.5, 4.0],
            "legacy_filter": [0.2, 0.8],
        },
        "tool_versions": {
            "torch": torch.__version__,
            "onnx": onnx.__version__,
            "onnxruntime": ort.__version__,
            "numpy": np.__version__,
        },
    }
    with open(manifest_path, "w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    print("Generated:")
    for path in [source_policy_copy, source_config_copy, native_path, compat_path, manifest_path]:
        print("  {}  sha256={}".format(path, sha256(path)))


if __name__ == "__main__":
    main()
