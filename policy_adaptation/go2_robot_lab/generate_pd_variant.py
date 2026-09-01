#!/usr/bin/env python3
"""Generate an isolated semantic-Go2 SDF/world physics variant."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-model-dir", required=True, type=Path)
    parser.add_argument("--source-world", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--kp", default=40.0, type=float)
    parser.add_argument("--kd", default=1.0, type=float)
    parser.add_argument("--ground-mu", default=100.0, type=float)
    parser.add_argument("--ground-mu2", default=50.0, type=float)
    args = parser.parse_args()

    variant_model_dir = args.output_dir / "go2_semantic_pd_variant"
    variant_world = args.output_dir / "world_large_dynamic_pd_variant.sdf"
    manifest_path = args.output_dir / "pd_variant_manifest.json"
    if variant_model_dir.exists() or variant_world.exists() or manifest_path.exists():
        raise FileExistsError("PD variant already exists; use a fresh output directory")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copytree(str(args.source_model_dir), str(variant_model_dir))
    model_sdf = variant_model_dir / "model.sdf"
    model_text = model_sdf.read_text(encoding="utf-8")
    kp_count = model_text.count("<kp>40.0</kp>")
    kd_count = model_text.count("<kd>1.0</kd>")
    if kp_count != 12 or kd_count != 12:
        raise ValueError("Expected exactly 12 original kp/kd entries, got {}/{}".format(kp_count, kd_count))
    model_text = model_text.replace("<kp>40.0</kp>", "<kp>{}</kp>".format(args.kp))
    model_text = model_text.replace("<kd>1.0</kd>", "<kd>{}</kd>".format(args.kd))
    model_sdf.write_text(model_text, encoding="utf-8")

    world_text = args.source_world.read_text(encoding="utf-8")
    original_ground_friction = "<friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>"
    if world_text.count(original_ground_friction) != 1:
        raise ValueError("Expected exactly one original ground friction entry")
    variant_ground_friction = (
        "<friction><ode><mu>{}</mu><mu2>{}</mu2></ode></friction>".format(
            args.ground_mu, args.ground_mu2
        )
    )
    world_text = world_text.replace(original_ground_friction, variant_ground_friction)
    original_uri = "<uri>model://go2_semantic</uri>"
    if world_text.count(original_uri) != 1:
        raise ValueError("Expected exactly one go2_semantic include")
    variant_uri = "<uri>file://{}</uri>".format(variant_model_dir.resolve())
    world_text = world_text.replace(original_uri, variant_uri)
    variant_world.write_text(world_text, encoding="utf-8")

    manifest = {
        "source_model_sdf": str((args.source_model_dir / "model.sdf").resolve()),
        "source_model_sha256": sha256(args.source_model_dir / "model.sdf"),
        "source_world": str(args.source_world.resolve()),
        "source_world_sha256": sha256(args.source_world),
        "generated_model_sdf": str(model_sdf.resolve()),
        "generated_model_sha256": sha256(model_sdf),
        "generated_world": str(variant_world.resolve()),
        "generated_world_sha256": sha256(variant_world),
        "kp": args.kp,
        "kd": args.kd,
        "ground_mu": args.ground_mu,
        "ground_mu2": args.ground_mu2,
        "changed_joint_count": 12,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
