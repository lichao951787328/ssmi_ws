#!/usr/bin/env python3
"""Generate the deterministic height and texture maps for the gentle mountain world."""

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter


MAP_PIXELS = 257  # Gazebo heightmaps require 2^n + 1 samples per side.
WORLD_SIZE = 60.0
HEIGHT_SCALE = 3.2
HEIGHT_OFFSET = -0.8
SEED = 20260825


def smooth_spawn_area(height: np.ndarray, xx: np.ndarray, yy: np.ndarray) -> np.ndarray:
    """Keep a flat 3 m spawn pad and smoothly join it to the surrounding land."""
    radius = np.hypot(xx, yy)
    # Ten metres of transition avoids an artificial steep ring around the pad.
    blend = np.clip((radius - 3.0) / 10.0, 0.0, 1.0)
    blend = blend * blend * (3.0 - 2.0 * blend)
    return height * blend


def generate_height() -> np.ndarray:
    axis = np.linspace(-WORLD_SIZE / 2.0, WORLD_SIZE / 2.0, MAP_PIXELS)
    xx, yy = np.meshgrid(axis, axis)
    rng = np.random.default_rng(SEED)

    # Long-wavelength undulation gives the world a mountain-road character
    # without introducing sharp steps under a quadruped's feet.
    height = 0.10 * np.sin((xx + 4.0) * 2.0 * math.pi / 22.0)
    height += 0.07 * np.cos((yy - 2.0) * 2.0 * math.pi / 17.0)
    height += 0.05 * np.sin((xx + yy) * 2.0 * math.pi / 27.0)
    height += 0.12 * xx / (WORLD_SIZE / 2.0)

    # Deterministic broad hills and shallow depressions. Centres are kept away
    # from the robot spawn, and wide sigmas keep the maximum grade moderate.
    for _ in range(13):
        while True:
            cx, cy = rng.uniform(-25.0, 25.0, size=2)
            if math.hypot(cx, cy) > 8.0:
                break
        sx, sy = rng.uniform(5.0, 10.0, size=2)
        amplitude = rng.uniform(-0.45, 0.95)
        gaussian = np.exp(-0.5 * (((xx - cx) / sx) ** 2 + ((yy - cy) / sy) ** 2))
        height += amplitude * gaussian

    # A few elongated rises resemble low shoulders beside a mountain trail.
    ridges = (
        (-18.0, 9.0, 1.05, 10.0, 5.5),
        (17.0, 15.0, 0.85, 8.5, 7.0),
        (18.0, -17.0, 0.70, 11.0, 6.0),
    )
    for cx, cy, amplitude, sx, sy in ridges:
        height += amplitude * np.exp(
            -0.5 * (((xx - cx) / sx) ** 2 + ((yy - cy) / sy) ** 2)
        )

    height = smooth_spawn_area(height, xx, yy)
    return np.clip(height, HEIGHT_OFFSET + 0.03, HEIGHT_OFFSET + HEIGHT_SCALE - 0.03)


def save_heightmap(height: np.ndarray, path: Path) -> None:
    normalized = np.clip((height - HEIGHT_OFFSET) / HEIGHT_SCALE, 0.0, 1.0)
    # ignition-common 4 / Fortress reads 8-bit grayscale heightmaps reliably;
    # its 16-bit PNG path reports every pixel as out of range.
    encoded = np.round(normalized * 255.0).astype(np.uint8)
    Image.fromarray(encoded, mode="L").save(str(path))


def save_surface_texture(path: Path, normal_path: Path) -> None:
    rng = np.random.default_rng(SEED + 1)
    size = 256
    coarse = rng.normal(0.0, 1.0, (size, size))
    coarse_u8 = np.clip(128.0 + 40.0 * coarse, 0.0, 255.0).astype(np.uint8)
    noise = Image.fromarray(coarse_u8, mode="L").filter(ImageFilter.GaussianBlur(3.0))
    noise_array = (np.asarray(noise, dtype=np.float32) - 128.0) / 128.0

    base = np.empty((size, size, 3), dtype=np.float32)
    base[:, :, 0] = 104.0 + 22.0 * noise_array
    base[:, :, 1] = 111.0 + 29.0 * noise_array
    base[:, :, 2] = 69.0 + 15.0 * noise_array
    Image.fromarray(np.clip(base, 0.0, 255.0).astype(np.uint8), mode="RGB").save(str(path))

    normal = np.zeros((8, 8, 3), dtype=np.uint8)
    normal[:, :, :] = (128, 128, 255)
    Image.fromarray(normal, mode="RGB").save(str(normal_path))


def main() -> None:
    parser = argparse.ArgumentParser()
    default_dir = Path(__file__).resolve().parents[1] / "sdf"
    parser.add_argument("--output-dir", type=Path, default=default_dir)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    height = generate_height()
    save_heightmap(height, args.output_dir / "gentle_mountain_heightmap.png")
    save_surface_texture(
        args.output_dir / "gentle_mountain_surface.png",
        args.output_dir / "gentle_mountain_flat_normal.png",
    )

    spacing = WORLD_SIZE / (MAP_PIXELS - 1)
    gy, gx = np.gradient(height, spacing, spacing)
    grade = np.hypot(gx, gy)
    print(
        "generated gentle terrain: "
        f"height={height.min():.3f}..{height.max():.3f} m, "
        f"p95_slope={math.degrees(math.atan(float(np.percentile(grade, 95)))):.1f} deg, "
        f"max_slope={math.degrees(math.atan(float(grade.max()))):.1f} deg"
    )


if __name__ == "__main__":
    main()
