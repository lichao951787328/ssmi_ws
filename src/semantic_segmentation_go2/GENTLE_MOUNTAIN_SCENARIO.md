# Gentle mountain terrain

This is an independent 60 x 60 m Gazebo Fortress world. It does not modify or
derive from `world_large_dynamic_park_path_omni.sdf`.

The collision surface is a deterministic 257 x 257 heightmap with broad random
slopes, shallow depressions, and a flat 3 m spawn pad. Sparse rock groups,
fallen logs, and tree trunks provide additional static obstacles while leaving
several open routes through the terrain.

- Heightmap terrain: semantic label 8
- Boundary: semantic label 4
- Rocks, logs, and trees: semantic label 5
- Robot spawn: `(0, 0, 0.34)`

Run the scenario:

```bash
cd /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_go2 seg_go2_gentle_mountain_omni.launch
```

Regenerate the deterministic height and texture maps after editing the
generator:

```bash
python3 src/semantic_segmentation_go2/scripts/generate_gentle_mountain_heightmap.py
```
