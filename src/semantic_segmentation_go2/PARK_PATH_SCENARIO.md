# Large park lawn with a curved path

This variant preserves the original world and the previous grass/omni world.
It replaces only the small north-west grass island in a newly generated SDF.

- Park extent: approximately `x=-33..-7 m`, `y=14..33 m` (about 26 x 19 m).
- Lawn: semantic label 8, mapped to terrain label 9 by the omni voxel config.
- Path: approximately 2.4 m wide, semantic label 0 (ground/free).
- The centreline runs west-to-east and curves between the north and south
  sides.  It extends beyond both side edges, producing distinct north and
  south lawn areas.
- Lawn and path are visual/semantic overlays without extra collision.  The
  original ground plane remains the traversable physical surface.

Run it with:

```bash
cd /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws
source devel/setup.bash
roslaunch semantic_segmentation_go2 seg_go2_park_path_omni.launch
```

The point-cloud and local-map configuration is unchanged from the omni grass
scenario; the merged output remains `/semantic_pcl/semantic_pcl`.

Regenerate the derived world with:

```bash
rosrun semantic_segmentation_go2 generate_go2_park_path_scenario.py \
  --source-world $(rospack find semantic_segmentation_go2)/sdf/world_large_dynamic_grass_omni.sdf \
  --output-world $(rospack find semantic_segmentation_go2)/sdf/world_large_dynamic_park_path_omni.sdf
```
