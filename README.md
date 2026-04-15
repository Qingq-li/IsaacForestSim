# IsaacForestSim

Turn segmented forest point clouds into a clean, navigable digital twin for Isaac Sim, with terrain, trunks, and more realistic pine canopy ready for playground experiments.

![SAHA Harvester](doc/harvester/saha-harvester.png)

SAHA harvester concept for future forest playground tasks, where the digital twin can be used as the scene layer for navigation, interaction, and machine simulation.

<table>
  <tr>
    <td align="center">
      <img src="doc/digi__twin/pointcloud_example.png" alt="Point Cloud Example" width="100%">
    </td>
    <td align="center">
      <img src="doc/digi__twin/isaacsim_example.png" alt="Isaac Sim Digital Twin Example" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center">
      Raw segmented forest point cloud with ground, trunks, and canopy prepared as reconstruction input.
    </td>
    <td align="center">
      Generated forest digital twin loaded in Isaac Sim with terrain surface, tree trunks, and realistic pine canopy.
    </td>
  </tr>
</table>

## Generate Digital Twin

The main generator is [forest_env_demo.py](/home/prefor/IsaacForestSim/script/digitwin/forest_env_demo.py).

Input point clouds:
- `script/data/example_forest_pointcloud/porvoo-20250520-000013_ground_sec.ply`
- `script/data/example_forest_pointcloud/porvoo-20250520-000013_trunk_sec.ply`
- `script/data/example_forest_pointcloud/porvoo-20250520-000013_canopy_sec.ply`

Output bundle:
- `data/digital_twin/forest_demo.usda`
- `data/digital_twin/forest_demo_manifest.json`
- `data/digital_twin/forest_demo_cache.npz`

Run inside Isaac Sim container:

```bash
make run-forest-usda-from-pointcloud
```

Or run the script directly with the default repo paths:

```bash
python3 script/digitwin/forest_env_demo.py --output-dir data/digital_twin --add-canopy
```

The generator will use the more realistic pine canopy asset from
`/home/prefor/isaac-sim/plan/canopy_assets/pine_tree.usd` when it is available.
