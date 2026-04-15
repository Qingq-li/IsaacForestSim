# Forest Digital Twin Pipeline

This directory contains the point-cloud-to-digital-twin generator used for the
forest playground demo.

## Inputs

- `script/data/example_forest_pointcloud/porvoo-20250520-000013_ground_sec.ply`
- `script/data/example_forest_pointcloud/porvoo-20250520-000013_trunk_sec.ply`
- `script/data/example_forest_pointcloud/porvoo-20250520-000013_canopy_sec.ply`

## Outputs

By default the generator writes these artifacts into `data/digital_twin/`:

- `forest_demo.usda`
- `forest_demo_manifest.json`
- `forest_demo_cache.npz`

Those files are intended to be mounted or copied into Isaac Sim as a single
digital-twin bundle.

## Recommended flow

From the repository root:

```bash
python3 script/digitwin/forest_env_demo.py --output-dir data/digital_twin --add-canopy
```

If you are running inside the Isaac Sim container, use the container Python
launcher instead:

```bash
/isaac-sim/python.sh /tmp/forest_env_demo.py --source-dir /tmp/forest_pointcloud --output-dir /tmp/forest_digital_twin --add-canopy
```

The `Makefile` target `run-forest-usda-from-pointcloud` automates the container
copy-in and copy-out workflow.

When available, the generator prefers the more realistic pine canopy asset from
`/home/prefor/isaac-sim/plan/canopy_assets/pine_tree.usd`. The Makefile copies
that asset into the Isaac Sim container automatically before generation.

## Notes

- `--output-dir` is the primary interface. `--output`, `--cache`, and
  `--manifest` are optional overrides.
- The script accepts PLY or LAS inputs for compatibility, but the checked-in
  example dataset uses PLY.
- The reconstruction step is deterministic for a fixed input dataset and
  command-line configuration.
