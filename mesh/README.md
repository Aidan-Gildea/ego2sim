# mesh

Stage 2 of the pipeline: builds the collision mesh (the collider) that robots stand on and bump into.

```
mesh/
├── mps-mesh/         # Default: mesh from the glasses' SLAM points, a few minutes
└── photogrammetry/   # Baseline: mesh from COLMAP multi-view stereo, hours.
                      # Also holds the mesh scoring and simplification both use
```

`run_ego2sim.sh` uses `mps-mesh`. Set `MESH_SOURCE=mvs` to use `photogrammetry` instead. Each folder has its own README with the details.

Either one can be run on its own:

```bash
bash mesh/mps-mesh/run_mps_mesh.sh <rectified_dir>
bash mesh/photogrammetry/run_photogrammetry.sh <rectified_dir>
```

The collider is written to `output/mps-mesh/<recording>/<rect>/mesh_delaunay_decimated.ply`.
