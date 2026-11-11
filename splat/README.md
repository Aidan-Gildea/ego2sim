# splat

Stage 3 of the pipeline: trains the Gaussian splat on the frames from stage 1. The training code comes from Meta's [egocentric_splats](https://github.com/facebookresearch/egocentric_splats).

```
splat/
├── train_gen2_outside.sh   # What run_ego2sim.sh runs for this stage
├── train_lightning.py      # Trains a splat
├── render_lightning.py     # Renders a video from a trained splat
├── run_aria_render.sh      # Wrapper for render_lightning.py
├── launch_viewer.py        # Shows a trained splat in the browser
├── conf/                   # Training parameters (Hydra)
├── model/                  # The 3DGS and 2DGS models and their losses
├── scene/                  # Cameras and dataset loading
├── utils/                  # Image, point cloud and render helpers
└── viewer/                 # Browser viewer
```

## Training parameters

Every training parameter is in [conf/config.yaml](conf/config.yaml), with a short note next to each. Edit the file and run the pipeline. The scripts only pass the input and output paths, so what's in the file is what trains.

The ones you're most likely to change:

| Parameter | Default | What it does |
| --- | --- | --- |
| `opt.iterations` | `30000` | Training steps. Time grows about linearly with it |
| `opt.mcmc_strategy.cap_max` | `1500000` | Most Gaussians allowed. Higher gives more detail and uses more GPU memory. 1.5M fits 16 GB outdoors |
| `opt.handle_rolling_shutter` | `true` | Models the camera reading out row by row. Sharper, but slower |
| `opt.handle_rolling_shutter_start_iter` | `10000` | Step where that starts |
| `opt.depth_loss` | `false` | Also fits the MPS sparse depth |
| `scene.train_split` | `7-1` | `7-1` holds out every 8th frame to score the splat. `all` trains on every frame |

To try a value once without editing the file, add it to the training command:

```bash
python splat/train_lightning.py opt.iterations=15000 \
    scene.data_root=<recordings>/processed scene.scene_name=<recording>/<rect> \
    exp_name=<recording>/<rect> output_root=output
```

`run_ego2sim.sh` skips training when a splat already exists. Delete `output/<recording>/<rect>/point_cloud/` to train again.

## View a trained splat

```bash
python splat/launch_viewer.py model_root=output/<recording>
```

It opens in a browser on port 8080.
