# tools

Scripts that are not part of the pipeline. Run them from the repository root.

```
tools/
├── debug/                        # Checks for your recording and setup
│   ├── recording_info.py         #   frame size of a recording and the settings to use for it
│   ├── solve_focal.py            #   focal length for another field of view
│   ├── verify_preprocessing.py   #   checks the extracted frames before training
│   └── check_env.py              #   checks the Python packages and CUDA
└── eval/                         # The paper's evaluation
    ├── eval_reference.py         #   aligns an iPad LiDAR scan to the glasses' frame
    └── score_cropped.py          #   scores a collider against the aligned scan
```

## Other recording profiles

The defaults fit `profile10`. For another profile, set the frame size before the run:

| Profile | RGB | `RGB_FOCAL` | `RGB_HEIGHT` |
| --- | --- | --- | --- |
| `profile10` | 2016×1512 at 30 Hz | `1008` | `1512` |
| `profile8` | 2560×1920 at 10 Hz | `1280` | `1920` |

```bash
RGB_FOCAL=1280 RGB_HEIGHT=1920 bash run_ego2sim.sh ~/recordings/<recording>.vrs
```

For any other profile, `recording_info.py` prints the values to use:

```bash
python tools/debug/recording_info.py --vrs ~/recordings/<recording>.vrs
```

## Evaluation

Grades a collider against a same-day iPad LiDAR scan, exported from RTAB-Map as a point cloud. The scripts need `scipy` on top of `requirements.txt`.

```bash
python tools/eval/eval_reference.py --ipad room.ply --slam ~/recordings/mps_<recording>_vrs/slam --out output/eval/room
python tools/eval/score_cropped.py --report output/eval/room/report.json --mesh output/mps-mesh/<recording>/<rect>/mesh_delaunay_decimated.ply
```

The first command aligns the scan to the glasses' frame and measures how far the scan can be trusted. The second reports precision, recall and F-score inside the scan's footprint and within 1 m of the ground.

The paper's numbers were made with the scripts at git tag `srf-2026`.
