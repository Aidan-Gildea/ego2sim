# tools

Scripts outside the pipeline in `run_ego2sim.sh`. Run them from the repository root.

| Folder | What it holds |
| --- | --- |
| `debug/` | Checks for when your recording or setup differs from the usual one |
| `eval/` | The paper's evaluation: an iPad LiDAR scan aligned to the MPS frame, and the collider scored against it |

## debug

| Script | What it does |
| --- | --- |
| `recording_info.py` | Prints a recording's RGB size, which MPS files are present, and the `RGB_FOCAL` and `RGB_HEIGHT` to use |
| `solve_focal.py` | Gives the `RGB_FOCAL` for a field of view other than 90° |
| `verify_preprocessing.py` | Checks a folder of extracted frames before training on it |
| `check_env.py` | Checks that the active Python has the packages and a CUDA build of PyTorch |

### Recording profiles other than profile10

The pipeline flattens each fisheye frame into an image of a set size, chosen by `RGB_FOCAL` and `RGB_HEIGHT`. The defaults match profile10. Other profiles still run with them, but their frames get resized: smaller loses detail, and larger takes longer with no gain.

| Profile | RGB | `RGB_FOCAL` | `RGB_HEIGHT` |
| --- | --- | --- | --- |
| `profile10` | 2016×1512 at 30 Hz | `1008` | `1512` |
| `profile8` | 2560×1920 at 10 Hz | `1280` | `1920` |

For any other profile, `recording_info.py` reads the frame size from your recording and prints the command to run with the right values:

```bash
python tools/debug/recording_info.py --vrs ~/recordings/my_recording.vrs
```

```bash
RGB_FOCAL=1280 RGB_HEIGHT=1920 bash run_ego2sim.sh ~/recordings/my_recording.vrs
```

Set the same values every time you rerun that recording, or edit them at the top of `run_ego2sim.sh`. With the defaults, the pipeline starts over at the profile10 size. The outputs go in `camera-rgb-rectified-<RGB_FOCAL>-h<RGB_HEIGHT>/` folders, not the `camera-rgb-rectified-1008-h1512/` ones in the main README.

These values keep a 90° field of view at the camera's own resolution. For a different field of view, `solve_focal.py` gives the matching `RGB_FOCAL`.

## eval

These scripts grade an Ego2Sim collider against an independent reference: a same-day iPad LiDAR scan of the place, exported from RTAB-Map as a point cloud.

The scan is called *reference geometry*, not ground truth. Its own error is measured, and only thresholds of at least twice that error are trusted.

| Script | What it does | Writes |
|---|---|---|
| `eval_reference.py` | Aligns the scan to the MPS world frame: levels the scan's floor, runs a 1° yaw sweep with top-down map correlation, then point-to-plane ICP of the confident MPS points onto the scan (never the mesh being graded). Then checks the alignment: the score at the other three quarter turns, a free-scale fit, floor tilt against MPS "up", and the error budget σ = √(σ_scan² + σ_align²). | `report.json` (includes `T_scan_raw_to_mps`), `alignment_overlay.png`, `scan_in_mps_frame.ply` |
| `score_cropped.py` | Scores the decimated collider inside the scan's footprint and in the contact band (up to 1 m above the local ground): precision, recall and F at 2/5/10/20 cm, plus how much of the scanned ground has collider within 5 and 10 cm. `--sim3` first refits the scan with free scale against the MPS points. | `cropped_<mesh>.json`, `cropped_<mesh>.png` (error map) |

Use the same environment as the pipeline. The scripts also need `scipy`, which isn't in `requirements.txt`.

```bash
python tools/eval/eval_reference.py --ipad room.ply --slam ~/recordings/mps_Room_vrs/slam \
  --eye 1.673 --out output/eval/room
python tools/eval/score_cropped.py --report output/eval/room/report.json \
  --mesh output/mps-mesh/Room/camera-rgb-rectified-1008-h1512/mesh_delaunay_decimated.ply
```

`--eye` is the wearer's eye height in metres. It's only a fallback for finding the MPS floor. `score_cropped.py` takes the scan and MPS folder from `report.json` and writes next to it. Reports from before November 2026 don't record those paths, so pass `--scan` and `--slam` for them.

- Precision, recall and F-score follow Tanks and Temples (Knapitsch et al., 2017). Scores are taken only inside the scan's footprint, because mesh outside it would otherwise count as error.
- The alignment uses the same MPS points the mesh is built from, so the reference is not fully independent of the mesh.
- The paper's numbers were made with the scripts at git tag `srf-2026` (then in `paper_evaluations/`).
