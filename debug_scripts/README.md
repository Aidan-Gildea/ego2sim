# Debug scripts

Checks for when your recording or setup differs from the usual one. Run them from the repository root.

## Recording profiles other than profile10

The pipeline flattens each fisheye frame into an image of a set size, chosen by `RGB_FOCAL` and `RGB_HEIGHT`. The defaults match profile10. Other profiles still run with them, but their frames get resized: smaller loses detail, and larger takes longer with no gain.

| Profile | RGB | `RGB_FOCAL` | `RGB_HEIGHT` |
| --- | --- | --- | --- |
| `profile10` | 2016×1512 at 30 Hz | `1008` | `1512` |
| `profile8` | 2560×1920 at 10 Hz | `1280` | `1920` |

For any other profile, `recording_info.py` reads the frame size from your recording and prints the command to run with the right values:

```bash
python debug_scripts/recording_info.py --vrs ~/recordings/my_recording.vrs
```

```bash
RGB_FOCAL=1280 RGB_HEIGHT=1920 bash run_ego2sim.sh ~/recordings/my_recording.vrs
```

Set the same values every time you rerun that recording, or edit them at the top of `run_ego2sim.sh`. With the defaults, the pipeline starts over at the profile10 size. The outputs go in `camera-rgb-rectified-<RGB_FOCAL>-h<RGB_HEIGHT>/` folders, not the `camera-rgb-rectified-1008-h1512/` ones in the main README.

These values keep a 90° field of view at the camera's own resolution. To trade field of view for detail, see [docs/rectification.md](../docs/rectification.md).

## Scripts

| Script | What it does |
| --- | --- |
| `recording_info.py` | Prints a recording's RGB size, which MPS files are present, and the `RGB_FOCAL` and `RGB_HEIGHT` to use |
| `solve_focal.py` | Gives the `RGB_FOCAL` for a field of view other than 90° |
| `verify_preprocessing.py` | Checks a folder of extracted frames before training on it |
| `check_env.py` | Checks that the active Python has the packages and a CUDA build of PyTorch |
