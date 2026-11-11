#!/usr/bin/env python3
# Facts about a recording needed before preprocessing: device generation, camera labels, RGB frame count and
# resolution, and each camera's rolling-shutter readout time (from MPS online calibration, as factory
# calibration has none). Ends with the rectification flags for a 90 degree horizontal FOV at native
# resolution and the matching run_ego2sim.sh command.
#
# python tools/debug/recording_info.py --vrs <recording>.vrs
# python tools/debug/recording_info.py --vrs <recording>.vrs --mps mps_<recording>_vrs/slam --fov 90

import argparse
import math
import sys
from pathlib import Path

from projectaria_tools.core import data_provider, mps

RGB_LABEL = "camera-rgb"


# slam folder MPS writes for a VRS file: <vrs dir>/mps_<stem>_vrs/slam
def derive_mps_folder(vrs_path: Path) -> Path:
    return vrs_path.parent / f"mps_{vrs_path.stem}_vrs" / "slam"


# recording summary, then the suggested rectification flags and pipeline command
def main() -> None:
    ap = argparse.ArgumentParser(description="facts about an Aria recording needed before preprocessing",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vrs", required=True, type=Path, help="path to the .vrs recording")
    ap.add_argument("--mps", type=Path, default=None,
                    help="path to the MPS slam folder (default: derived from --vrs)")
    ap.add_argument("--fov", type=float, default=90,
                    help="target horizontal FOV for the suggested flags (default: 90)")
    args = ap.parse_args()

    vrs_path = args.vrs.expanduser()
    if not vrs_path.exists():
        sys.exit(f"cannot find {vrs_path}")

    mps_folder = (args.mps.expanduser() if args.mps else derive_mps_folder(vrs_path))

    provider = data_provider.create_vrs_data_provider(str(vrs_path))
    calib = provider.get_device_calibration()

    print(f"vrs            {vrs_path}")
    print(f"device         {calib.get_device_version().name}")
    print(f"cameras        {sorted(calib.get_camera_labels())}")
    print()

    # resolution from a decoded frame; the calibration image size can be wrong (see extract_aria_vrs.py)
    stream_id = provider.get_stream_id_from_label(RGB_LABEL)
    n_frames = provider.get_num_data(stream_id)
    image_data, _ = provider.get_sensor_data_by_index(
        stream_id, n_frames // 2).image_data_and_record()
    array = image_data.to_numpy_array()
    height, width = array.shape[0], array.shape[1]

    print(f"rgb frames     {n_frames}")
    print(f"rgb resolution {width} x {height}   (width x height)")
    print()

    # readout time per camera, then the MPS files preprocessing reads
    if mps_folder.exists():
        calib_file = mps_folder / "online_calibration.jsonl"
        if calib_file.exists():
            online = mps.read_online_calibration(str(calib_file))
            for cam in online[0].camera_calibs:
                readout = cam.get_readout_time_sec()
                ms = 0.0 if readout is None else readout * 1e3
                note = "  (global shutter)" if readout is None else ""
                print(f"readout {cam.get_label():18s} {ms:6.2f} ms{note}")
        else:
            print(f"no online_calibration.jsonl in {mps_folder}")

        print()
        required = ["closed_loop_trajectory.csv", "online_calibration.jsonl",
                    "semidense_points.csv.gz", "semidense_observations.csv.gz"]
        for name in required:
            present = (mps_folder / name).exists()
            print(f"  {'found  ' if present else 'MISSING'} {name}")
    else:
        print(f"MPS folder not found at {mps_folder}")
        print("Pass --mps if it lives somewhere else.")

    # focal that gives --fov across the native width
    focal = (width / 2) / math.tan(math.radians(args.fov) / 2)
    print()
    print(f"For {args.fov:.0f} degrees horizontal at native resolution:")
    print(f"    --rectified_rgb_focal {focal:.0f} --rectified_rgb_size {height}")
    print(f"    --rectified_monochrome_focal 180 --rectified_monochrome_height 512")
    print()
    print(f"Rectified folder will be named camera-rgb-rectified-{focal:.0f}-h{height}")
    print()
    print("To run the pipeline with them:")
    print(f"    RGB_FOCAL={focal:.0f} RGB_HEIGHT={height} bash run_ego2sim.sh {vrs_path}")


if __name__ == "__main__":
    main()
