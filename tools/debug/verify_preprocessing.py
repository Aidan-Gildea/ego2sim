#!/usr/bin/env python3
# Checks on a rectified camera folder before training: helper images the same size as the frames,
# transforms_with_sparse_depth.json present with frames, a non-zero readout time for an RGB camera, and
# sparse depth files that hold points. Exits 1 if any check fails.
#
# python tools/debug/verify_preprocessing.py /path/to/camera-rgb-rectified-1008-h1512

import argparse
import glob
import json
import sys
from pathlib import Path

from PIL import Image

# must match the image size, or training fails with a broadcast error in Camera.vignette_image.expand_as
HELPERS = ["vignette.png", "mask.png", "image_index.png"]


# all checks on the folder, with a summary of what failed
def main() -> None:
    ap = argparse.ArgumentParser(description="checks on a rectified camera folder before training",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", nargs="?", default=".", type=Path,
                    help="rectified camera folder (default: current directory)")
    ap.add_argument("--samples", type=int, default=5,
                    help="how many frames to sample for depth counts (default: 5)")
    args = ap.parse_args()

    folder = args.folder.expanduser().resolve()
    print(f"folder    {folder}")
    failures = []

    images = sorted(glob.glob(str(folder / "images" / "*.png")))
    if not images:
        sys.exit(f"no images found in {folder}/images")
    size = Image.open(images[0]).size
    print(f"images    {len(images)}  {size[0]} x {size[1]}")
    print()

    # helper image sizes; image_index.png only exists for the rolling-shutter RGB camera
    is_rgb = folder.name.startswith("camera-rgb")
    for name in HELPERS:
        path = folder / name
        if not path.exists():
            if name == "image_index.png" and not is_rgb:
                print(f"  skipped {name:18s} (global shutter camera, not written)")
            else:
                print(f"  MISSING {name}")
                failures.append(name)
            continue
        helper_size = Image.open(path).size
        if helper_size == size:
            print(f"  ok      {name:18s} {helper_size[0]} x {helper_size[1]}")
        else:
            print(f"  FAIL    {name:18s} {helper_size[0]} x {helper_size[1]} "
                  f"does not match the images")
            failures.append(name)

    # frames, readout time and sparse depth from stages 3 and 4
    print()
    transforms = folder / "transforms_with_sparse_depth.json"
    if not transforms.exists():
        print(f"  MISSING transforms_with_sparse_depth.json")
        print("          Stages 3 and 4 did not complete. Training reads this file.")
        failures.append(transforms.name)
    else:
        data = json.load(open(transforms))
        frames = data["frames"]
        print(f"  frames            {len(frames)}")

        first = frames[0]
        readout_ms = (first["timestamp_read_end"] - first["timestamp_read_start"]) / 1e6
        print(f"  readout           {readout_ms:.2f} ms")
        if is_rgb and readout_ms == 0:
            print("          RGB readout of zero means online calibration carried no")
            print("          readout time. Rolling shutter cannot be modelled.")
            failures.append("readout")

        # depth counts sampled across the recording; frame zero sits at the edge of the trajectory window
        step = max(1, len(frames) // args.samples)
        counts = []
        for frame in frames[::step][:args.samples]:
            rel = frame.get("sparse_depth")
            if rel is None:
                counts.append(0)
                continue
            depth = json.load(open(folder / rel))
            counts.append(len(depth["z"]))
        print(f"  depth pts sampled {counts}")
        if not any(counts):
            print("          Every sampled frame has zero depth points. Check the")
            print("          observation match rate printed during preprocessing.")
            failures.append("sparse_depth")

    print()
    if failures:
        print(f"FAILED: {', '.join(failures)}")
        sys.exit(1)
    print("All checks passed. Ready to train on:")
    print(f"    scene.scene_name=\"{folder.parent.name}/{folder.name}\"")


if __name__ == "__main__":
    main()
