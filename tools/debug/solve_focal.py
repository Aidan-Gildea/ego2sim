#!/usr/bin/env python3
# Rectified RGB focal length (pixels) for a target horizontal FOV, or the FOV a given focal gives, from
# FOV = 2 * atan((width / 2) / focal). With no arguments, the profile10 size (2016 x 1512) and 90 degrees.
#
# python tools/debug/solve_focal.py
# python tools/debug/solve_focal.py --width 2560 --height 1920
# python tools/debug/solve_focal.py --width 2016 --focal 756

import argparse
import math

COMMON_FOVS = [60, 70, 80, 90, 100, 110, 120]       # degrees, rows of the printed table


# focal in pixels that spans fov_deg across width
def focal_for_fov(width: float, fov_deg: float) -> float:
    return (width / 2) / math.tan(math.radians(fov_deg) / 2)


# horizontal FOV in degrees for a focal in pixels
def fov_for_focal(width: float, focal: float) -> float:
    return math.degrees(2 * math.atan((width / 2) / focal))


# focal table for common FOVs and the flags to pass, or the FOV of --focal
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--width", type=float, default=2016,
                    help="rectified image width in pixels (default: 2016, profile10)")
    ap.add_argument("--height", type=float, default=1512,
                    help="rectified image height in pixels (default: 1512, profile10)")
    ap.add_argument("--fov", type=float, default=90,
                    help="target horizontal field of view in degrees (default: 90)")
    ap.add_argument("--focal", type=float, default=None,
                    help="if given, report the FOV this focal produces instead")
    args = ap.parse_args()

    print(f"width  {args.width:.0f} px")
    print(f"height {args.height:.0f} px")
    print()

    if args.focal is not None:
        fov = fov_for_focal(args.width, args.focal)
        print(f"focal {args.focal:.0f} gives {fov:.1f} degrees horizontal")
        print()
        print("Flags to pass:")
        print(f"    --rectified_rgb_focal {args.focal:.0f} "
              f"--rectified_rgb_size {args.height:.0f}")
        return

    print("  FOV     focal")
    print("  -----   -----")
    for fov in COMMON_FOVS:
        f = focal_for_fov(args.width, fov)
        mark = "  <- target" if abs(fov - args.fov) < 1e-6 else ""
        print(f"  {fov:3d}     {f:5.0f}{mark}")

    focal = focal_for_fov(args.width, args.fov)
    print()
    print(f"For {args.fov:.0f} degrees at width {args.width:.0f}, focal is {focal:.0f}.")
    print()
    print("Flags to pass:")
    print(f"    --rectified_rgb_focal {focal:.0f} "
          f"--rectified_rgb_size {args.height:.0f}")
    print()
    print("--rectified_rgb_size is the output HEIGHT. The width is derived from the")
    print("source aspect ratio, so set the height to your sensor's native height and")
    print("the width above will follow.")


if __name__ == "__main__":
    main()
