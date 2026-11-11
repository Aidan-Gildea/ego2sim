#!/bin/bash
# Rectified frames, MPS poses and sparse depth from an Aria Gen 2 profile10 recording (RGB 2016x1512,
# 30 Hz), made with frames/extract_aria_vrs.py. Stage 1 of run_ego2sim.sh; paths come from the environment.
#
# REC_ROOT=/path/to/recordings SCENE=<recording> bash frames/run_gen2_outside.sh

set -euo pipefail

REC_ROOT="${REC_ROOT:?set REC_ROOT to the folder containing \$SCENE.vrs}"
SCENE="${SCENE:?set SCENE to the .vrs filename without the extension}"
MPS_FOLDER="${MPS_FOLDER:-$REC_ROOT/mps_${SCENE}_vrs/slam}"   # aria_mps SLAM output
OUT_ROOT="${OUT_ROOT:-$REC_ROOT/processed}"                   # extracted frames

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"                                               # paths below are repo-relative

if [ ! -f "$REC_ROOT/$SCENE.vrs" ]; then
    echo "No recording at: $REC_ROOT/$SCENE.vrs" >&2
    exit 1
fi
if [ ! -f "$MPS_FOLDER/closed_loop_trajectory.csv" ]; then
    echo "No MPS output at: $MPS_FOLDER (expected closed_loop_trajectory.csv)" >&2
    exit 1
fi

mkdir -p "$OUT_ROOT"

# rectification for profile10; the profile8 values (focal 1280, height 1920) would upsample every frame
RGB_FOCAL="${RGB_FOCAL:-1008}"       # 90 deg horizontal FOV at the native 2016 px width
RGB_HEIGHT="${RGB_HEIGHT:-1512}"     # output height; the width follows the 4:3 source
SLAM_FOCAL="${SLAM_FOCAL:-180}"      # about 110 of the 119 deg SLAM FOV at 512 px
SLAM_HEIGHT="${SLAM_HEIGHT:-512}"    # SLAM cameras are 512x512

python frames/extract_aria_vrs.py \
    --input_root  "$REC_ROOT" \
    --output_root "$OUT_ROOT" \
    --vrs_file    "$SCENE.vrs" \
    --rectified_rgb_focal        "$RGB_FOCAL" \
    --rectified_rgb_size         "$RGB_HEIGHT" \
    --rectified_monochrome_focal "$SLAM_FOCAL" \
    --rectified_monochrome_height "$SLAM_HEIGHT" \
    --online_calib_file           "$MPS_FOLDER/online_calibration.jsonl" \
    --trajectory_file             "$MPS_FOLDER/closed_loop_trajectory.csv" \
    --semi_dense_points_file      "$MPS_FOLDER/semidense_points.csv.gz" \
    --semi_dense_observation_file "$MPS_FOLDER/semidense_observations.csv.gz"
    # --overwrite                          # regenerate instead of skipping existing output
    # --visualize                          # stream each stage to a rerun viewer
    # --timestamp_convention readout_start # upstream timestamps, half a readout off

echo
echo "Done. Training folder:"
echo "  $OUT_ROOT/$SCENE/camera-rgb-rectified-${RGB_FOCAL}-h${RGB_HEIGHT}"
