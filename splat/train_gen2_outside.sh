#!/bin/bash
# Stage 3 of run_ego2sim.sh: splat training. Every training setting is in splat/conf/config.yaml.
#
# SCENE=<recording> OUT_ROOT=<recordings>/processed bash splat/train_gen2_outside.sh

set -euo pipefail

SCENE="${SCENE:?set SCENE to the recording name}"
OUT_ROOT="${OUT_ROOT:-${REC_ROOT:+$REC_ROOT/processed}}"
OUT_ROOT="${OUT_ROOT:?set OUT_ROOT (or REC_ROOT) to the preprocessed data root}"
RGB_FOCAL="${RGB_FOCAL:-1008}"
RGB_HEIGHT="${RGB_HEIGHT:-1512}"
RECT="${RECT:-camera-rgb-rectified-${RGB_FOCAL}-h${RGB_HEIGHT}}"   # preprocessing output folder
EXP_NAME="$SCENE/$RECT"                                            # model_path = OUTPUT_ROOT/EXP_NAME

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
OUTPUT_ROOT="${OUTPUT_ROOT:-./output}"

if [ ! -f "$OUT_ROOT/$SCENE/$RECT/transforms_with_sparse_depth.json" ]; then
    echo "No preprocessed data at: $OUT_ROOT/$SCENE/$RECT" >&2
    echo "Run frames/run_gen2_outside.sh first, or check RECT." >&2
    exit 1
fi

export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True   # avoids fragmentation OOMs

python splat/train_lightning.py \
    scene.data_root="$OUT_ROOT" \
    scene.scene_name="$SCENE/$RECT" \
    exp_name="$EXP_NAME" \
    output_root="$OUTPUT_ROOT"

echo
echo "Trained PLY:"
ls "$OUTPUT_ROOT/$EXP_NAME"/point_cloud/iteration_*/point_cloud.ply
