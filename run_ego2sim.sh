#!/bin/bash
# Aria Gen 2 recording -> Isaac Sim scene
#
# bash run_ego2sim.sh /path/to/<recording>.vrs

set -euo pipefail

# Settings: edit here, or set before the command. Training settings are in splat/conf/config.yaml
export MESH_SOURCE="${MESH_SOURCE:-mps}"            # mps (~1 min) or mvs (hours)
export DECIMATE="${DECIMATE:-1}"                    # 1 simplified collider, 0 full mesh
export EYE_HEIGHT="${EYE_HEIGHT:-1.6683}"           # glasses height (m), scoring only

SPLAT_RADIUS="${SPLAT_RADIUS:-50}"                  # drop Gaussians beyond this (m)
MAX_EDGE="${MAX_EDGE:-2.0}"                         # drop longer triangle edges (m)

export RGB_FOCAL="${RGB_FOCAL:-1008}"               # profile10 1008, profile8 1280
export RGB_HEIGHT="${RGB_HEIGHT:-1512}"             # profile10 1512, profile8 1920

# Paths
VRS="${1:?usage: bash run_ego2sim.sh /path/to/<recording>.vrs}"
[ -f "$VRS" ] || { echo "No recording at: $VRS" >&2; exit 1; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REC_ROOT="$(cd "$(dirname "$VRS")" && pwd)"
export SCENE="$(basename "$VRS" .vrs)"
export RECT="camera-rgb-rectified-${RGB_FOCAL}-h${RGB_HEIGHT}"
MPS_FOLDER="${MPS_FOLDER:-$REC_ROOT/mps_${SCENE}_vrs/slam}"   # aria_mps SLAM output
OUT_ROOT="${OUT_ROOT:-$REC_ROOT/processed}"                   # extracted frames
OUTPUT_ROOT="${OUTPUT_ROOT:-$REPO_ROOT/output}"               # mesh, splat, scene

case "$MESH_SOURCE" in
    mps) MESH_SCRIPT="mesh/mps-mesh/run_mps_mesh.sh";             MESH_SUBDIR="mps-mesh" ;;
    mvs) MESH_SCRIPT="mesh/photogrammetry/run_photogrammetry.sh"; MESH_SUBDIR="photogrammetry" ;;
    *)   echo "MESH_SOURCE must be mps or mvs, not '$MESH_SOURCE'" >&2; exit 1 ;;
esac

# Preflight: missing inputs or tools caught before any long stage
[ -f "$MPS_FOLDER/closed_loop_trajectory.csv" ] || {
    echo "No MPS SLAM output at: $MPS_FOLDER" >&2
    echo "Run aria_mps on the recording, or set MPS_FOLDER to its slam/ folder." >&2
    exit 1
}
command -v colmap >/dev/null || { echo "colmap is not on PATH" >&2; exit 1; }
python -c "import torch, lightning, hydra, gsplat, open3d, pxr, msgpack, plyfile" 2>/dev/null || {
    echo "The active Python is missing packages. Run: pip install -r requirements.txt" >&2
    exit 1
}
echo "Checking gsplat's CUDA kernels (the first run compiles them, a few minutes)"
python -c "from gsplat.cuda._backend import _C; assert _C is not None" >/dev/null 2>&1 || {
    echo "gsplat could not load or build its CUDA kernels." >&2
    echo "Install the CUDA toolkit (nvcc) that matches your PyTorch build and set CUDA_HOME." >&2
    exit 1
}

mkdir -p "$OUT_ROOT" "$OUTPUT_ROOT"
export MPS_FOLDER="$(cd "$MPS_FOLDER" && pwd)"     # absolute: stage scripts cd
export OUT_ROOT="$(cd "$OUT_ROOT" && pwd)"
export OUTPUT_ROOT="$(cd "$OUTPUT_ROOT" && pwd)"

RECT_DIR="$OUT_ROOT/$SCENE/$RECT"
MESH_DIR="$OUTPUT_ROOT/$MESH_SUBDIR/$SCENE/$RECT"
SPLAT_DIR="$OUTPUT_ROOT/$SCENE/$RECT/point_cloud"
USDZ="$OUTPUT_ROOT/$SCENE/$RECT/isaacsim/${SCENE}_with_collider.usdz"
if [ "$DECIMATE" = "1" ]; then
    MESH_PLY="$MESH_DIR/mesh_delaunay_decimated.ply"
else
    MESH_PLY="$MESH_DIR/mesh_delaunay.ply"
fi

echo "recording : $VRS"
echo "frames    : $RECT_DIR"
echo "outputs   : $OUTPUT_ROOT"
echo

# Stages
echo "== 1/4 frames"
if [ -f "$RECT_DIR/transforms_with_sparse_depth.json" ]; then
    echo "already done"
else
    bash "$REPO_ROOT/frames/run_gen2_outside.sh"
fi

echo "== 2/4 mesh ($MESH_SOURCE)"
bash "$REPO_ROOT/$MESH_SCRIPT" "$RECT_DIR" "$MESH_DIR"   # before splat: fails fast

echo "== 3/4 splat"
if ls "$SPLAT_DIR"/iteration_*/point_cloud.ply >/dev/null 2>&1; then
    echo "already done"                                  # delete point_cloud/ to retrain
else
    bash "$REPO_ROOT/splat/train_gen2_outside.sh"
fi
SPLAT_PLY="$(ls "$SPLAT_DIR"/iteration_*/point_cloud.ply | sort -V | tail -n 1)"   # highest iteration

echo "== 4/4 scene"
python "$REPO_ROOT/isaacsim/splat_and_mesh_to_usdz.py" \
    --splat "$SPLAT_PLY" --mesh "$MESH_PLY" --out "$USDZ" \
    --radius "$SPLAT_RADIUS" --max-edge "$MAX_EDGE"

echo
echo "Isaac Sim scene:"
echo "  $USDZ"
echo "Open it with File > Open and press Play to turn on collisions."
