#!/bin/bash
# Photogrammetry mesh from a posed, rectified Aria dataset. COLMAP's pose solver is bypassed (MPS poses stay
# fixed), so the mesh lands in the MPS world frame, the same frame as the trained splat. Stages: keyframes,
# SIFT, matching, triangulation, patch-match stereo, fusion, Delaunay mesh, evaluation, each skipped once
# done; settings below can be set before the command.
#
# bash mesh/photogrammetry/run_photogrammetry.sh <rectified_dir>
# bash mesh/photogrammetry/run_photogrammetry.sh <rectified_dir> <out_dir>

set -euo pipefail

# <rectified_dir>: processed/<recording>/camera-rgb-rectified-<focal>-h<height>, from extract_aria_vrs.py
RECT_DIR="${1:?usage: run_photogrammetry.sh <rectified_dir> [out_dir]}"
RECT_DIR="$(cd "$RECT_DIR" && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"

RECT_NAME="$(basename "$RECT_DIR")"
SCENE_NAME="$(basename "$(dirname "$RECT_DIR")")"
OUT_DIR="${2:-$REPO_ROOT/output/photogrammetry/$SCENE_NAME/$RECT_NAME}"
mkdir -p "$OUT_DIR/logs" "$OUT_DIR/.done"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

START="${START:-0}"                           # time window start, s after the first posed frame
DURATION="${DURATION:--1}"                    # time window length (s), -1 to the end
MIN_TRANSLATION="${MIN_TRANSLATION:-0.10}"    # keyframe spacing (m)
MIN_ROTATION_DEG="${MIN_ROTATION_DEG:-10}"    # keyframe spacing (deg)
MAX_KEYFRAMES="${MAX_KEYFRAMES:--1}"          # keyframe cap by uniform thinning, -1 none
MAX_IMAGE_SIZE="${MAX_IMAGE_SIZE:-1008}"      # longest side for dense stereo (2016 native, 4x slower)
NUM_SRC_IMAGES="${NUM_SRC_IMAGES:-20}"        # patch-match source images per reference image
SEQ_OVERLAP="${SEQ_OVERLAP:-20}"              # sequential matcher window (plus powers of two beyond)
LOOP_DETECTION="${LOOP_DETECTION:-0}"         # 1 vocab-tree loop detection (downloads the tree once)
GPU_INDEX="${GPU_INDEX:-0}"                   # CUDA device for SIFT and patch-match
CACHE_GB="${CACHE_GB:-16}"                    # patch-match and fusion image cache (GB)
EYE_HEIGHT="${EYE_HEIGHT:-1.6683}"            # trajectory-to-floor distance (m), for the evaluator
STOP_AFTER="${STOP_AFTER:-}"                  # stage to stop after, e.g. triangulate before the GPU hours
FORCE="${FORCE:-0}"                           # 1 ignores finished stages and reruns all
DECIMATE="${DECIMATE:-0}"                     # 1 also writes and scores a simplified copy for simulation
PYTHON="${PYTHON:-python}"                    # interpreter with numpy, pandas, open3d

IMAGES="$RECT_DIR/images"
DB="$OUT_DIR/database.db"
SPARSE_IN="$OUT_DIR/sparse_in"
SPARSE="$OUT_DIR/sparse"
DENSE="$OUT_DIR/dense"
MESH="$OUT_DIR/mesh_delaunay.ply"
MESH_DECIMATED="$OUT_DIR/mesh_delaunay_decimated.ply"

# preflight
[ -d "$IMAGES" ] || { echo "no images/ in $RECT_DIR" >&2; exit 1; }
[ -f "$RECT_DIR/transforms.json" ] || { echo "no transforms.json in $RECT_DIR" >&2; exit 1; }
command -v colmap >/dev/null || { echo "colmap not on PATH" >&2; exit 1; }
"$PYTHON" -c "import numpy, pandas, open3d" 2>/dev/null \
    || { echo "PYTHON=$PYTHON lacks numpy/pandas/open3d; conda activate ego_splats" >&2; exit 1; }
if command -v nvidia-smi >/dev/null; then     # warning if another process holds the GPU
    BUSY="$(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader 2>/dev/null || true)"
    [ -n "$BUSY" ] && echo "WARNING: GPU already in use:" && echo "$BUSY"
fi

echo "rectified : $RECT_DIR"
echo "output    : $OUT_DIR"
echo "colmap    : $(colmap -h 2>&1 | head -1)"
echo

# marker file of a finished stage
done_marker() { echo "$OUT_DIR/.done/$1"; }
# true if the stage has a marker and FORCE is not 1
is_done()     { [ "$FORCE" != "1" ] && [ -f "$(done_marker "$1")" ]; }
# marker holding the finish time
mark_done()   { date -Is > "$(done_marker "$1")"; }
# exit 0 here if STOP_AFTER names this stage
maybe_stop()  { [ "$STOP_AFTER" = "$1" ] && { echo; echo "STOP_AFTER=$1 reached."; exit 0; } || true; }
# fail_stage <name> <message>: marker removed, message printed, exit 1
fail_stage()  { rm -f "$(done_marker "$1")"; echo "== $1 FAILED: $2" >&2; exit 1; }

# run_stage <name> <command...>: one stage, logged to logs/<name>.log, skipped if done, exits on failure
run_stage() {
    local name="$1"; shift
    if is_done "$name"; then
        echo "== $name: already done ($(cat "$(done_marker "$name")")), skipping"
        return 0
    fi
    echo "== $name: $(date -Is)"
    local t0=$SECONDS
    "$@" 2>&1 | tee "$OUT_DIR/logs/$name.log"
    local rc=${PIPESTATUS[0]}
    if [ "$rc" -ne 0 ]; then
        echo "== $name FAILED (exit $rc), see $OUT_DIR/logs/$name.log" >&2
        exit "$rc"
    fi
    mark_done "$name"
    echo "== $name: done in $(( SECONDS - t0 )) s"
    echo
}

# 1. prepare: keyframes and the fixed-pose COLMAP model
run_stage prepare "$PYTHON" "$HERE/colmap_from_transforms.py" \
    --rectified_dir "$RECT_DIR" --out_dir "$OUT_DIR" \
    --start "$START" --duration "$DURATION" \
    --min_translation "$MIN_TRANSLATION" --min_rotation_deg "$MIN_ROTATION_DEG" \
    --max_keyframes "$MAX_KEYFRAMES"
maybe_stop prepare

N_KEYFRAMES="$(grep -c . "$OUT_DIR/keyframes.txt")"
CAM_PARAMS="$(awk '!/^#/ {print $5","$6","$7","$8; exit}' "$SPARSE_IN/cameras.txt")"   # fx,fy,cx,cy

# 2. features: SIFT on the keyframes only, one shared PINHOLE camera that matches the fixed-pose model
if [ "$FORCE" = "1" ] && [ -f "$DB" ]; then rm -f "$DB"; fi     # fresh database with FORCE=1
run_stage features colmap feature_extractor \
    --database_path "$DB" \
    --image_path "$IMAGES" \
    --image_list_path "$OUT_DIR/keyframes.txt" \
    --ImageReader.camera_model PINHOLE \
    --ImageReader.single_camera 1 \
    --ImageReader.camera_params "$CAM_PARAMS" \
    --FeatureExtraction.use_gpu 1 \
    --FeatureExtraction.gpu_index "$GPU_INDEX"
maybe_stop features

# 3. match: sequential (a continuous walk); quadratic_overlap also pairs i with i + 2^k to catch revisits
run_stage match colmap sequential_matcher \
    --database_path "$DB" \
    --SequentialMatching.overlap "$SEQ_OVERLAP" \
    --SequentialMatching.quadratic_overlap 1 \
    --SequentialMatching.loop_detection "$LOOP_DETECTION" \
    --FeatureMatching.use_gpu 1 \
    --FeatureMatching.gpu_index "$GPU_INDEX"
maybe_stop match

# 4. triangulate: 3D points only, poses and intrinsics held fixed; in place of mapper, keeps the MPS frame
mkdir -p "$SPARSE"
run_stage triangulate colmap point_triangulator \
    --database_path "$DB" \
    --image_path "$IMAGES" \
    --input_path "$SPARSE_IN" \
    --output_path "$SPARSE" \
    --clear_points 1 \
    --refine_intrinsics 0 \
    --Mapper.ba_refine_focal_length 0 \
    --Mapper.ba_refine_principal_point 0 \
    --Mapper.ba_refine_extra_params 0
# text copy of the model, its statistics, and a check that every keyframe registered with points
colmap model_converter --input_path "$SPARSE" --output_path "$SPARSE" --output_type TXT >/dev/null 2>&1 || true
colmap model_analyzer --path "$SPARSE" 2>&1 | tee "$OUT_DIR/logs/triangulate_stats.log" || true
N_REG="$(grep -oE "Registered images: [0-9]+" "$OUT_DIR/logs/triangulate_stats.log" | grep -oE "[0-9]+$" || echo 0)"
N_PTS="$(grep -oE "Points: [0-9]+" "$OUT_DIR/logs/triangulate_stats.log" | grep -oE "[0-9]+$" || echo 0)"
if [ "$N_REG" -lt "$N_KEYFRAMES" ] || [ "$N_PTS" -eq 0 ]; then
    fail_stage triangulate "$N_REG of $N_KEYFRAMES keyframes registered, $N_PTS points; see logs/triangulate_stats.log"
fi
echo "   $N_REG keyframes, $N_PTS points, $(grep -oE "Mean reprojection error: [0-9.]+px" "$OUT_DIR/logs/triangulate_stats.log")"
maybe_stop triangulate

# 5. undistort: images are already pinhole, so only the dense workspace is laid out (images linked, full size)
# no --max_image_size here: COLMAP 3.13 resizes the images but not the camera, and stereo rejects every view;
# stereo and fusion downscale instead
run_stage undistort colmap image_undistorter \
    --image_path "$IMAGES" \
    --input_path "$SPARSE" \
    --output_path "$DENSE" \
    --output_type COLMAP \
    --copy_policy soft-link \
    --num_patch_match_src_images "$NUM_SRC_IMAGES"
maybe_stop undistort

# 6. stereo (GPU, the slow one)
run_stage stereo colmap patch_match_stereo \
    --workspace_path "$DENSE" \
    --workspace_format COLMAP \
    --PatchMatchStereo.max_image_size "$MAX_IMAGE_SIZE" \
    --PatchMatchStereo.geom_consistency 1 \
    --PatchMatchStereo.gpu_index "$GPU_INDEX" \
    --PatchMatchStereo.cache_size "$CACHE_GB"
# depth maps counted, since COLMAP exits 0 even when views fail
N_DEPTH="$(ls "$DENSE/stereo/depth_maps"/*.geometric.bin 2>/dev/null | wc -l)"
if [ "$N_DEPTH" -lt "$N_KEYFRAMES" ]; then
    fail_stage stereo "only $N_DEPTH of $N_KEYFRAMES geometric depth maps were written; see logs/stereo.log"
fi
echo "   $N_DEPTH geometric depth maps"
maybe_stop stereo

# 7. fuse
run_stage fuse colmap stereo_fusion \
    --workspace_path "$DENSE" \
    --workspace_format COLMAP \
    --input_type geometric \
    --output_path "$DENSE/fused.ply" \
    --StereoFusion.max_image_size "$MAX_IMAGE_SIZE" \
    --StereoFusion.cache_size "$CACHE_GB"
# point count from the PLY header
N_FUSED="$(head -c 400 "$DENSE/fused.ply" | awk '/^element vertex/ {print $3; exit}')"
if [ -z "$N_FUSED" ] || [ "$N_FUSED" -eq 0 ]; then
    fail_stage fuse "fused.ply holds no points; see logs/fuse.log"
fi
echo "   fused cloud: $N_FUSED points"
maybe_stop fuse

# 8. mesh
run_stage mesh colmap delaunay_mesher \
    --input_path "$DENSE" \
    --input_type dense \
    --output_path "$MESH"
maybe_stop mesh

# 9. evaluate
run_stage evaluate "$PYTHON" "$HERE/evaluate_mesh.py" \
    --mesh "$MESH" \
    --rectified_dir "$RECT_DIR" \
    --fused "$DENSE/fused.ply" \
    --keyframes_json "$OUT_DIR/keyframes.json" \
    --eye_height "$EYE_HEIGHT" \
    --out "$OUT_DIR/report.json"

# 10. decimate (DECIMATE=1): >2 m spikes dropped, quadric decimation at about 1 cm; the full mesh is kept
if [ "$DECIMATE" = "1" ]; then
    run_stage decimate "$PYTHON" "$HERE/decimate_mesh.py" "$MESH" "$MESH_DECIMATED"
    maybe_stop decimate
    run_stage evaluate_decimated "$PYTHON" "$HERE/evaluate_mesh.py" \
        --mesh "$MESH_DECIMATED" \
        --rectified_dir "$RECT_DIR" \
        --fused "$DENSE/fused.ply" \
        --keyframes_json "$OUT_DIR/keyframes.json" \
        --eye_height "$EYE_HEIGHT" \
        --out "$OUT_DIR/report_decimated.json"
fi

echo "Mesh (MPS world frame, metres, Z-up):"
echo "  $MESH"
echo "Report:"
echo "  $OUT_DIR/report.md"
if [ "$DECIMATE" = "1" ]; then
    echo "Simplified mesh and its report:"
    echo "  $MESH_DECIMATED"
    echo "  $OUT_DIR/report_decimated.md"
fi
