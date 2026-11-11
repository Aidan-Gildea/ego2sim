#!/bin/bash
# Collider mesh from the MPS semi-dense points instead of patch-match MVS. The points and the SLAM views that
# saw them are written as a COLMAP dense workspace, then meshed and scored with the same delaunay_mesher and
# evaluator as mesh/photogrammetry/run_photogrammetry.sh, so the two meshes differ only in their points.
# Each stage is skipped once done; settings below can be set before the command.
#
# bash mesh/mps-mesh/run_mps_mesh.sh <rectified_dir>
# bash mesh/mps-mesh/run_mps_mesh.sh <rectified_dir> <out_dir>

set -euo pipefail

# <rectified_dir>: processed/<recording>/camera-rgb-rectified-<focal>-h<height>, from extract_aria_vrs.py
RECT_DIR="${1:?usage: run_mps_mesh.sh <rectified_dir> [out_dir]}"
RECT_DIR="$(cd "$RECT_DIR" && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"

RECT_NAME="$(basename "$RECT_DIR")"
SCENE_NAME="$(basename "$(dirname "$RECT_DIR")")"
OUT_DIR="${2:-$REPO_ROOT/output/mps-mesh/$SCENE_NAME/$RECT_NAME}"
mkdir -p "$OUT_DIR/logs" "$OUT_DIR/.done"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

EYE_HEIGHT="${EYE_HEIGHT:-1.6683}"     # trajectory-to-floor distance (m), for the evaluator
DECIMATE="${DECIMATE:-1}"              # 1 also writes the simplified collider, 0 full mesh only
FORCE="${FORCE:-0}"                    # 1 ignores finished stages and reruns all
PYTHON="${PYTHON:-python}"             # interpreter with numpy, pandas, open3d

DENSE="$OUT_DIR/dense"
MESH="$OUT_DIR/mesh_delaunay.ply"
MESH_DECIMATED="$OUT_DIR/mesh_delaunay_decimated.ply"

# preflight
[ -e "$RECT_DIR/semidense_points.csv.gz" ] || { echo "no semidense_points.csv.gz in $RECT_DIR" >&2; exit 1; }
[ -f "$REPO_ROOT/mesh/photogrammetry/evaluate_mesh.py" ] || { echo "mesh/photogrammetry/evaluate_mesh.py not found" >&2; exit 1; }
command -v colmap >/dev/null || { echo "colmap not on PATH" >&2; exit 1; }
"$PYTHON" -c "import numpy, pandas, open3d" 2>/dev/null \
    || { echo "PYTHON=$PYTHON lacks numpy/pandas/open3d; conda activate ego_splats" >&2; exit 1; }

echo "rectified : $RECT_DIR"
echo "output    : $OUT_DIR"
echo

# marker file of a finished stage
done_marker() { echo "$OUT_DIR/.done/$1"; }
# true if the stage has a marker and FORCE is not 1
is_done()     { [ "$FORCE" != "1" ] && [ -f "$(done_marker "$1")" ]; }
# marker holding the finish time
mark_done()   { date -Is > "$(done_marker "$1")"; }

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

# 1. prepare: MPS points and views as a COLMAP dense workspace
run_stage prepare "$PYTHON" "$HERE/mps_points_to_colmap.py" \
    --rectified_dir "$RECT_DIR" --out_dir "$OUT_DIR"

# 2. mesh (default delaunay_mesher options)
run_stage mesh colmap delaunay_mesher \
    --input_path "$DENSE" \
    --input_type dense \
    --output_path "$MESH"

# 3. evaluate; --fused is the MPS cloud, so the report's fused rows describe the mesher's input
run_stage evaluate "$PYTHON" "$REPO_ROOT/mesh/photogrammetry/evaluate_mesh.py" \
    --mesh "$MESH" \
    --rectified_dir "$RECT_DIR" \
    --fused "$DENSE/fused.ply" \
    --eye_height "$EYE_HEIGHT" \
    --out "$OUT_DIR/report.json"

# 4. decimate (on by default here): >2 m spike triangles dropped, then quadric decimation at about 1 cm
if [ "$DECIMATE" = "1" ]; then
    run_stage decimate "$PYTHON" "$REPO_ROOT/mesh/photogrammetry/decimate_mesh.py" "$MESH" "$MESH_DECIMATED"
    run_stage evaluate_decimated "$PYTHON" "$REPO_ROOT/mesh/photogrammetry/evaluate_mesh.py" \
        --mesh "$MESH_DECIMATED" \
        --rectified_dir "$RECT_DIR" \
        --fused "$DENSE/fused.ply" \
        --eye_height "$EYE_HEIGHT" \
        --out "$OUT_DIR/report_decimated.json"
fi

echo "Mesh (MPS world frame, metres, Z-up):"
echo "  $MESH"
echo "Report:"
echo "  $OUT_DIR/report.md"
if [ "$DECIMATE" = "1" ]; then
    echo "Simplified mesh, used as the Isaac Sim collider:"
    echo "  $MESH_DECIMATED"
    echo "  $OUT_DIR/report_decimated.md"
fi
