#!/usr/bin/env python3
# Mesh in the MPS world frame scored against two independent MPS sources: the closed-loop trajectory (where
# the floor must be) and the semi-dense points (where surfaces were seen). Metrics: floor coverage and largest
# hole along the walked path, semi-dense point-to-mesh distance, mesh hygiene and, with --fused, how much of
# the fused MVS cloud is floor (patch-match MVS tends to miss untextured ground).
#
# python mesh/photogrammetry/evaluate_mesh.py --mesh mesh.ply --rectified_dir <rectified_dir> --out report.json
# writes report.json and report.md next to it

import argparse
import json
from collections import deque
from pathlib import Path

import numpy as np
import open3d as o3d
import pandas as pd

CELL = 0.10          # 10 cm floor grid cells
BUFFER = 0.75        # floor cells within this of the walked path are scored (m)
FLOOR_TOL = 0.15     # hits within this of the expected floor height count as floor (m)
RAY_START = 0.50     # rays cast down from this far above the expected floor (m)
FUSED_BAND = 0.20    # fused points within this of the expected floor count as floor (m)
SMALL_COMPONENT_AREA = 0.10   # m^2; smaller connected components count as junk
INV_DIST_STD_THR = 0.005      # same semi-dense confidence filter as extract_aria_vrs.py
DIST_STD_THR = 0.01


# device positions at about hz, optionally only within t_range_ns (device time, the keyframes' span)
def load_trajectory(csv_path: Path, hz: float = 10.0, t_range_ns=None) -> np.ndarray:
    df = pd.read_csv(csv_path, usecols=["tracking_timestamp_us", "tx_world_device",
                                        "ty_world_device", "tz_world_device"])
    if t_range_ns is not None:
        t0_us, t1_us = t_range_ns[0] / 1e3, t_range_ns[1] / 1e3
        df = df[(df["tracking_timestamp_us"] >= t0_us) & (df["tracking_timestamp_us"] <= t1_us)]
        if len(df) == 0:
            raise SystemExit("no trajectory samples inside the keyframe time span")
    t_us = df["tracking_timestamp_us"].to_numpy()
    if len(t_us) > 1:
        dt_s = np.median(np.diff(t_us)) / 1e6
        stride = max(1, int(round(1.0 / (hz * dt_s))))
    else:
        stride = 1
    return df[["tx_world_device", "ty_world_device", "tz_world_device"]].to_numpy()[::stride]


# confident semi-dense points, N x 3
def load_semidense(path: Path) -> np.ndarray:
    df = pd.read_csv(path, usecols=["px_world", "py_world", "pz_world", "inv_dist_std", "dist_std"])
    keep = (df["inv_dist_std"] < INV_DIST_STD_THR) & (df["dist_std"] < DIST_STD_THR)
    return df.loc[keep, ["px_world", "py_world", "pz_world"]].to_numpy(dtype=np.float64)


# index of and XY distance to the nearest ref point, for every query point (brute force, in chunks)
def nearest_index(query_xy: np.ndarray, ref_xy: np.ndarray, chunk: int = 4096):
    idx = np.empty(len(query_xy), dtype=np.int64)
    dist = np.empty(len(query_xy), dtype=np.float64)
    for s in range(0, len(query_xy), chunk):
        q = query_xy[s:s + chunk]
        d2 = ((q[:, None, :] - ref_xy[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        idx[s:s + chunk] = j
        dist[s:s + chunk] = np.sqrt(d2[np.arange(len(q)), j])
    return idx, dist


# cells within BUFFER of the path: centres (N, 2), expected floor z (N,), grid (ix, iy) for hole finding
def build_floor_grid(traj: np.ndarray, eye_height: float):
    lo = traj[:, :2].min(0) - BUFFER
    hi = traj[:, :2].max(0) + BUFFER
    nx = int(np.ceil((hi[0] - lo[0]) / CELL)) + 1
    ny = int(np.ceil((hi[1] - lo[1]) / CELL)) + 1
    ix, iy = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
    centres = np.stack([lo[0] + (ix.ravel() + 0.5) * CELL,
                        lo[1] + (iy.ravel() + 0.5) * CELL], axis=1)
    j, d = nearest_index(centres, traj[:, :2])
    inside = d <= BUFFER
    return (centres[inside], traj[j[inside], 2] - eye_height,
            np.stack([ix.ravel()[inside], iy.ravel()[inside]], axis=1))


# size of the largest 4-connected block of missing cells
def largest_hole_cells(grid_ij: np.ndarray, missing: np.ndarray) -> int:
    missing_set = {tuple(c) for c in grid_ij[missing]}
    seen = set()
    best = 0
    for start in missing_set:
        if start in seen:
            continue
        seen.add(start)
        q = deque([start])
        size = 0
        while q:
            i, j = q.popleft()
            size += 1
            for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if n in missing_set and n not in seen:
                    seen.add(n)
                    q.append(n)
        best = max(best, size)
    return best


# q-th percentile, NaN for an empty array
def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


# report.json and report.md for one mesh
def main():
    ap = argparse.ArgumentParser(description="mesh scored against the MPS trajectory and semi-dense points",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mesh", required=True, type=Path)
    ap.add_argument("--rectified_dir", required=True, type=Path,
                    help="folder holding closed_loop_trajectory.csv and semidense_points.csv.gz "
                         "(extract_aria_vrs.py symlinks both into the rectified folder)")
    ap.add_argument("--fused", type=Path, default=None, help="optional fused.ply from stereo_fusion")
    ap.add_argument("--eye_height", type=float, default=1.6683,
                    help="metres from the device trajectory down to the floor")
    ap.add_argument("--keyframes_json", type=Path, default=None,
                    help="keyframes.json from colmap_from_transforms.py; restricts the floor "
                         "metric to the trajectory span the keyframes cover")
    ap.add_argument("--out", required=True, type=Path, help="report JSON path")
    args = ap.parse_args()

    # keyframes' time span, so a windowed run is scored on its window
    t_range_ns = None
    if args.keyframes_json is not None and args.keyframes_json.exists():
        with open(args.keyframes_json) as f:
            kf_ts = json.load(f)["keyframe_timestamps_ns"]
        t_range_ns = (min(kf_ts), max(kf_ts))

    traj_csv = args.rectified_dir / "closed_loop_trajectory.csv"
    semi_csv = args.rectified_dir / "semidense_points.csv.gz"
    for p in (traj_csv, semi_csv, args.mesh):
        if not p.exists():
            raise SystemExit(f"missing: {p}")

    mesh = o3d.io.read_triangle_mesh(str(args.mesh))
    if len(mesh.triangles) == 0:
        raise SystemExit(f"{args.mesh} has no triangles")
    verts = np.asarray(mesh.vertices)
    aabb_min, aabb_max = verts.min(0), verts.max(0)

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

    report = {"mesh": str(args.mesh.resolve()), "eye_height_m": args.eye_height,
              "trajectory_span_ns": list(t_range_ns) if t_range_ns else "whole recording"}

    # hygiene: size, bounds, connected components
    tri_clusters, cluster_n_tri, cluster_area = mesh.cluster_connected_triangles()
    cluster_area = np.asarray(cluster_area)
    report["hygiene"] = {
        "vertices": int(len(mesh.vertices)),
        "triangles": int(len(mesh.triangles)),
        "aabb_min_m": aabb_min.round(3).tolist(),
        "aabb_max_m": aabb_max.round(3).tolist(),
        "surface_area_m2": float(mesh.get_surface_area()),
        "connected_components": int(len(cluster_area)),
        "small_component_area_m2": float(cluster_area[cluster_area < SMALL_COMPONENT_AREA].sum()),
        "largest_component_area_m2": float(cluster_area.max()),
    }

    # floor coverage: rays cast down near the walked path, hits within FLOOR_TOL of the expected floor
    traj = load_trajectory(traj_csv, t_range_ns=t_range_ns)
    centres, floor_z, grid_ij = build_floor_grid(traj, args.eye_height)
    origins = np.column_stack([centres, floor_z + RAY_START]).astype(np.float32)
    dirs = np.tile(np.array([0, 0, -1], dtype=np.float32), (len(origins), 1))
    rays = o3d.core.Tensor(np.hstack([origins, dirs]), dtype=o3d.core.Dtype.Float32)
    t_hit = scene.cast_rays(rays)["t_hit"].numpy()
    hit_z = origins[:, 2] - t_hit
    err = hit_z - floor_z
    covered = np.isfinite(t_hit) & (np.abs(err) <= FLOOR_TOL)
    missing = ~covered
    largest_hole = largest_hole_cells(grid_ij, missing)
    report["floor"] = {
        "cells": int(len(centres)),
        "cell_size_m": CELL,
        "walked_area_m2": float(len(centres) * CELL * CELL),
        "coverage": float(covered.mean()),
        "largest_hole_m2": float(largest_hole * CELL * CELL),
        "height_error_covered_cm": {
            "median": float(np.median(np.abs(err[covered])) * 100) if covered.any() else None,
            "p95": pct(np.abs(err[covered]) * 100, 95) if covered.any() else None,
        },
        "expected_floor_z_m": {"min": float(floor_z.min()), "max": float(floor_z.max())},
    }

    # semi-dense agreement: point-to-mesh distance, for all points and for those near the mesh's bbox
    pts = load_semidense(semi_csv)
    d_all = scene.compute_distance(o3d.core.Tensor(pts.astype(np.float32))).numpy()
    pad = 0.5                                     # bbox margin (m)
    in_box = np.all((pts >= aabb_min - pad) & (pts <= aabb_max + pad), axis=1)
    d_box = d_all[in_box]

    # count, mean / median / p95 and fraction within 5 and 10 cm of a set of distances
    def summarise(d):
        if len(d) == 0:
            return {"count": 0}
        return {"count": int(len(d)),
                "mean_cm": float(d.mean() * 100), "median_cm": float(np.median(d) * 100),
                "p95_cm": pct(d * 100, 95),
                "within_5cm": float((d < 0.05).mean()), "within_10cm": float((d < 0.10).mean())}

    report["semidense"] = {
        "confidence_filter": {"inv_dist_std_lt": INV_DIST_STD_THR, "dist_std_lt": DIST_STD_THR},
        "all_points": summarise(d_all),
        "points_inside_mesh_bbox": summarise(d_box),
        "fraction_inside_mesh_bbox": float(in_box.mean()),
    }

    # fused cloud near the path: floor band vs. everything else
    if args.fused is not None and args.fused.exists():
        fused = np.asarray(o3d.io.read_point_cloud(str(args.fused)).points)
        j, d = nearest_index(fused[:, :2], traj[:, :2])
        near_path = d <= BUFFER
        exp_floor = traj[j, 2] - args.eye_height
        floor_band = near_path & (np.abs(fused[:, 2] - exp_floor) <= FUSED_BAND)
        above = near_path & ~floor_band
        # floor cells with at least one fused floor point
        cell_of_pt = np.floor((fused[floor_band, :2] - (traj[:, :2].min(0) - BUFFER)) / CELL).astype(int)
        hit_cells = {tuple(c) for c in cell_of_pt}
        cells_hit = np.array([tuple(c) in hit_cells for c in grid_ij])
        report["fused"] = {
            "points": int(len(fused)),
            "near_path_floor_band": int(floor_band.sum()),
            "near_path_above_floor": int(above.sum()),
            "far_from_path": int((~near_path).sum()),
            "floor_band_fraction_of_near_path": float(floor_band.sum() / max(1, near_path.sum())),
            "floor_cells_with_fused_points": float(cells_hit.mean()),
            "fused_floor_points_per_m2": float(floor_band.sum() / (len(centres) * CELL * CELL)),
        }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(report, f, indent=2)

    # markdown table of the main numbers
    h, fl, sd = report["hygiene"], report["floor"], report["semidense"]
    lines = [f"# Mesh report: `{args.mesh.name}`", "",
             f"eye height {args.eye_height} m; floor cells {fl['cells']} x {CELL} m "
             f"({fl['walked_area_m2']:.1f} m^2 of walked floor)", "",
             "| metric | value |", "|---|---|",
             f"| triangles | {h['triangles']:,} |",
             f"| AABB (m) | {h['aabb_min_m']} .. {h['aabb_max_m']} |",
             f"| connected components | {h['connected_components']:,} |",
             f"| junk area (< {SMALL_COMPONENT_AREA} m^2 components) | "
             f"{h['small_component_area_m2']:.2f} m^2 |",
             f"| **floor coverage** | **{fl['coverage']:.4f}** |",
             f"| **largest hole** | **{fl['largest_hole_m2']:.2f} m^2** |",
             f"| floor height error (covered), median / p95 | "
             f"{fl['height_error_covered_cm']['median']} / {fl['height_error_covered_cm']['p95']} cm |",
             f"| semi-dense median / p95 (all {sd['all_points'].get('count', 0):,} pts) | "
             f"{sd['all_points'].get('median_cm', float('nan')):.2f} / "
             f"{sd['all_points'].get('p95_cm', float('nan')):.2f} cm |",
             f"| semi-dense median / p95 (inside bbox, "
             f"{sd['points_inside_mesh_bbox'].get('count', 0):,} pts) | "
             f"{sd['points_inside_mesh_bbox'].get('median_cm', float('nan')):.2f} / "
             f"{sd['points_inside_mesh_bbox'].get('p95_cm', float('nan')):.2f} cm |",
             f"| semi-dense within 10 cm (inside bbox) | "
             f"{sd['points_inside_mesh_bbox'].get('within_10cm', float('nan')):.3f} |"]
    if "fused" in report:
        fu = report["fused"]
        lines += [f"| fused points | {fu['points']:,} |",
                  f"| fused floor-band / above-floor / far | {fu['near_path_floor_band']:,} / "
                  f"{fu['near_path_above_floor']:,} / {fu['far_from_path']:,} |",
                  f"| floor cells with any fused point | {fu['floor_cells_with_fused_points']:.4f} |"]
    md_path = args.out.with_suffix(".md")
    md_path.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\nwrote {args.out} and {md_path}")


if __name__ == "__main__":
    main()
