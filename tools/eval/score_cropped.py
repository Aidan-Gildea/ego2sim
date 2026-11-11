#!/usr/bin/env python
# Mesh scored against an aligned iPad scan, inside the scan's footprint and contact band.
# Alignment from eval_reference.py's report.json. Paper version: git tag srf-2026.
#
# python tools/eval/score_cropped.py --report <dir>/report.json --mesh mesh.ply
# python tools/eval/score_cropped.py --report <dir>/report.json --mesh mesh.ply --sim3
# writes cropped_<mesh>.json and cropped_<mesh>.png next to report.json

import argparse
import json
import os
from types import SimpleNamespace

import numpy as np
import open3d as o3d
import pandas as pd
from scipy.ndimage import binary_dilation, maximum_filter, minimum_filter
from scipy.spatial import cKDTree

from eval_reference import free_scale_fit, load_mps_points, load_path

TAUS = (0.02, 0.05, 0.10, 0.20)           # distance thresholds (m)


# precision, recall and F at each threshold; trustworthy when tau >= 2 sigma
def scores(acc, comp, sigma_gt):
    out = {}
    for tau in TAUS:
        P = float((acc < tau).mean()) if len(acc) else 0.0
        R = float((comp < tau).mean()) if len(comp) else 0.0
        F = 2 * P * R / (P + R) if P + R else 0.0
        out[f'{int(tau * 100)}cm'] = dict(precision=round(P, 3), recall=round(R, 3), F=round(F, 3),
                                          trustworthy=bool(tau >= 2 * sigma_gt))
    return out


# scan in the MPS frame at 1 cm; with sim3, refit with free scale against the MPS points (not the mesh)
def load_scan(path, T, slam, sim3):
    R = np.asarray(o3d.io.read_point_cloud(path).voxel_down_sample(0.01).points) @ T[:3, :3].T + T[:3, 3]
    if not sim3:
        return R, None

    M = load_mps_points(slam)
    M = M[np.all((M >= R.min(0) - 0.3) & (M <= R.max(0) + 0.3), axis=1)]
    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(M))
    tgt = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(R)).voxel_down_sample(0.02)
    X = free_scale_fit(src, tgt, np.eye(4))
    Xi = np.linalg.inv(X)
    scale = float(np.cbrt(np.linalg.det(X[:3, :3])))          # scan / MPS
    return R @ Xi[:3, :3].T + Xi[:3, 3], scale


# top-down grids of the visible scan: footprint, local ground, local top, ground surface
def ground_grids(Rv, cell, dilate):
    g = SimpleNamespace()
    g.lo = Rv[:, :2].min(0) - 1.0
    ij = np.floor((Rv[:, :2] - g.lo) / cell).astype(int)
    g.shape = tuple(ij.max(0) + 3)
    g.cid = ij[:, 0] * g.shape[1] + ij[:, 1]               # each scan point's cell

    # robust low height (10th percentile) and highest point per cell
    zlow_c = pd.DataFrame({'c': g.cid, 'z': Rv[:, 2]}).groupby('c')['z'].quantile(0.10)
    zlow = np.full(g.shape, np.inf)
    zlow.flat[zlow_c.index.to_numpy()] = zlow_c.to_numpy()
    zmax = np.full(g.shape, -np.inf)
    np.maximum.at(zmax, (ij[:, 0], ij[:, 1]), Rv[:, 2])

    # footprint: occupied cells, grown by dilate cells
    g.foot = binary_dilation(np.isfinite(zlow), structure=np.ones((3, 3), bool), iterations=dilate)

    # local ground and top: lowest low height and highest point within +-0.5 m
    k = 2 * max(1, int(round(0.5 / cell))) + 1
    g.ground = minimum_filter(zlow, size=k, mode='constant', cval=np.inf)
    g.top = maximum_filter(zmax, size=k, mode='constant', cval=-np.inf)
    fallback = float(np.percentile(Rv[:, 2], 2))
    g.ground[~np.isfinite(g.ground)] = fallback
    g.top[~np.isfinite(g.top)] = fallback

    # ground cells sit at the local ground level (not furniture tops); surface = median of the low cluster
    zl_pt = zlow.flat[g.cid]
    g.ground_cell_pt = zl_pt <= g.ground.flat[g.cid] + 0.10
    low = g.ground_cell_pt & (Rv[:, 2] <= zl_pt + 0.06)
    gs_c = pd.Series(Rv[low, 2]).groupby(g.cid[low]).median()
    g.surface = np.full(g.shape, np.nan)
    g.surface.flat[gs_c.index.to_numpy()] = gs_c.to_numpy()
    return g


# grid cell of each point, and whether it falls inside the grid
def cells(P, g, cell):
    c = np.floor((P[:, :2] - g.lo) / cell).astype(int)
    inside = np.all((c >= 0) & (c < np.array(g.shape)), axis=1)
    return np.clip(c, 0, np.array(g.shape) - 1), inside


# top-down error map: footprint mesh samples coloured by distance to the scan
def save_error_map(png, name, scan, samples, dist, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    col = np.where(dist < 0.05, 'tab:green', np.where(dist < 0.10, 'gold', 'tab:red'))
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(scan[::40, 0], scan[::40, 1], s=0.2, c='lightgray', label='iPad scan')
    ax.scatter(samples[:, 0], samples[:, 1], s=0.3, c=col)
    ax.plot(path[:, 0], path[:, 1], 'k-', lw=0.6, label='glasses path')
    ax.set_aspect('equal')
    ax.set_title(f'{name}: mesh in scan footprint (green <5 cm, yellow <10, red >=10)')
    lo, hi = scan.min(0) - 1, scan.max(0) + 1
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    fig.tight_layout()
    fig.savefig(png, dpi=110)


# footprint and contact-band scores for one mesh, written next to the report
def main():
    ap = argparse.ArgumentParser(description='mesh scored against an aligned iPad scan')
    ap.add_argument('--report', required=True, help="eval_reference.py's report.json")
    ap.add_argument('--mesh', required=True, help='mesh to score (.ply)')
    ap.add_argument('--scan', help='iPad scan; default: the one in the report')
    ap.add_argument('--slam', help='MPS slam/ folder; default: the one in the report')
    ap.add_argument('--out', help="output folder; default: the report's folder")
    ap.add_argument('--cell', type=float, default=0.10, help='top-down cell size (m)')
    ap.add_argument('--dilate', type=int, default=2, help='footprint growth (cells)')
    ap.add_argument('--band', type=float, default=1.0, help='contact band height above the ground (m)')
    ap.add_argument('--vis-range', type=float, default=4.0, help='scan within this distance of the path (m)')
    ap.add_argument('--sim3', action='store_true',
                    help='refit the scan with free scale against the MPS points first')
    a = ap.parse_args()

    with open(a.report) as f:
        rep = json.load(f)
    scan_path = a.scan or rep.get('scan_path')
    slam = a.slam or rep.get('slam_path')
    if not scan_path or not slam:
        ap.error('report has no scan_path/slam_path (older eval_reference.py); pass --scan and --slam')
    out_dir = a.out or os.path.dirname(os.path.abspath(a.report))

    o3d.utility.random.seed(0)
    np.random.seed(0)

    # scan in the MPS frame, the part within vis-range of the glasses path
    R, scale_fit = load_scan(scan_path, np.array(rep['T_scan_raw_to_mps']), slam, a.sim3)
    path = load_path(slam)
    Rv = R[cKDTree(path).query(R, k=1)[0] < a.vis_range]
    g = ground_grids(Rv, a.cell, a.dilate)

    # mesh samples, about one per cm^2
    mesh = o3d.io.read_triangle_mesh(a.mesh)
    ns = int(min(4e6, max(1e5, mesh.get_surface_area() / 1e-4)))
    ms = np.asarray(mesh.sample_points_uniformly(ns).points)

    # footprint: 1 m below the local ground to 0.3 m above the local top; band: up to --band above ground
    cm, inside = cells(ms, g, a.cell)
    g_m, t_m = g.ground[cm[:, 0], cm[:, 1]], g.top[cm[:, 0], cm[:, 1]]
    in_foot = inside & g.foot[cm[:, 0], cm[:, 1]] & (ms[:, 2] >= g_m - 1.0) & (ms[:, 2] <= t_m + 0.3)
    in_band = in_foot & (ms[:, 2] <= g_m + a.band)

    # scan points in the band, and on the ground surface (within 3 cm of it)
    r_band = Rv[:, 2] <= g.ground.flat[g.cid] + a.band
    gs_r = g.surface.flat[g.cid]
    r_ground = g.ground_cell_pt & np.isfinite(gs_r) & (np.abs(Rv[:, 2] - np.nan_to_num(gs_r, nan=1e9)) < 0.03)

    # precision: cropped mesh vs all visible scan; recall: scan vs all mesh, so crop edges cost neither
    tree_R, tree_M = cKDTree(Rv), cKDTree(ms)
    acc_foot = tree_R.query(ms[in_foot], k=1)[0]
    acc_band = tree_R.query(ms[in_band], k=1)[0]
    comp = tree_M.query(Rv, k=1)[0]
    sigma_gt = float(rep.get('sigma_gt_cm', 0)) / 100

    out = dict(
        mesh=a.mesh,
        crop=dict(cell_m=a.cell, dilate_cells=a.dilate, band_m=a.band, vis_range_m=a.vis_range),
        sim3=bool(a.sim3),
        scale_fit_scan_over_mps=None if scale_fit is None else round(scale_fit, 4),
        mesh_samples=int(len(ms)),
        samples_in_footprint=int(in_foot.sum()),
        samples_in_band=int(in_band.sum()),
        reference_points=int(len(Rv)),
        reference_in_band=int(r_band.sum()),
        reference_ground=int(r_ground.sum()),
        footprint_area_m2=round(float(g.foot.sum()) * a.cell ** 2, 1),
        footprint=dict(accuracy_median_cm=round(100 * float(np.median(acc_foot)), 2),
                       completeness_median_cm=round(100 * float(np.median(comp)), 2),
                       scores=scores(acc_foot, comp, sigma_gt)),
        contact_band=dict(accuracy_median_cm=round(100 * float(np.median(acc_band)), 2),
                          completeness_median_cm=round(100 * float(np.median(comp[r_band])), 2),
                          scores=scores(acc_band, comp[r_band], sigma_gt)),
        ground_covered_within_5cm=round(float((comp[r_ground] < 0.05).mean()), 3),
        ground_covered_within_10cm=round(float((comp[r_ground] < 0.10).mean()), 3),
        sigma_gt_cm=rep.get('sigma_gt_cm'))

    name = os.path.splitext(os.path.basename(a.mesh))[0] + ('_sim3' if a.sim3 else '')
    try:
        save_error_map(os.path.join(out_dir, f'cropped_{name}.png'), name,
                       Rv, ms[in_foot][::10], acc_foot[::10], path)
    except Exception as e:
        out['png_error'] = str(e)

    with open(os.path.join(out_dir, f'cropped_{name}.json'), 'w') as f:
        json.dump(out, f, indent=2)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
