#!/usr/bin/env python
# iPad LiDAR scan (RTAB-Map point cloud) aligned to the Aria MPS world frame, plus checks on the fit.
# Paper version: git tag srf-2026.
#
# python tools/eval/eval_reference.py --ipad room.ply --slam <mps_..._vrs/slam> --out <dir>
# writes report.json (with T_scan_raw_to_mps), scan_in_mps_frame.ply, alignment_overlay.png

import argparse
import json
import os

import numpy as np
import open3d as o3d
import pandas as pd
from scipy.ndimage import binary_dilation
from scipy.signal import fftconvolve
from scipy.spatial import cKDTree

reg = o3d.pipelines.registration

INV_DIST_STD, DIST_STD = 0.005, 0.01      # confidence filter for aria SLAM points
GRID = 0.05                               # 5 cm cells for the top-down maps
BAND = (0.3, 2.0)                         # heights above the floor that count as walls and furniture
UP = np.array([0, 0, 1.0])


# smallest rotation taking unit vector a onto unit vector b (Rodrigues)
def rot_a_to_b(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


# turn about the vertical axis, as a 3x3 rotation
def rot_z(deg):
    th = np.radians(deg)
    return np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])


# 4x4 transform from a rotation and a translation
def rigid(R=np.eye(3), t=np.zeros(3)):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


# angle between two unit normals in degrees, ignoring their sign
def angle_deg(a, b):
    return round(float(np.degrees(np.arccos(min(1.0, abs(a @ b))))), 3)


# largest horizontal or vertical plane: normal, centre, RMS of points within thresh, count
def plane_rms(pc, thresh=0.05, horizontal=None):
    rest = pc
    for _ in range(8):
        model, inl = rest.segment_plane(0.02, 3, 3000)
        n = np.array(model[:3])
        n /= np.linalg.norm(n)
        is_horizontal, is_vertical = abs(n[2]) > 0.9, abs(n[2]) < 0.15
        if horizontal is None or (horizontal and is_horizontal) or (not horizontal and is_vertical):
            P = np.asarray(rest.points)
            sel = P[np.abs(P @ n + model[3]) < thresh]
            c = sel.mean(0)
            n2 = np.linalg.svd(sel - c, full_matrices=False)[2][2]
            n2 = n2 * np.sign(n2 @ n)
            r = (sel - c) @ n2
            return n2, c, float(np.sqrt(np.mean(r ** 2))), len(sel)
        rest = rest.select_by_index(inl, invert=True)
    raise RuntimeError('no plane of the requested kind found')


# floor normal: RANSAC plane, then an SVD refit on points within 3 cm
def floor_normal(P):
    m, _ = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)).segment_plane(0.02, 3, 5000)
    n0 = np.array(m[:3]) / np.linalg.norm(m[:3])
    sel = P[np.abs(P @ n0 + m[3] / np.linalg.norm(m[:3])) < 0.03]
    c = sel.mean(0)
    n = np.linalg.svd(sel - c, full_matrices=False)[2][2]
    return n * np.sign(n[2])


# top-down occupancy map of xy points, GRID cells starting at lo
def top_map(xy, lo, shape):
    ij = np.floor((xy - lo) / GRID).astype(int)
    keep = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    M = np.zeros(shape, bool)
    M[ij[keep, 0], ij[keep, 1]] = True
    return M


# robust point-to-plane ICP, match distance shrinking from 30 to 5 cm
def icp_coarse_to_fine(src, tgt, T_init, max_iteration):
    T = T_init
    for dmax in (0.30, 0.15, 0.08, 0.05):
        T = reg.registration_icp(src, tgt, dmax, T,
                                 reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=dmax)),
                                 reg.ICPConvergenceCriteria(max_iteration=max_iteration)).transformation
    return T


# point-to-point ICP with free scale on close pairs; the scale is cbrt(det) of the result
def free_scale_fit(src, tgt, T_init):
    return reg.registration_icp(src, tgt, 0.05, T_init,
                                reg.TransformationEstimationPointToPoint(with_scaling=True),
                                reg.ICPConvergenceCriteria(max_iteration=60)).transformation


# confident MPS semi-dense points, N x 3
def load_mps_points(slam):
    pts = pd.read_csv(os.path.join(slam, 'semidense_points.csv.gz'),
                      usecols=['px_world', 'py_world', 'pz_world', 'inv_dist_std', 'dist_std'])
    pts = pts[(pts.inv_dist_std < INV_DIST_STD) & (pts.dist_std < DIST_STD)]
    return pts[['px_world', 'py_world', 'pz_world']].to_numpy()


# glasses path, every 100th pose
def load_path(slam):
    tr = pd.read_csv(os.path.join(slam, 'closed_loop_trajectory.csv'),
                     usecols=['tx_world_device', 'ty_world_device', 'tz_world_device'])
    return tr.to_numpy()[::100]


# scan levelled on its floor (floor at z = 0), with the levelling transform and scan noise
def level_scan(path):
    raw = o3d.io.read_point_cloud(path)
    pc = raw.voxel_down_sample(0.01)

    # horizontal planes in a coarse copy; the floor is the lowest big one
    rest, horiz = pc.voxel_down_sample(0.03), []
    for _ in range(6):
        model, inl = rest.segment_plane(0.02, 3, 3000)
        n = np.array(model[:3])
        n /= np.linalg.norm(n)
        if abs(n[2]) > 0.9:
            n = n * np.sign(n[2])
            P = np.asarray(rest.select_by_index(inl).points)
            horiz.append((float((P @ n).mean()), n, len(inl)))
        rest = rest.select_by_index(inl, invert=True)
    big = [h for h in horiz if h[2] > 0.25 * max(x[2] for x in horiz)]
    n_floor = min(big, key=lambda h: h[0])[1]

    R_level = rot_a_to_b(n_floor, UP)
    pc.rotate(R_level, center=(0, 0, 0))

    # floor height: densest of 60 bins in the lowest 30 cm
    z = np.asarray(pc.points)[:, 2]
    z_low = np.percentile(z, 1)
    hist, edges = np.histogram(z[z < z_low + 0.3], bins=60)
    z_floor = edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2
    pc.translate((0, 0, -z_floor))
    T_level = rigid(t=np.array([0, 0, -z_floor])) @ rigid(R_level)

    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
    rms_floor = plane_rms(pc, horizontal=True)[2]
    rms_wall = plane_rms(pc, horizontal=False)[2]

    stats = {
        'scan_points': len(raw.points),
        'scan_up_error_deg': float(np.degrees(np.arccos(min(1.0, n_floor[2])))),
        'scan_floor_plane_rms_cm': round(100 * rms_floor, 2),
        'scan_wall_plane_rms_cm': round(100 * rms_wall, 2),
    }
    return pc, T_level, max(rms_floor, rms_wall), stats


# MPS floor height: densest 2 cm bin 1.2-2.2 m below the median glasses height
def mps_floor_height(M, path, eye):
    zt = np.median(path[:, 2])
    zz = M[(M[:, 2] > zt - 2.2) & (M[:, 2] < zt - 1.2), 2]
    if len(zz) <= 200:
        return zt - (eye or 1.6683)
    h, e = np.histogram(zz, bins=np.arange(zt - 2.2, zt - 1.2, 0.02))
    return e[np.argmax(h)] + 0.01


# 1 degree heading sweep: top-down maps of scan and MPS walls/furniture, cross-correlated
def rough_fit(S, M, mps_floor):
    Sb = S[(S[:, 2] > BAND[0]) & (S[:, 2] < BAND[1])]
    Mb = M[(M[:, 2] > mps_floor + BAND[0]) & (M[:, 2] < mps_floor + BAND[1])]

    # MPS map padded by the scan's diagonal, so any turned scan map fits inside it
    pad = 1.0 + float(np.linalg.norm(np.ptp(Sb[:, :2], axis=0))) + 2 * GRID
    lo = Mb[:, :2].min(0) - pad
    shape = tuple((np.ceil((Mb[:, :2].max(0) + pad - lo) / GRID)).astype(int))
    Mmap = binary_dilation(top_map(Mb[:, :2], lo, shape), iterations=1).astype(np.float32)

    centre = Sb[:, :2].mean(0)
    sc = Sb[:, :2] - centre
    cands = []
    for deg in range(360):
        q = sc @ rot_z(deg)[:2, :2].T
        qlo = q.min(0)
        kshape = tuple((np.ceil((q.max(0) - qlo) / GRID) + 1).astype(int))
        K = top_map(q, qlo, kshape).astype(np.float32)
        C = fftconvolve(Mmap, K[::-1, ::-1], mode='valid')
        i, j = np.unravel_index(np.argmax(C), C.shape)
        shift = lo + np.array([i, j]) * GRID - qlo          # where the scan map's corner lands
        cands.append((C[i, j] / K.sum(), deg, shift))

    cands.sort(key=lambda c: -c[0])
    return cands, centre


# ICP of the MPS points onto the scan from the best 4 distinct rough fits; best fitness wins
def fine_fit(tgt, S, M, cands, centre, mps_floor):
    best, seen = None, set()
    for _, deg, shift in cands:
        if len(seen) >= 4:
            break
        key = (round(deg / 10), tuple(np.round(shift, 0)))
        if key in seen:
            continue
        seen.add(key)

        # scan -> MPS: turn about the scan centre, shift, scan floor 0 -> MPS floor
        Rz = rot_z(deg)
        T_m2s = np.linalg.inv(rigid(Rz, np.r_[shift, mps_floor] - Rz @ np.r_[centre, 0]))

        # MPS points that land near the scan
        Ms = (T_m2s[:3, :3] @ M.T).T + T_m2s[:3, 3]
        keep = np.all((Ms >= S.min(0) - 0.3) & (Ms <= S.max(0) + 0.3), axis=1)
        if keep.sum() < 200:
            continue

        src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(M[keep]))
        T_m2s = icp_coarse_to_fine(src, tgt, T_m2s, max_iteration=60)
        fitness = reg.evaluate_registration(src, tgt, 0.05, T_m2s).fitness
        if best is None or fitness > best['fitness']:
            best = dict(deg=deg, fitness=fitness, T_m2s=T_m2s, keep=keep)

    if best is None:
        raise RuntimeError('rough fit found no overlap between the scan and the MPS points')
    return best


# distance of each aligned MPS point to the plane of its nearest scan point
def plane_distances(P, tgt):
    TP, TN = np.asarray(tgt.points), np.asarray(tgt.normals)
    idx = cKDTree(TP).query(P, k=1)[1]
    return np.abs(np.einsum('ij,ij->i', P - TP[idx], TN[idx]))


# ICP fitness with the aligned scan turned 0/90/180/270 degrees; only 0 should fit well
def quarter_turn_fitness(S_m, M):
    scan = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(S_m)).voxel_down_sample(0.02)
    scan.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.06, max_nn=30))
    mps = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(M)).voxel_down_sample(0.02)
    c = S_m.mean(0)

    fits = {}
    for deg in (0, 90, 180, 270):
        Rq = rot_z(deg)
        turned = o3d.geometry.PointCloud(scan)
        turned.transform(rigid(Rq, c - Rq @ c))
        T = icp_coarse_to_fine(mps, turned, np.eye(4), max_iteration=30)
        fits[f'+{deg}deg'] = round(reg.evaluate_registration(mps, turned, 0.05, T).fitness, 3)
    return fits


# top and side views of the aligned scan over the MPS points
def save_overlay(png, scan, mps, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 2, figsize=(13, 6))
    ax[0].scatter(scan[:, 0], scan[:, 1], s=0.2, c='tab:blue', alpha=0.3, label='iPad scan (aligned)')
    ax[0].scatter(mps[:, 0], mps[:, 1], s=0.6, c='tab:red', alpha=0.6, label='MPS points')
    ax[0].plot(path[:, 0], path[:, 1], 'k-', lw=0.6, label='glasses path')
    ax[0].set_aspect('equal')
    ax[0].legend(markerscale=10, loc='upper right')
    ax[0].set_title('Top view')

    ax[1].scatter(scan[:, 0], scan[:, 2], s=0.2, c='tab:blue', alpha=0.3)
    ax[1].scatter(mps[:, 0], mps[:, 2], s=0.6, c='tab:red', alpha=0.6)
    ax[1].set_aspect('equal')
    ax[1].set_title('Side view (x, z)')

    lo, hi = scan.min(0) - 0.5, scan.max(0) + 0.5
    ax[0].set_xlim(lo[0], hi[0])
    ax[0].set_ylim(lo[1], hi[1])
    ax[1].set_xlim(lo[0], hi[0])
    ax[1].set_ylim(lo[2], hi[2])
    fig.tight_layout()
    fig.savefig(png, dpi=110)


# alignment steps 1-4, checks, and report.json with the scan in the MPS frame
def main():
    ap = argparse.ArgumentParser(description='iPad scan aligned to the Aria MPS world frame')
    ap.add_argument('--ipad', required=True, help='RTAB-Map point cloud (.ply)')
    ap.add_argument('--slam', required=True, help='MPS slam/ folder')
    ap.add_argument('--out', required=True, help='output folder')
    ap.add_argument('--eye', type=float, default=None,
                    help='eye height (m); only a fallback for the MPS floor')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    o3d.utility.random.seed(0)
    np.random.seed(0)

    # 1. scan levelled on its floor
    pc, T_level, sigma_scan, rep = level_scan(a.ipad)
    S = np.asarray(pc.points)

    # MPS points, glasses path and MPS floor
    M = load_mps_points(a.slam)
    path = load_path(a.slam)
    mps_floor = mps_floor_height(M, path, a.eye)
    rep['mps_points_confident'] = len(M)
    rep['mps_floor_guess_z'] = round(float(mps_floor), 3)

    # 2. rough fit
    cands, centre = rough_fit(S, M, mps_floor)
    rep['rough_fit_best_scores'] = [[round(float(c[0]), 3), c[1]] for c in cands[:5]]

    # 3. fine fit, and the residual of each MPS point
    tgt = pc.voxel_down_sample(0.02)
    tgt.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.06, max_nn=30))
    best = fine_fit(tgt, S, M, cands, centre, mps_floor)
    T_m2s = best['T_m2s']
    T_s2m = np.linalg.inv(T_m2s)

    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(M[best['keep']]))
    aligned = o3d.geometry.PointCloud(src)
    aligned.transform(T_m2s)
    d = plane_distances(np.asarray(aligned.points), tgt)
    inl = d < 0.05
    sigma_align = float(np.sqrt(np.mean(d[inl] ** 2)))

    rep['icp_candidate_turn_deg'] = best['deg']
    rep['icp_mps_points_used'] = int(best['keep'].sum())
    rep['icp_inlier_fraction_5cm'] = round(float(inl.mean()), 3)
    rep['icp_inlier_rms_cm'] = round(100 * sigma_align, 2)
    rep['icp_inlier_median_cm'] = round(100 * float(np.median(d[inl])), 2)

    # 4. floor tilt: aligned scan floor vs MPS up, and vs the MPS points' own floor
    S_m = (T_s2m[:3, :3] @ S.T).T + T_s2m[:3, 3]
    zf = np.percentile(S_m[:, 2], 2)
    n_scan = floor_normal(S_m[np.abs(S_m[:, 2] - zf) < 0.06])
    rep['tilt_scan_floor_vs_mps_up_deg'] = angle_deg(n_scan, UP)
    Mf = M[np.abs(M[:, 2] - zf) < 0.06]
    if len(Mf) > 500:
        n_mps = floor_normal(Mf)
        rep['tilt_mps_own_floor_vs_mps_up_deg'] = angle_deg(n_mps, UP)
        rep['angle_scan_floor_vs_mps_floor_deg'] = angle_deg(n_scan, n_mps)
        rep['mps_floor_points'] = int(len(Mf))

    # uniqueness of the heading, and scale
    rep['uniqueness_fitness_by_quarter_turn'] = quarter_turn_fitness(S_m, M)
    scale = float(np.cbrt(np.linalg.det(free_scale_fit(src, tgt, T_m2s)[:3, :3])))
    rep['scale_scan_over_mps'] = round(scale, 4)

    # error budget: scan plane noise and ICP residual; trust thresholds >= 2 sigma
    sigma_gt = float(np.hypot(sigma_scan, sigma_align))
    rep['sigma_scan_cm (plane noise only, single scan)'] = round(100 * sigma_scan, 2)
    rep['sigma_align_cm (ICP inlier RMS, includes MPS point noise)'] = round(100 * sigma_align, 2)
    rep['sigma_gt_cm'] = round(100 * sigma_gt, 2)
    rep['smallest_threshold_to_report_cm'] = round(200 * sigma_gt, 1)

    # scan in the MPS frame (1 cm), and the full raw-scan -> MPS transform
    rep['T_scan_raw_to_mps'] = np.round(T_s2m @ T_level, 6).tolist()
    scan_mps = o3d.geometry.PointCloud(pc)
    scan_mps.transform(T_s2m)
    o3d.io.write_point_cloud(os.path.join(a.out, 'scan_in_mps_frame.ply'), scan_mps)

    try:
        save_overlay(os.path.join(a.out, 'alignment_overlay.png'),
                     np.asarray(scan_mps.points)[::20], M[best['keep']], path)
    except Exception as e:
        rep['overlay_error'] = str(e)

    # inputs, so score_cropped.py only needs this report
    rep['scan_path'] = os.path.abspath(a.ipad)
    rep['slam_path'] = os.path.abspath(a.slam)

    with open(os.path.join(a.out, 'report.json'), 'w') as f:
        json.dump(rep, f, indent=2)
    for k, v in rep.items():
        if k != 'T_scan_raw_to_mps':
            print(f'{k}: {v}')


if __name__ == '__main__':
    main()
