#!/usr/bin/env python3
# Delaunay mesh simplified for simulation, staying within about 1 cm of the original. Spike triangles go
# first, then quadric error decimation (Garland & Heckbert 1997, via Open3D): flat ground collapses into a
# few large triangles while edges and small objects keep their detail, so the contact surface stays put.
#
# python mesh/photogrammetry/decimate_mesh.py mesh_delaunay.ply mesh_delaunay_decimated.ply

import argparse
import time

import numpy as np
import open3d as o3d

MAX_EDGE_M = 2.0     # longer edges are spikes the mesher draws out to far noise and sky
TOLERANCE_M = 0.01   # about 1 cm: 13% of triangles kept, floor coverage unchanged on a 2.5 M-triangle mesh


# full mesh in, simplified mesh out, with triangle counts printed
def main():
    ap = argparse.ArgumentParser(description="Delaunay mesh simplified for simulation, within about 1 cm",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh_in", help="full mesh, e.g. mesh_delaunay.ply")
    ap.add_argument("mesh_out", help="simplified mesh to write")
    args = ap.parse_args()

    mesh = o3d.io.read_triangle_mesh(args.mesh_in)
    n_in = len(mesh.triangles)
    if n_in == 0:
        raise SystemExit(f"{args.mesh_in} holds no triangles")

    # 1. spikes first; with them in, flat ground would merge into >2 m triangles, later lost as floor holes
    v = np.asarray(mesh.vertices)
    p = v[np.asarray(mesh.triangles)]
    longest = np.max(
        np.stack([np.linalg.norm(p[:, i] - p[:, (i + 1) % 3], axis=1) for i in range(3)], 1), axis=1
    )
    spikes = longest > MAX_EDGE_M
    mesh.remove_triangles_by_mask(spikes)
    mesh.remove_unreferenced_vertices()

    # 2. quadric decimation, bounded by error (squared distance, so approximate) rather than a triangle count
    t0 = time.time()
    simple = mesh.simplify_quadric_decimation(target_number_of_triangles=1, maximum_error=TOLERANCE_M ** 2)
    simple.remove_unreferenced_vertices()
    dt = time.time() - t0

    if not o3d.io.write_triangle_mesh(args.mesh_out, simple, write_ascii=False):
        raise SystemExit(f"failed to write {args.mesh_out}")

    n_out = len(simple.triangles)
    print(f"input     : {n_in:,} triangles")
    print(f"spikes    : {int(spikes.sum()):,} triangles with an edge > {MAX_EDGE_M} m removed")
    print(f"simplified: {n_out:,} triangles ({100.0 * n_out / n_in:.1f}% of input), "
          f"tolerance about {TOLERANCE_M * 100:.0f} cm, {dt:.0f} s")
    print(f"wrote {args.mesh_out}")


if __name__ == "__main__":
    main()
