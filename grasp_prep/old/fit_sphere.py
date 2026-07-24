import numpy as np
import open3d as o3d

def fit_sphere_ransac(points, iterations=1000, threshold=0.005):
    best_center = None
    best_radius = None
    best_inliers = 0

    for _ in range(iterations):
        idx = np.random.choice(len(points), 4, replace=False)
        sample = points[idx]

        A = np.c_[2*sample, np.ones(4)]
        b = (sample**2).sum(axis=1)
        try:
            result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx, cy, cz = result[:3]
            r_sq = result[3] + cx**2 + cy**2 + cz**2
            if r_sq <= 0:
                continue
            r = np.sqrt(r_sq)
        except np.linalg.LinAlgError:
            continue

        center = np.array([cx, cy, cz])
        dists = np.abs(np.linalg.norm(points - center, axis=1) - r)
        inliers = np.sum(dists < threshold)

        if inliers > best_inliers:
            best_inliers = inliers
            best_center = center
            best_radius = r

    return best_center, best_radius, best_inliers

# --- load your point cloud ---
pc = o3d.io.read_point_cloud("orange_single.ply")
pts = np.asarray(pc.points)
print("Loaded points:", len(pts))

center, radius, inliers = fit_sphere_ransac(pts)
diameter = 2 * radius

print(f"Inlier count: {inliers} / {len(pts)}")
print(f"Estimated center: {center}")
print(f"Estimated radius: {radius*1000:.1f} mm")
print(f"Estimated diameter: {diameter*1000:.1f} mm")
