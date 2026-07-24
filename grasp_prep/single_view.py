import numpy as np, open3d as o3d, time, json
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from move import set_pose
from gz_grab import grab

# ==================== CONFIG ====================
WORLD="agri_grasp_world"; GRIP="robotiq_gripper"; TOPIC="/depth_camera/points"
FRUIT_NAME  = "orange"
FRUIT_POS   = np.array([-0.26, 0.14, 0.77])   # ROI centre only (stands in for a detector bbox)
VIEW_OFFSET = np.array([0.30, 0.0, 0.02])     # camera position relative to fruit

ROI_RADIUS   = 0.12      # generous ROI; DBSCAN does the real segmentation
DBSCAN_EPS   = 0.010
VOXEL        = 0.0015
GRAZE_DEG    = 70        # discard points seen at more than this angle from the normal
INLIER_TH    = 0.0015    # RANSAC inlier band (m)
R_BOUNDS     = (0.015, 0.070)
NOISE_SIGMA  = 0.0       # 0 = noiseless. See part 3 for realistic values.
# ================================================

R_bc = R.from_euler('xyz', [-1.5708, -1.5708, 0]).as_matrix()
t_bc = np.array([0.0, -0.165, 0.04])

def aim(cam_pos, target):
    f = target - cam_pos; f /= np.linalg.norm(f)
    up = np.array([0,0,1.0])
    if abs(np.dot(up,f)) > 0.95: up = np.array([0,1.0,0])
    y = np.cross(up,f); y /= np.linalg.norm(y); z = np.cross(f,y)
    R_wc = np.column_stack([f,y,z])
    return cam_pos - (R_wc@R_bc.T)@t_bc, R.from_matrix(R_wc@R_bc.T).as_quat(), R_wc

# ---------- sphere fitting ----------
def _algebraic(P):
    A = np.c_[2*P, np.ones(len(P))]; b = (P**2).sum(1)
    sol,*_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:3]
    return c, np.sqrt(max(sol[3] + c@c, 1e-12))

def _geometric(P, c0, r0):
    res = least_squares(lambda x: np.linalg.norm(P-x[:3],axis=1)-x[3],
                        np.r_[c0,r0], loss='soft_l1', f_scale=0.002)
    return res.x[:3], res.x[3]

def fit_sphere(P, iters=500):
    best=(0,None,None); n=len(P)
    for _ in range(iters):
        s = P[np.random.choice(n, 8, replace=False)]
        try: c,r = _algebraic(s)
        except np.linalg.LinAlgError: continue
        if not (R_BOUNDS[0] < r < R_BOUNDS[1]): continue
        k = int(np.sum(np.abs(np.linalg.norm(P-c,axis=1)-r) < INLIER_TH))
        if k > best[0]: best = (k,c,r)
    k,c,r = best
    if c is None: raise RuntimeError("sphere fit failed")
    inl = P[np.abs(np.linalg.norm(P-c,axis=1)-r) < INLIER_TH]
    c,r = _geometric(inl, c, r)                       # refit on ALL inliers
    rms = float(np.sqrt(np.mean((np.linalg.norm(inl-c,axis=1)-r)**2)))
    return c, r, len(inl), rms

# ---------- capture ----------
def capture(view_offset=VIEW_OFFSET, settle=1.5):
    cam = FRUIT_POS + np.asarray(view_offset)
    gp,q,R_wc = aim(cam, FRUIT_POS)
    set_pose(WORLD, GRIP, gp, q); time.sleep(settle)
    xyz = grab(TOPIC, timeout=15.0)

    if NOISE_SIGMA > 0:                               # depth noise along the ray, camera frame
        rng = np.linalg.norm(xyz, axis=1, keepdims=True)
        u = xyz / np.maximum(rng, 1e-9)
        sig = NOISE_SIGMA * (rng**2) / (0.30**2)      # quadratic with range, calibrated at 30cm
        xyz = xyz + u * np.random.normal(0.0, 1.0, sig.shape) * sig

    w = (R_wc @ xyz.T).T + cam                        # camera -> world
    return w, cam

def segment(w, cam):
    w = w[np.linalg.norm(w - FRUIT_POS, axis=1) < ROI_RADIUS]      # ROI
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(w)

    lab = np.array(pc.cluster_dbscan(eps=DBSCAN_EPS, min_points=20))
    if lab.max() < 0: raise RuntimeError("no cluster found")
    sizes = [(np.sum(lab==i), i) for i in range(lab.max()+1)]
    # pick the cluster closest to the ROI centre among the big ones (fruit, not branch)
    big = [i for s,i in sizes if s > 0.15*max(s for s,_ in sizes)]
    pick = min(big, key=lambda i: np.linalg.norm(
        np.asarray(pc.points)[lab==i].mean(0) - FRUIT_POS))
    pc = pc.select_by_index(np.where(lab==pick)[0])

    pc,_ = pc.remove_statistical_outlier(20, 2.0)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    pc.orient_normals_towards_camera_location(cam)                 # outward on visible side

    P = np.asarray(pc.points); N = np.asarray(pc.normals)
    v = cam - P; v /= np.linalg.norm(v,axis=1,keepdims=True)
    keep = np.sum(N*v,axis=1) > np.cos(np.radians(GRAZE_DEG))       # drop grazing points
    pc = pc.select_by_index(np.where(keep)[0])
    return pc.voxel_down_sample(VOXEL)

def run(view_offset=VIEW_OFFSET, save=True):
    w, cam = capture(view_offset)
    pc = segment(w, cam)
    pts = np.asarray(pc.points)
    c, r, n_inl, rms = fit_sphere(pts)
    out = dict(fruit=FRUIT_NAME, diameter_mm=2*r*1000, center=c.tolist(),
               rms_mm=rms*1000, n_points=len(pts),
               inlier_frac=n_inl/len(pts), view_offset=list(map(float,view_offset)))
    if save:
        completed = np.vstack([pts, 2*c - pts])                    # symmetry completion
        pcc = o3d.geometry.PointCloud()
        pcc.points = o3d.utility.Vector3dVector(completed)
        pcc = pcc.voxel_down_sample(VOXEL)
        pcc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01,max_nn=30))
        pcc.orient_normals_towards_camera_location(c)
        pcc.normals = o3d.utility.Vector3dVector(-np.asarray(pcc.normals))
        o3d.io.write_point_cloud(f"{FRUIT_NAME}_partial.ply", pc)
        o3d.io.write_point_cloud(f"{FRUIT_NAME}_completed.ply", pcc)
        np.savetxt(f"{FRUIT_NAME}_partial.csv", np.c_[pts, np.asarray(pc.normals)],
                   delimiter=",", header="x,y,z,nx,ny,nz", comments="")
        np.savetxt(f"{FRUIT_NAME}_completed.csv",
                   np.c_[np.asarray(pcc.points), np.asarray(pcc.normals)],
                   delimiter=",", header="x,y,z,nx,ny,nz", comments="")
        json.dump(out, open(f"{FRUIT_NAME}_fit.json","w"), indent=2)
    return pc, out

if __name__ == "__main__":
    pc, out = run()
    for k,v in out.items(): print(f"{k}: {v}")
    o3d.visualization.draw_geometries([pc])