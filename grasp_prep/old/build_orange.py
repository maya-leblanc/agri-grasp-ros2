# build_orange.py
import numpy as np, open3d as o3d
from scipy.spatial.transform import Rotation as R
from gz_grab import grab

TOPIC = "/depth_camera/points"          # <-- your topic from `gz topic -l`
ORANGE_CENTER = np.array([-0.2600, 0.1384, 0.7665])   # <-- orange world position from sim
ORANGE_RADIUS = 0.06                        # crop radius in meters

def T_from(pos, quat_xyzw):
    T = np.eye(4)
    T[:3,:3] = R.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = pos
    return T

# camera LINK world pose at each capture (position, quaternion) — you know these from sim
CAM_POSES = [
    (np.array([ 0.3, 0.0, 0.1]), np.array([0,0,0,1])),   # <-- fill per viewpoint
    # (np.array([-0.3, 0.0, 0.1]), quat_pointing_back),
    # (np.array([ 0.0, 0.3, 0.1]), quat_pointing_side),
]

fused = o3d.geometry.PointCloud()
for pos, quat in CAM_POSES:
    input(f"Move gripper to camera pose {pos}, then press Enter...")  # or automate
    xyz = grab(TOPIC)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pc.transform(T_from(pos, quat))     # sensor frame -> world
    fused += pc

# crop = segmentation + "it's the orange", using its known location
d = np.linalg.norm(np.asarray(fused.points) - ORANGE_CENTER, axis=1)
orange = fused.select_by_index(np.where(d < ORANGE_RADIUS)[0])

orange = orange.voxel_down_sample(0.002)                       # 2mm
orange, _ = orange.remove_statistical_outlier(20, 2.0)
orange.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
orange.orient_normals_towards_camera_location(ORANGE_CENTER)
orange.normals = o3d.utility.Vector3dVector(-np.asarray(orange.normals))  # outward

o3d.io.write_point_cloud("orange_fused.ply", orange)
print("points:", len(orange.points))
