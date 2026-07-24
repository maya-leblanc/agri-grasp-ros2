import numpy as np, open3d as o3d, time
from scipy.spatial.transform import Rotation as R
from move import set_pose
from gz_grab import grab

WORLD="agri_grasp_world"; GRIP="robotiq_gripper"; TOPIC="/depth_camera/points"
ORANGE=np.array([-0.26, 0.14, 0.77])

# gripper sits 30cm to the side of the orange, same height
grip_pos = ORANGE + np.array([0.30, 0.0, 0.0])

# try several whole-gripper rotations; keep the one that sees the most points
best=None
for name, euler in [("none",(0,0,0)), ("yaw90",(0,0,np.pi/2)), ("yaw180",(0,0,np.pi)),
                    ("yaw270",(0,0,-np.pi/2)), ("pitch90",(0,np.pi/2,0)), ("pitch-90",(0,-np.pi/2,0)),
                    ("roll90",(np.pi/2,0,0)), ("roll-90",(-np.pi/2,0,0))]:
    q = R.from_euler('xyz', euler).as_quat()
    set_pose(WORLD, GRIP, grip_pos, q)
    time.sleep(1.2)
    xyz = grab(TOPIC, timeout=20.0)
    n = len(xyz)
    print(f"{name:9s} -> {n} points")
    if best is None or n > best[1]:
        best = (name, n, euler, xyz)

print("\nBEST:", best[0], "with", best[1], "points")
xyz = best[3]
if len(xyz):
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(xyz.astype(float))
    o3d.visualization.draw_geometries([pc])
