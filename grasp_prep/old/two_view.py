import numpy as np, open3d as o3d, time
from scipy.spatial.transform import Rotation as R
from move import set_pose
from gz_grab import grab

WORLD="agri_grasp_world"; GRIP="robotiq_gripper"; TOPIC="/depth_camera/points"
ORANGE=np.array([-0.26,0.14,0.77])      # <-- change ONLY this for other fruits
R_bc=R.from_euler('xyz',[-1.5708,-1.5708,0]).as_matrix(); t_bc=np.array([0.0,-0.165,0.04])

def aim(cam_pos,target):
    f=target-cam_pos; f/=np.linalg.norm(f); up=np.array([0,0,1.0])
    if abs(np.dot(up,f))>0.95: up=np.array([0,1.0,0])
    y=np.cross(up,f); y/=np.linalg.norm(y); z=np.cross(f,y)
    R_wc=np.column_stack([f,y,z])
    return cam_pos-(R_wc@R_bc.T)@t_bc, R.from_matrix(R_wc@R_bc.T).as_quat(), R_wc

fused=o3d.geometry.PointCloud()
for off in [[0.30,0,0.02], [-0.30,0,0.02]]:          # front and back, 180 apart
    cam=ORANGE+np.array(off); gp,q,R_wc=aim(cam,ORANGE)
    set_pose(WORLD,GRIP,gp,q); time.sleep(1.5)
    xyz=grab(TOPIC,timeout=15.0)
    w=(R_wc@xyz.T).T+cam
    d=np.linalg.norm(w-ORANGE,axis=1); near=w[d<0.09]
    shift=ORANGE-near.mean(0)                          # measured error for THIS view
    w=w+shift                                          # snap this view's center onto the orange
    pc=o3d.geometry.PointCloud(); pc.points=o3d.utility.Vector3dVector(w)
    fused+=pc.select_by_index(np.where(np.linalg.norm(w-ORANGE,axis=1)<0.07)[0])

fused=fused.voxel_down_sample(0.002)
fused,_=fused.remove_statistical_outlier(20,2.0)
fused.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01,max_nn=30))
fused.orient_normals_towards_camera_location(ORANGE)
fused.normals=o3d.utility.Vector3dVector(-np.asarray(fused.normals))
o3d.io.write_point_cloud("orange_two.ply",fused)
print("final points:",len(fused.points))
o3d.visualization.draw_geometries([fused])
