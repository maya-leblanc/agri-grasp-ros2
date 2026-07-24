import numpy as np, open3d as o3d, time, threading, itertools
from scipy.spatial.transform import Rotation as R
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from move import set_pose
from gz_grab import grab

WORLD="agri_grasp_world"; GRIP="robotiq_gripper"; MODEL="robotiq_gripper"
TOPIC="/depth_camera/points"
ORANGE=np.array([-0.26,0.14,0.77])       # <-- change ONLY this for other fruits
RING_R=0.30; CROP=0.10

R_bc=R.from_euler('xyz',[-1.5708,-1.5708,0]).as_matrix()
t_bc=np.array([0.0,-0.165,0.04])

# SAFE viewpoints: all ABOVE the orange, never dipping toward the branch
VIEW_ANGLES=[(0,30),(72,30),(144,30),(216,30),(288,30),(20,65)]

def read_model_pose(world,name,timeout=5.0):
    node=Node(); box={"m":None}; lk=threading.Lock()
    def cb(m):
        with lk: box["m"]=m
    node.subscribe(Pose_V,f"/world/{world}/pose/info",cb)
    t0=time.time()
    while time.time()-t0<timeout:
        with lk:
            if box["m"] is not None: msg=box["m"]; break
        time.sleep(0.02)
    else: raise TimeoutError("no pose/info")
    for p in msg.pose:
        if p.name==name:
            t=np.array([p.position.x,p.position.y,p.position.z])
            q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
            return t,R.from_quat(q).as_matrix()
    raise KeyError("names: "+", ".join(sorted({p.name for p in msg.pose})))

def aim(cam_pos,target):
    f=target-cam_pos; f/=np.linalg.norm(f); up=np.array([0,0,1.0])
    if abs(np.dot(up,f))>0.95: up=np.array([0,1.0,0])
    y=np.cross(up,f); y/=np.linalg.norm(y); z=np.cross(f,y)
    R_wc=np.column_stack([f,y,z]); R_wb=R_wc@R_bc.T
    return cam_pos-R_wb@t_bc, R.from_matrix(R_wb).as_quat()

def rots24():
    out=[]
    for perm in itertools.permutations(range(3)):
        for s in itertools.product([1,-1],repeat=3):
            M=np.zeros((3,3))
            for i,p in enumerate(perm): M[i,p]=s[i]
            if abs(np.linalg.det(M)-1)<1e-6: out.append(M)
    return out

views=[]
for az,el in VIEW_ANGLES:
    a,e=np.radians(az),np.radians(el)
    cam=ORANGE+RING_R*np.array([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
    gp,q=aim(cam,ORANGE); set_pose(WORLD,GRIP,gp,q); time.sleep(1.2)
    xyz=grab(TOPIC,timeout=15.0)
    mt,mR=read_model_pose(WORLD,MODEL)
    camT=mt+mR@t_bc; camR=mR@R_bc
    print(f"az={az:3d} el={el:3d}: {len(xyz)} pts")
    if len(xyz)>0: views.append((xyz.astype(float),camT,camR))

def transformed(K,lowres=True):
    f=o3d.geometry.PointCloud()
    for xyz,camT,camR in views:
        p=xyz[::20] if lowres else xyz
        w=(camR@K@p.T).T+camT
        pc=o3d.geometry.PointCloud(); pc.points=o3d.utility.Vector3dVector(w); f+=pc
    return f
def tightness(pc):
    d=np.linalg.norm(np.asarray(pc.points)-ORANGE,axis=1); near=d[d<0.07]
    return np.std(near) if len(near)>500 else 1e9

best=None
for K in rots24():
    s=tightness(transformed(K))
    if best is None or s<best[1]: best=(K,s)
K=best[0]; print("calibration score (lower=better):",round(best[1],4))

fused=transformed(K,lowres=False)
d=np.linalg.norm(np.asarray(fused.points)-ORANGE,axis=1)
obj=fused.select_by_index(np.where(d<CROP)[0]).voxel_down_sample(0.002)
obj,_=obj.remove_statistical_outlier(20,2.0)
obj.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01,max_nn=30))
obj.orient_normals_towards_camera_location(ORANGE)
obj.normals=o3d.utility.Vector3dVector(-np.asarray(obj.normals))
o3d.io.write_point_cloud("orange_fused.ply",obj)
print("final points:",len(obj.points))
o3d.visualization.draw_geometries([obj])
