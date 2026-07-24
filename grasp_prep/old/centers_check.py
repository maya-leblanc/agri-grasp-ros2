import numpy as np, time
from scipy.spatial.transform import Rotation as R
from move import set_pose
from gz_grab import grab

WORLD="agri_grasp_world"; GRIP="robotiq_gripper"; TOPIC="/depth_camera/points"
ORANGE=np.array([-0.26,0.14,0.77])
R_bc=R.from_euler('xyz',[-1.5708,-1.5708,0]).as_matrix(); t_bc=np.array([0.0,-0.165,0.04])

def aim(cam_pos,target):
    f=target-cam_pos; f/=np.linalg.norm(f); up=np.array([0,0,1.0])
    if abs(np.dot(up,f))>0.95: up=np.array([0,1.0,0])
    y=np.cross(up,f); y/=np.linalg.norm(y); z=np.cross(f,y)
    R_wc=np.column_stack([f,y,z]); return cam_pos-(R_wc@R_bc.T)@t_bc, R.from_matrix(R_wc@R_bc.T).as_quat(), R_wc

for label,off in [("front",[0.30,0,0.02]),("back",[-0.30,0,0.02])]:
    cam=ORANGE+np.array(off); gp,q,R_wc=aim(cam,ORANGE); set_pose(WORLD,GRIP,gp,q); time.sleep(1.5)
    xyz=grab(TOPIC,timeout=15.0)
    w=(R_wc@xyz.T).T+cam
    d=np.linalg.norm(w-ORANGE,axis=1); near=w[d<0.07]
    print(label,"center:",near.mean(0).round(3) if len(near) else "none","  count:",len(near))
