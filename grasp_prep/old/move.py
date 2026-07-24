import subprocess, numpy as np
from scipy.spatial.transform import Rotation as R

def set_pose(world, model, pos, quat_xyzw, timeout_ms=2000):
    x, y, z = pos
    qx, qy, qz, qw = quat_xyzw
    req = (f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
    cmd = ["gz", "service", "-s", f"/world/{world}/set_pose",
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", str(timeout_ms), "--req", req]
    out = subprocess.run(cmd, capture_output=True, text=True)
    if "true" not in out.stdout.lower():
        raise RuntimeError(f"set_pose failed: {out.stdout} {out.stderr}")

def look_at_quat(cam_pos, target, up=(0, 0, 1.0)):
    cam_pos = np.asarray(cam_pos, float); target = np.asarray(target, float); up = np.asarray(up, float)
    f = target - cam_pos; f /= np.linalg.norm(f)
    z = up - f * np.dot(up, f)
    if np.linalg.norm(z) < 1e-6: z = np.array([0., 0., 1.]) - f * f[2]
    z /= np.linalg.norm(z)
    y = np.cross(z, f)
    return R.from_matrix(np.column_stack([f, y, z])).as_quat()
