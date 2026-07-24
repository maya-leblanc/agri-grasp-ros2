# gz_grab.py
import time, threading, numpy as np
from gz.transport13 import Node
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked

_latest = {"msg": None}; _lock = threading.Lock()

def _cb(msg): 
    with _lock: _latest["msg"] = msg

def _unpack(msg):
    offs = {f.name: f.offset for f in msg.field}
    step = msg.point_step
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
    def f32(name):
        return raw[:, offs[name]:offs[name]+4].copy().view(np.float32).reshape(-1)
    xyz = np.stack([f32("x"), f32("y"), f32("z")], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]   # drop inf/NaN background

def grab(topic, timeout=5.0):
    node = Node()
    node.subscribe(PointCloudPacked, topic, _cb)
    t0 = time.time()
    while time.time() - t0 < timeout:
        with _lock:
            if _latest["msg"] is not None:
                m = _latest["msg"]; _latest["msg"] = None
                return _unpack(m)
        time.sleep(0.05)
    raise TimeoutError(f"no cloud on {topic}")
