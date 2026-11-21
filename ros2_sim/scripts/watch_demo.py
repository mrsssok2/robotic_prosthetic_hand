#!/usr/bin/env python3
"""Print palm position (reach_x, reach_y, lift), the object's position, the chosen grip and the grasp phase once per second
(run in the container next to `fake_udp.py --demo cup`):  python3 scripts/watch_demo.py cup [seconds]"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

obj = sys.argv[1] if len(sys.argv) > 1 else "cup"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 22
rclpy.init()
n = Node("watch_demo")
st = {"j": {}, "obj": None, "grip": "?", "phase": "?"}
n.create_subscription(JointState, "/joint_states", lambda m: st["j"].update(zip(m.name, m.position)), 10)
n.create_subscription(String, "/hand/selected_grip", lambda m: st.update(grip=m.data), 10)
n.create_subscription(String, "/hand/grip_state", lambda m: st.update(phase=m.data), 10)


def on_obj(m):
    for tf in m.transforms:
        if tf.child_frame_id == obj:
            t = tf.transform.translation
            st["obj"] = (t.x, t.y, t.z)


n.create_subscription(TFMessage, f"/object/{obj}/pose", on_obj, 10)
t0, last, z0, zmax = time.time(), 0, None, -1
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.05)
    if st["obj"]:
        z0 = z0 if z0 is not None else st["obj"][2]
        zmax = max(zmax, st["obj"][2])
    if time.time() - last >= 1.0 and st["j"]:
        last = time.time()
        j = st["j"]
        o = "-" if not st["obj"] else "(%.2f %.2f %.3f)" % st["obj"]
        print(f"t={last - t0:4.1f}s palm x={j.get('reach_x', 0):+.3f} y={j.get('reach_y', 0):+.3f} lift={j.get('lift', 0):+.3f}  "
              f"{obj}={o}  grip={st['grip']:7s} phase={st['phase']}", flush=True)
print(f"{obj} start z {z0}, highest z {zmax:.3f}, rise {100 * (zmax - (z0 or 0)):.1f} cm")
