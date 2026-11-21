#!/usr/bin/env python3
"""Scripted grasp test (run inside the container while `ros2 launch hand_bringup hand.launch.py grasp:=true auto_grip:=true`
is up).

For each object and each trial: park the other objects, drop the target in front of the open hand with a small random
offset, then **close the hand** the way a user would: this script publishes /hand/state with a closing hand (gesture "Fist",
all curls 1) at 30 Hz -- no grip is named. The controller picks the grip from the hand pose and the nearest object (power
for the ball and the cup, pinch for the pen; hand_controller.grips.select_grip) and closes until the fingertip contact
force reaches its threshold. Then lift the hand 10 cm, hold for 1.5 s. Success = the object rose >= 7 cm with the hand AND
stayed within 8 cm of its grasp point relative to the palm. Then open the hand and check that the object drops.

    python3 scripts/grasp_test.py --trials 5 --out docs/results/grasp.json
    python3 scripts/grasp_test.py --objects pen:pinch,ball:power   # name a grip instead (publishes /hand/grip): ablations
"""
import argparse
import json
import random
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32, String
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from hand_interfaces.msg import HandState

# spots found by scanning the table in front of the hand (data: docs/results/README.md); the hand reaches over the table
# palm-down, so the objects sit under the index finger / thumb side
SPOTS = {"ball": (0.05, -0.11, 0.312), "cup": (0.05, -0.11, 0.317), "pen": (0.04, -0.085, 0.35)}
PARK = {"ball": (0.30, -0.20, 0.312), "cup": (0.0, -0.30, 0.317), "pen": (-0.30, -0.20, 0.35)}
EXPECTED = {"ball": "power", "cup": "power", "pen": "pinch"}   # what select_grip should pick
LIFT = 0.10


def gz_set_pose(name, xyz):
    q = "w: 1"
    req = f'name: "{name}" position {{x: {xyz[0]} y: {xyz[1]} z: {xyz[2]}}} orientation {{{q}}}'
    out = subprocess.run(["gz", "service", "-s", "/world/hand_world/set_pose", "--reqtype", "gz.msgs.Pose",
                          "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                         check=True, capture_output=True, text=True).stdout
    if "true" not in out:   # a timed-out call also exits 0 (e.g. GZ_PARTITION not set: use scripts/sim_exec.sh)
        sys.exit(f"gz set_pose {name} failed: {out.strip() or 'no reply'} (is GZ_PARTITION set? use scripts/sim_exec.sh)")


class GraspTest(Node):
    def __init__(self):
        super().__init__("grasp_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.poses, self.force, self.phase = {}, np.zeros(5), "?"
        for obj in SPOTS:  # PosePublisher plugin in the world -> /object/<name>/pose
            self.create_subscription(TFMessage, f"/object/{obj}/pose", lambda m, o=obj: self.on_pose(o, m), 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Float32MultiArray, "/hand/fingertip_force", lambda m: setattr(self, "force", np.array(m.data)), 10)
        self.create_subscription(String, "/hand/grip_state", lambda m: setattr(self, "phase", m.data), 10)
        self.create_subscription(String, "/hand/selected_grip", lambda m: setattr(self, "selected", m.data), 10)
        self.selected = "?"
        self.attempt, self.contact = 0, np.zeros(5)
        self.create_subscription(Int32, "/hand/grip_attempt", lambda m: setattr(self, "attempt", m.data), 10)
        self.create_subscription(Float32MultiArray, "/hand/contact", lambda m: setattr(self, "contact", np.array(m.data)), 10)
        self.state_pub = self.create_publisher(HandState, "/hand/state", 10)
        self.hand = ("Open Hand", [0.0] * 5)     # what the "camera" reports; published at 30 Hz while waiting
        self.wrist = (0.0, 0.0)                  # (pronation, flexion) the "camera" reports
        self._last_pub = 0.0
        self.grip_pub = self.create_publisher(String, "/hand/grip", 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, "/lift_controller/commands", 10)

    def on_pose(self, obj, msg):
        for tf in msg.transforms:   # [hand_world -> <obj>, <obj> -> <obj>/link]
            if tf.child_frame_id == obj:
                t = tf.transform.translation
                self.poses[obj] = np.array([t.x, t.y, t.z])

    def on_joints(self, msg):
        if "lift" in msg.name:  # palm frame origin = base_z (0.36) + lift
            self.poses["palm"] = np.array([0.0, 0.0, 0.36 + msg.position[msg.name.index("lift")]])

    def publish_hand(self):
        if time.monotonic() - self._last_pub < 1 / 30:
            return
        self._last_pub = time.monotonic()
        m = HandState()
        m.valid, m.handedness, m.gesture = True, "Right", self.hand[0]
        m.curl = [float(c) for c in self.hand[1]]
        m.wrist = [float(a) for a in self.wrist]
        self.state_pub.publish(m)

    def wait(self, seconds, until=None):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            self.publish_hand()
            rclpy.spin_once(self, timeout_sec=0.01)
            if until and until():
                return True
        return False

    def set_lift(self, h):
        self.lift_pub.publish(Float64MultiArray(data=[float(h)]))

    def grip(self, name):
        """name: a grip to command directly, 'auto' = close the hand (fist) and let the controller choose, 'open' = release."""
        if name == "auto":
            self.grip_pub.publish(String(data="tracker"))
            self.hand = ("Fist", [1.0] * 5)
        elif name == "open":
            self.grip_pub.publish(String(data="tracker"))
            self.hand = ("Open Hand", [0.0] * 5)
        else:
            self.grip_pub.publish(String(data=name))

    def trial(self, obj, rng, grip=None, spot=None, wrist=(0.0, 0.0)):
        grip = grip or "auto"
        self.wrist = wrist
        for other, p in PARK.items():
            if other != obj:
                gz_set_pose(other, p)
        lift0 = getattr(self, "lift0", 0.0)       # experiments: approach height of the palm (negative = lower)
        self.grip("open"); self.set_lift(lift0)
        self.wait(1.5)
        jitter = (rng.uniform(-0.006, 0.006), rng.uniform(-0.003, 0.003), 0.0)
        spot = np.array(SPOTS[obj] if spot is None else spot) + jitter
        gz_set_pose(obj, spot)
        self.wait(1.0)
        rest = self.poses[obj].copy()
        self.phase, self.selected = "?", "?"
        self.grip(grip)
        self.wait(25.0, until=lambda: self.phase in ("holding", "failed"))    # up to 3 verified attempts
        reached = self.phase == "holding"
        attempts, n_contact = int(self.attempt), int(self.contact.sum())
        fmax = np.zeros(5)
        self.wait(0.5)
        palm0 = self.poses["palm"].copy()
        rel0 = self.poses[obj] - palm0
        fmax = np.maximum(fmax, self.force)
        # lift 10 cm in 2 s, hold 1.5 s
        steps = 40
        for k in range(1, steps + 1):
            self.set_lift(lift0 + LIFT * k / steps)
            self.wait(2.0 / steps)
            fmax = np.maximum(fmax, self.force)
        self.wait(1.5)
        fmax = np.maximum(fmax, self.force)
        z_rise = float(self.poses[obj][2] - rest[2])
        rel = self.poses[obj] - self.poses["palm"]
        slip = float(np.linalg.norm(rel - rel0))
        f_hold_end, chosen = self.force.copy(), self.selected
        ok = bool(z_rise >= 0.07 and slip < 0.08)
        # release: open the hand, the object should fall away from the lifted palm
        self.grip("open")
        self.wait(2.0)
        released = bool(self.poses[obj][2] < rest[2] + 0.03)
        self.set_lift(lift0)
        self.wait(1.0)
        return dict(object=obj, grip=chosen if grip == "auto" else grip, mode=grip, settled=reached, success=ok, z_rise_cm=round(100 * z_rise, 1),
                    slip_cm=round(100 * slip, 1), released=released, attempts=attempts, contacts=n_contact,
                    peak_force_N=[round(float(x), 2) for x in fmax], hold_force_N=[round(float(x), 2) for x in f_hold_end],
                    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--objects", default="pen,cup,ball")
    ap.add_argument("--out", default="grasp.json")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rclpy.init()
    node = GraspTest()
    node.wait(1.0)
    while not all(k in node.poses for k in ("palm", *SPOTS)):
        rclpy.spin_once(node, timeout_sec=0.2)
    rng = random.Random(a.seed)
    results = []
    for spec in a.objects.split(","):
        obj, _, grip = spec.partition(":")
        for i in range(a.trials):
            r = node.trial(obj, rng, grip or None)
            r["trial"] = i
            results.append(r)
            print(json.dumps(r), flush=True)
    node.grip("open")
    with open(a.out, "w") as f:
        json.dump(results, f, indent=1)
    for spec in a.objects.split(","):
        obj, _, grip = spec.partition(":")
        rs = [r for r in results if r["object"] == obj and r["mode"] == (grip or "auto")]
        grips = sorted({r["grip"] for r in rs})
        print(f"{obj:5s} grip={'/'.join(grips):8s} ({'auto' if not grip else 'named'})  success {sum(r['success'] for r in rs)}/{len(rs)}"
              f"  first attempt {sum(r['success'] and r['attempts'] == 1 for r in rs)}/{len(rs)}"
              f"  released {sum(r['released'] for r in rs)}/{len(rs)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
