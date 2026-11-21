"""HandState -> servo angles (real-driver interface) + 16 joint position commands (simulation).

Three ways the finger targets are produced:
  * tracker (default): curl straight from /hand/state;
  * grip-assist (grip_assist:=true): recognised gestures (Open Hand, Pointing, Pinch, Fist) select a coordinated grip;
    with auto_grip:=true a closing hand (Fist / Pinch) near an object picks the grip from the object (grips.select_grip):
    power for the ball and the cup, pinch for the pen;
  * scripted: publish a grip name (open | point | pinch | power) on /hand/grip, 'tracker' returns control to /hand/state.
Smoothing: each finger curl and each wrist angle goes through a One Euro filter when a sample arrives (smoothing:=one_euro,
default); smoothing:=ema is the original EMA 0.3 + 4 curl/s rate limit, kept for before/after measurements. A rate limiter
(max_rate) follows the filter at the 50 Hz control tick.
In every mode the fingertip contact sensors (/contact/<finger>, simulation only) limit the closing force: a finger
stops closing once its fingertip force reaches `f_hold` N (and backs off above `f_max`).
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32, String
from tf2_msgs.msg import TFMessage

from hand_controller.grips import CLOSING_GESTURES, FINGERS, GESTURE_TO_GRIP, GRIPS, OBJECT_WIDTH, GripPlanner, select_grip
from hand_controller.shaping import JOINT_NAMES, WRIST_LIMITS, OneEuroFilter, Shaper, joint_targets, servo_angles
from hand_interfaces.msg import HandState

try:  # only present with ros_gz (simulation); the node also runs without it
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover
    Contacts = None


# a finger is "blocked" when its commanded joint is this far ahead of the measured one for BLOCK_TIME seconds: it is pressing on
# something with the joint effort limit (the fingertip sensor only sees contacts of the last segment)
BLOCK_JOINTS = ["little_mcp", "index_mcp", "middle_mcp", "ring_mcp", "thumb_swing"]   # in curl order
BLOCK_ERROR = np.array([0.10, 0.10, 0.10, 0.10, 0.25])   # rad
BLOCK_TIME = 0.12


def contacts_force(msg) -> float:
    """Total contact force (N) of a fingertip sensor message: sum of the force magnitudes at its contact points."""
    total = 0.0
    for contact in msg.contacts:
        for w in contact.wrenches:
            f = w.body_1_wrench.force
            total += float(np.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2))
    return total


class ControllerNode(Node):
    def __init__(self):
        super().__init__("hand_controller")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("smoothing", "one_euro")   # one_euro | ema (the original, for comparisons)
        self.declare_parameter("min_cutoff", 0.5)         # One Euro: resting cut-off (Hz), lower = less jitter
        self.declare_parameter("beta", 2.5)               # One Euro: speed coefficient, higher = less lag when moving fast
        self.declare_parameter("d_cutoff", 1.0)
        self.declare_parameter("ema_alpha", 0.3)          # smoothing:=ema only
        self.declare_parameter("max_rate", -1.0)          # curl units per second; -1 = 10 (one_euro) / 4 (ema)
        self.declare_parameter("wrist_max_rate", 6.0)     # rad/s
        self.declare_parameter("grip_assist", False)
        self.declare_parameter("auto_grip", False)        # grip from the nearest object + hand pose (implies grip_assist)
        self.declare_parameter("reach", 0.20)             # m: an object closer than this to the palm selects the grip
        self.declare_parameter("palm_base_z", 0.36)       # grasp world: palm height at lift = 0
        self.declare_parameter("f_hold", 2.0)    # N: a finger stops closing at this fingertip force
        self.declare_parameter("f_max", 5.0)     # N: above this it backs off
        self.declare_parameter("verify_grasp", True)   # auto_grip: check an object grasp, re-try / report 'failed' if it missed
        self.one_euro = self.get_parameter("smoothing").value == "one_euro"
        max_rate = self.get_parameter("max_rate").value
        max_rate = max_rate if max_rate > 0 else (10.0 if self.one_euro else 4.0)
        alpha = 1.0 if self.one_euro else self.get_parameter("ema_alpha").value
        self.shaper = Shaper(5, alpha, max_rate)
        self.wrist_shaper = Shaper(2, 1.0, self.get_parameter("wrist_max_rate").value, lo=WRIST_LIMITS[:, 0], hi=WRIST_LIMITS[:, 1])
        euro = dict(min_cutoff=self.get_parameter("min_cutoff").value, beta=self.get_parameter("beta").value,
                    d_cutoff=self.get_parameter("d_cutoff").value)
        self.curl_filter, self.wrist_filter = OneEuroFilter(5, **euro), OneEuroFilter(2, **euro)
        self.f_hold = self.get_parameter("f_hold").value
        self.planner = GripPlanner(f_hold=self.f_hold, f_max=self.get_parameter("f_max").value)
        self.auto_grip = self.get_parameter("auto_grip").value
        self.assist = self.get_parameter("grip_assist").value or self.auto_grip
        self.reach, self.palm_base_z = self.get_parameter("reach").value, self.get_parameter("palm_base_z").value
        self.target = np.zeros(5)  # curl x5 after the filter; held when tracking is lost
        self.wrist_target = np.zeros(2)
        self.last_rx = None
        self.scripted = False
        self.assist_grip = None
        self.closing = False
        self.lift = 0.0
        self.objects = {}          # name -> xyz (simulation ground truth, stands in for object perception)
        self.force = np.zeros(5)
        self.force_t = np.full(5, -1e9)
        self.joint_pos = {}        # measured joint positions (/joint_states)
        self.last_cmd = None       # joint commands of the previous tick
        self.block_since = np.full(5, np.inf)
        self.blocked = np.zeros(5, bool)
        self.servo_pub = self.create_publisher(Float32MultiArray, "/hand/servo_angles", 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, "/finger_position_controller/commands", 10)
        self.force_pub = self.create_publisher(Float32MultiArray, "/hand/fingertip_force", 10)
        self.phase_pub = self.create_publisher(String, "/hand/grip_state", 10)
        self.contact_pub = self.create_publisher(Float32MultiArray, "/hand/contact", 10)   # 1 = fingertip force or blocked
        self.attempt_pub = self.create_publisher(Int32, "/hand/grip_attempt", 10)
        self.grip_name_pub = self.create_publisher(String, "/hand/selected_grip", 10)
        self.wrist_pub = self.create_publisher(Float32MultiArray, "/hand/wrist_angles", 10)  # degrees: pronation, flexion
        self.create_subscription(HandState, "/hand/state", self.on_state, 10)
        self.create_subscription(String, "/hand/grip", self.on_grip, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        if self.auto_grip:
            for name in OBJECT_WIDTH:
                self.create_subscription(TFMessage, f"/object/{name}/pose", lambda m, n=name: self.on_object(n, m), 10)
        if Contacts is not None:
            for i, name in enumerate(FINGERS):
                self.create_subscription(Contacts, f"/contact/{name}", lambda m, i=i: self.on_contact(i, m), 10)
        self.dt = 1.0 / self.get_parameter("rate").value
        self.create_timer(self.dt, self.tick)

    def on_contact(self, i, msg):
        self.force[i], self.force_t[i] = contacts_force(msg), time.monotonic()

    def fingertip_forces(self):
        fresh = (time.monotonic() - self.force_t) < 0.15   # sensors publish only while touching
        return np.where(fresh, self.force, 0.0)

    def on_grip(self, msg: String):
        name = msg.data.strip().lower()
        if name == "tracker":
            self.scripted = False
        elif name in GRIPS:
            self.scripted = True
            self.planner.command(name, current=self.shaper.value)
        else:
            self.get_logger().warn(f"unknown grip {name!r}; use {sorted(GRIPS)} or 'tracker'")

    def on_joints(self, msg: JointState):
        self.joint_pos = dict(zip(msg.name, msg.position))
        self.lift = self.joint_pos.get("lift", self.lift)

    def update_blocked(self):
        """A finger whose measured joint stays behind its command is pressing on something."""
        if self.last_cmd is None or not all(n in self.joint_pos for n in BLOCK_JOINTS):
            return
        now = time.monotonic()
        err = np.array([self.last_cmd[JOINT_NAMES.index(n)] - self.joint_pos[n] for n in BLOCK_JOINTS])
        lagging = err > BLOCK_ERROR
        self.block_since = np.where(lagging, np.minimum(self.block_since, now), np.inf)
        self.blocked = lagging & (now - self.block_since >= BLOCK_TIME)

    def on_object(self, name, msg: TFMessage):
        for tf in msg.transforms:      # [hand_world -> <name>, <name> -> <name>/link]
            if tf.child_frame_id == name:
                t = tf.transform.translation
                self.objects[name] = np.array([t.x, t.y, t.z])

    def nearest_object(self):
        """(name, distance to the palm in m) of the closest object, or None."""
        if not self.objects:
            return None
        palm = np.array([0.0, 0.0, self.palm_base_z + self.lift])
        name = min(self.objects, key=lambda n: np.linalg.norm(self.objects[n] - palm))
        return name, float(np.linalg.norm(self.objects[name] - palm))

    def on_state(self, msg: HandState):
        if not msg.valid:
            return
        now = time.monotonic()
        dt = 1e9 if self.last_rx is None else now - self.last_rx   # a first sample / after a gap: restart the filter
        self.last_rx = now
        curl, wrist = np.array(msg.curl, float), np.array(msg.wrist, float)
        if self.one_euro:
            curl, wrist = self.curl_filter(curl, dt), self.wrist_filter(wrist, dt)
        self.target, self.wrist_target = curl, wrist
        grip = None
        if self.assist and not self.scripted:
            closing = msg.gesture in CLOSING_GESTURES
            if self.auto_grip and closing and self.closing and self.assist_grip:
                grip = self.assist_grip                      # keep the grip chosen when the hand started closing
            elif self.auto_grip:
                grip = select_grip(msg.gesture, self.nearest_object(), self.reach)
            else:
                grip = GESTURE_TO_GRIP.get(msg.gesture)
            self.closing = closing
        if grip and grip != self.assist_grip:
            near = self.nearest_object() if self.auto_grip else None
            at_object = bool(near and near[1] <= self.reach)
            self.get_logger().info(f"grip {grip} (gesture {msg.gesture}" + (f", nearest {near[0]} at {near[1]:.2f} m)" if near else ")"))
            self.planner.command(grip, current=self.shaper.value, verify=self.get_parameter("verify_grasp").value and at_object)
        self.assist_grip = grip

    def tick(self):
        forces = self.fingertip_forces()
        self.update_blocked()
        phase = "tracker"
        if self.scripted or self.assist_grip:
            target, phase = self.planner.step(self.dt, forces, self.blocked if self.planner.phase in ("closing", "holding") else None)
        else:
            # tracker mode: a finger in contact above f_hold may not close further, but can always open
            target = np.where((forces >= self.f_hold) & (self.target > self.shaper.value), self.shaper.value, self.target)
        s = self.shaper.step(target, self.dt)
        w = self.wrist_shaper.step(np.zeros(2) if self.scripted else self.wrist_target, self.dt)
        self.servo_pub.publish(Float32MultiArray(data=[float(a) for a in servo_angles(s)]))
        self.last_cmd = joint_targets(s, w)
        self.joint_pub.publish(Float64MultiArray(data=[float(q) for q in self.last_cmd]))
        self.contact_pub.publish(Float32MultiArray(data=[float(x) for x in (self.planner.in_contact if self.planner.grip else self.blocked | (forces >= self.f_hold))]))
        self.attempt_pub.publish(Int32(data=int(self.planner.attempt)))
        self.wrist_pub.publish(Float32MultiArray(data=[float(np.degrees(a)) for a in w]))
        self.grip_name_pub.publish(String(data=self.assist_grip or ("scripted" if self.scripted else "tracker")))
        self.force_pub.publish(Float32MultiArray(data=[float(x) for x in forces]))
        self.phase_pub.publish(String(data=phase))


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
