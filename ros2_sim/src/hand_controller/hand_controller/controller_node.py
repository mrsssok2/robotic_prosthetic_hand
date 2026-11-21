"""HandState -> servo angles (real-driver interface) + 16 joint position commands (simulation).

Three ways the finger targets are produced:
  * tracker (default): curl straight from /hand/state;
  * grip-assist (grip_assist:=true): recognised gestures (Open Hand, Pointing, Pinch, Fist) select a coordinated grip;
  * scripted: publish a grip name (open | point | pinch | power) on /hand/grip, 'tracker' returns control to /hand/state.
In every mode the fingertip contact sensors (/contact/<finger>, simulation only) limit the closing force: a finger
stops closing once its fingertip force reaches `f_hold` N (and backs off above `f_max`).
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from hand_controller.grips import FINGERS, GESTURE_TO_GRIP, GRIPS, GripPlanner
from hand_controller.shaping import Shaper, joint_targets, servo_angles
from hand_interfaces.msg import HandState

try:  # only present with ros_gz (simulation); the node also runs without it
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover
    Contacts = None


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
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("max_rate", 4.0)  # curl units per second
        self.declare_parameter("grip_assist", False)
        self.declare_parameter("f_hold", 2.0)    # N: a finger stops closing at this fingertip force
        self.declare_parameter("f_max", 5.0)     # N: above this it backs off
        self.shaper = Shaper(5, self.get_parameter("ema_alpha").value, self.get_parameter("max_rate").value)
        self.f_hold = self.get_parameter("f_hold").value
        self.planner = GripPlanner(f_hold=self.f_hold, f_max=self.get_parameter("f_max").value)
        self.assist = self.get_parameter("grip_assist").value
        self.target = np.zeros(5)  # curl x5; held when tracking is lost
        self.scripted = False
        self.assist_grip = None
        self.force = np.zeros(5)
        self.force_t = np.full(5, -1e9)
        self.servo_pub = self.create_publisher(Float32MultiArray, "/hand/servo_angles", 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, "/finger_position_controller/commands", 10)
        self.force_pub = self.create_publisher(Float32MultiArray, "/hand/fingertip_force", 10)
        self.phase_pub = self.create_publisher(String, "/hand/grip_state", 10)
        self.create_subscription(HandState, "/hand/state", self.on_state, 10)
        self.create_subscription(String, "/hand/grip", self.on_grip, 10)
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

    def on_state(self, msg: HandState):
        if not msg.valid:
            return
        self.target = np.array(msg.curl, float)
        grip = GESTURE_TO_GRIP.get(msg.gesture) if self.assist and not self.scripted else None
        if grip and grip != self.assist_grip:
            self.planner.command(grip, current=self.shaper.value)
        self.assist_grip = grip

    def tick(self):
        forces = self.fingertip_forces()
        phase = "tracker"
        if self.scripted or self.assist_grip:
            target, phase = self.planner.step(self.dt, forces)
        else:
            # tracker mode: a finger in contact above f_hold may not close further, but can always open
            target = np.where((forces >= self.f_hold) & (self.target > self.shaper.value), self.shaper.value, self.target)
        s = self.shaper.step(target, self.dt)
        self.servo_pub.publish(Float32MultiArray(data=[float(a) for a in servo_angles(s)]))
        self.joint_pub.publish(Float64MultiArray(data=[float(q) for q in joint_targets(s)]))
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
