"""HandState -> servo angles (real-driver interface) + 16 joint position commands (simulation)."""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray

from hand_controller.shaping import Shaper, joint_targets, servo_angles
from hand_interfaces.msg import HandState


class ControllerNode(Node):
    def __init__(self):
        super().__init__("hand_controller")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("max_rate", 4.0)  # curl units per second
        self.shaper = Shaper(5, self.get_parameter("ema_alpha").value, self.get_parameter("max_rate").value)
        self.target = np.zeros(5)  # curl x5; held when tracking is lost
        self.servo_pub = self.create_publisher(Float32MultiArray, "/hand/servo_angles", 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, "/finger_position_controller/commands", 10)
        self.create_subscription(HandState, "/hand/state", self.on_state, 10)
        self.dt = 1.0 / self.get_parameter("rate").value
        self.create_timer(self.dt, self.tick)

    def on_state(self, msg: HandState):
        if msg.valid:
            self.target = np.array(msg.curl, float)

    def tick(self):
        s = self.shaper.step(self.target, self.dt)
        self.servo_pub.publish(Float32MultiArray(data=[float(a) for a in servo_angles(s)]))
        self.joint_pub.publish(Float64MultiArray(data=[float(q) for q in joint_targets(s)]))


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
