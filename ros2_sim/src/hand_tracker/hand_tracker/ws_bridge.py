"""Browser -> ROS bridge: serves the MediaPipe page and receives landmarks over a WebSocket.

    http://localhost:8765/        the page (webcam + MediaPipe Tasks Vision JS)
    ws://localhost:8765/ws        page -> bridge, JSON: {"hand": "Right", "score": 0.97, "lm": [[x,y,z] x 21]}
                                  or {"hand": null} when no hand is visible (nothing is published)
"""
import asyncio
import json
import os
import threading
from http import HTTPStatus

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from websockets.asyncio.server import serve
from websockets.datastructures import Headers
from websockets.http11 import Response

from hand_interfaces.msg import HandLandmarks


def parse_message(raw):
    """Return (handedness, score, [[x,y,z]*21]) or None if the message is not a usable hand."""
    try:
        data = json.loads(raw)
        if not data.get("hand"):
            return None
        lm = [[float(v) for v in p] for p in data["lm"]]
        if len(lm) != 21 or any(len(p) != 3 for p in lm):
            return None
        return str(data["hand"]), float(data.get("score", 1.0)), lm
    except (ValueError, TypeError, KeyError, AttributeError):
        return None


class WsBridge(Node):
    def __init__(self):
        super().__init__("ws_bridge")
        self.declare_parameter("port", 8765)
        self.declare_parameter("host", "0.0.0.0")
        self.pub = self.create_publisher(HandLandmarks, "/hand/landmarks", 10)
        share = get_package_share_directory("hand_tracker")
        with open(os.path.join(share, "web", "index.html"), "rb") as f:
            self.page = f.read()
        threading.Thread(target=lambda: asyncio.run(self.serve()), daemon=True).start()

    def on_hand(self, handedness, score, lm):
        msg = HandLandmarks()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.handedness, msg.score = handedness, score
        msg.landmarks = [Point(x=x, y=y, z=z) for x, y, z in lm]
        self.pub.publish(msg)

    def process_request(self, connection, request):
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None
        if request.path in ("/", "/index.html"):
            return Response(200, "OK", Headers([("Content-Type", "text/html; charset=utf-8"),
                                                ("Cache-Control", "no-store")]), self.page)
        return connection.respond(HTTPStatus.NOT_FOUND, "not found\n")

    async def handler(self, ws):
        self.get_logger().info("browser connected")
        try:
            async for raw in ws:
                parsed = parse_message(raw)
                if parsed:
                    self.on_hand(*parsed)
        finally:
            self.get_logger().info("browser disconnected")

    async def serve(self):
        host, port = self.get_parameter("host").value, self.get_parameter("port").value
        async with serve(self.handler, host, port, process_request=self.process_request):
            self.get_logger().info(f"open http://localhost:{port}/ in your browser")
            await asyncio.get_running_loop().create_future()


def main():
    rclpy.init()
    node = WsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
