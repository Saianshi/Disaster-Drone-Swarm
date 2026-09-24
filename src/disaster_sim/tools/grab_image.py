#!/usr/bin/env python3
"""Save N frames from an image topic to /tmp/wcam then exit.
Usage: grab_image.py /drone_0/camera/image_raw [count] [prefix]
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

topic = sys.argv[1]
count = int(sys.argv[2]) if len(sys.argv) > 2 else 1
prefix = sys.argv[3] if len(sys.argv) > 3 else "frame"


class Grab(Node):
    def __init__(self):
        super().__init__("grab_image")
        self.n = 0
        self.sub = self.create_subscription(Image, topic, self.cb, 10)

    def cb(self, msg):
        # skip the first few frames: a lazy camera plugin replays a stale
        # last-rendered frame the instant a subscriber connects
        self.n += 1
        if self.n <= 4:
            return
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ("rgb8", "8UC3"):
            a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        k = self.n - 5
        path = f"/tmp/wcam/{prefix}_{k:02d}.jpg"
        cv2.imwrite(path, a)
        self.get_logger().info(f"saved {path} ({msg.width}x{msg.height} {msg.encoding})")
        if k + 1 >= count:
            raise SystemExit


def main():
    rclpy.init()
    g = Grab()
    try:
        rclpy.spin(g)
    except SystemExit:
        pass
    g.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
