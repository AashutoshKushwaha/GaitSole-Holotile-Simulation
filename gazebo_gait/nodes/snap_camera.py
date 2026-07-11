#!/usr/bin/env python3
"""
Save N frames from the bridged /camera/image topic to output/ as PNGs, then exit.
Used to visually verify the walking figure and calibrate joint signs. Decodes
rgb8 manually (no cv_bridge dependency).

Run:  ~/venvs/gait/bin/python nodes/snap_camera.py [n_frames] [interval_s]
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.image as mpimg

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output")


class Snapper(Node):
    def __init__(self, n, interval):
        super().__init__("snap_camera")
        self.n, self.interval = n, interval
        self.saved = 0
        self.last_t = -1e9
        self.sub = self.create_subscription(Image, "/camera/image", self.cb, 10)
        self.get_logger().info(f"waiting for /camera/image, will save {n} frame(s)")

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        if t - self.last_t < self.interval:
            return
        self.last_t = t
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        try:
            img = arr.reshape(msg.height, msg.width, -1)[:, :, :3]
        except ValueError:
            self.get_logger().warn(f"unexpected image size {len(arr)} "
                                   f"for {msg.width}x{msg.height} ({msg.encoding})")
            return
        path = os.path.join(OUT, f"cam_{self.saved:02d}.png")
        mpimg.imsave(path, img)
        self.get_logger().info(f"saved {path} ({msg.width}x{msg.height} {msg.encoding})")
        self.saved += 1
        if self.saved >= self.n:
            raise SystemExit


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    rclpy.init()
    node = Snapper(n, interval)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
