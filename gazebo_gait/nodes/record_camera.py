#!/usr/bin/env python3
"""
Record N frames from /camera/image into output/<name>.mp4. Manual rgb8 decode.
Run: ~/venvs/gait/bin/python nodes/record_camera.py [n_frames] [name] [fps]
"""
import os
import sys

import numpy as np
import imageio.v2 as imageio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output")


class Recorder(Node):
    def __init__(self, n, name, fps):
        super().__init__("record_camera")
        self.n, self.name, self.fps = n, name, fps
        self.frames = []
        self.sub = self.create_subscription(Image, "/camera/image", self.cb, 30)
        self.get_logger().info(f"recording {n} frames -> {name}.mp4")

    def cb(self, msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        try:
            img = arr.reshape(msg.height, msg.width, -1)[:, :, :3]
        except ValueError:
            return
        self.frames.append(np.ascontiguousarray(img))
        if len(self.frames) >= self.n:
            path = os.path.join(OUT, f"{self.name}.mp4")
            imageio.mimsave(path, self.frames, fps=self.fps, macro_block_size=None)
            self.get_logger().info(f"wrote {path} ({len(self.frames)} frames @ {self.fps}fps)")
            raise SystemExit


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 120
    name = sys.argv[2] if len(sys.argv) > 2 else "walk"
    fps = int(sys.argv[3]) if len(sys.argv) > 3 else 30
    rclpy.init()
    node = Recorder(n, name, fps)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
