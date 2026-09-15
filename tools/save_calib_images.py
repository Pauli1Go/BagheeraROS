"""Save calibration stills from /camera/image_raw to a mounted directory.

Run inside the bagheera-base container (has rclpy + cv2):

    python3 /bagheera_ws/maps/save_calib_images.py \
        --outdir /bagheera_ws/maps/calib --interval 1.0 --max-images 150

Move the robot/board through calibration poses while it runs. Stop with Ctrl-C.
"""

from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Saver(Node):
    def __init__(self, outdir: str, interval: float, max_images: int) -> None:
        super().__init__("calib_image_saver")
        self._outdir = outdir
        self._interval = interval
        self._max_images = max_images
        self._count = 0
        self._last = 0.0
        os.makedirs(outdir, exist_ok=True)
        self._sub = self.create_subscription(Image, "/camera/image_raw", self._cb, 10)
        self.get_logger().info(
            f"saving every {interval:.1f}s to {outdir} (max {max_images})"
        )

    def _cb(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last < self._interval or self._count >= self._max_images:
            return
        if msg.encoding != "bgr8":
            self.get_logger().warn(f"unexpected encoding {msg.encoding}, skipping")
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        path = os.path.join(self._outdir, f"img_{self._count:03d}.png")
        cv2.imwrite(path, frame)
        self._count += 1
        self._last = now
        self.get_logger().info(f"saved {path} ({self._count}/{self._max_images})")
        if self._count >= self._max_images:
            self.get_logger().info("done, shutting down")
            raise SystemExit


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--outdir", default="/bagheera_ws/maps/calib")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--max-images", type=int, default=150)
    args = parser.parse_args()

    rclpy.init()
    node = Saver(args.outdir, args.interval, args.max_images)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        print(f"saved {node._count} images to {args.outdir}")


if __name__ == "__main__":
    main()
