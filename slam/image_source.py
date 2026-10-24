#!/usr/bin/env python3
"""
ROS 2 image source for monocular SLAM. Publishes /camera/image_raw (mono8 or
bgr8) and /camera/camera_info from either:

  --url  http://127.0.0.1:8000/mjpeg   live stream from the turbodrone backend
  --dir  recordings/run1               frames saved by record_frames.py,
                                       replayed at their original timing

Swapping live <-> recorded changes nothing downstream.
"""
import argparse
import glob
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

SOI, EOI = b"\xff\xd8", b"\xff\xd9"


def live_frames(url):
    import requests

    while True:
        try:
            with requests.get(url, stream=True, timeout=10) as r:
                r.raise_for_status()
                buf = b""
                for chunk in r.iter_content(chunk_size=4096):
                    buf += chunk
                    while True:
                        a = buf.find(SOI)
                        if a < 0:
                            buf = b""
                            break
                        b = buf.find(EOI, a + 2)
                        if b < 0:
                            buf = buf[a:]
                            break
                        yield time.time_ns(), buf[a:b + 2]
                        buf = buf[b + 2:]
        except Exception:
            time.sleep(0.5)


def recorded_frames(d, rate, skip=0):
    files = sorted(glob.glob(os.path.join(d, "*.jpg")))[skip:]
    if not files:
        raise SystemExit(f"no .jpg frames in {d}")
    t0 = int(os.path.basename(files[0])[:-4])
    wall0 = time.monotonic()
    for f in files:
        ts = int(os.path.basename(f)[:-4])
        delay = (ts - t0) / 1e9 / rate - (time.monotonic() - wall0)
        if delay > 0:
            time.sleep(delay)
        with open(f, "rb") as fh:
            yield time.time_ns(), fh.read()


class ImageSource(Node):
    def __init__(self, args):
        super().__init__("image_source")
        self.pub = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.gray = args.gray
        self.frame_id = "camera"
        self.info = None
        self.calib = args.calib

    def _camera_info(self, w, h):
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width, msg.height = w, h
        if self.calib:
            import yaml

            with open(self.calib) as f:
                c = yaml.safe_load(f)
            msg.k = [float(x) for x in c["K"]]
            msg.d = [float(x) for x in c.get("D", [])]
            msg.distortion_model = "plumb_bob"
            msg.p = [msg.k[0], 0.0, msg.k[2], 0.0, 0.0, msg.k[4], msg.k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def publish(self, ts_ns, jpg):
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE if self.gray else cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = h, w
        msg.encoding = "mono8" if self.gray else "bgr8"
        msg.step = w * (1 if self.gray else 3)
        msg.data = img.tobytes()
        self.pub.publish(msg)
        if self.info is None:
            self.info = self._camera_info(w, h)
        self.info.header.stamp = stamp
        self.info_pub.publish(self.info)


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--url")
    g.add_argument("--dir")
    ap.add_argument("--rate", type=float, default=1.0, help="replay speed multiplier (--dir only)")
    ap.add_argument("--skip", type=int, default=0, help="skip first N recorded frames")
    ap.add_argument("--calib", help="YAML with K (9 floats) and D; optional for CameraInfo")
    ap.add_argument("--gray", action="store_true", help="publish mono8")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = ImageSource(args)
    frames = live_frames(args.url) if args.url else recorded_frames(args.dir, args.rate, args.skip)
    try:
        for ts, jpg in frames:
            if not rclpy.ok():
                break
            node.publish(ts, jpg)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
