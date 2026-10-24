#!/usr/bin/env python3
"""
Capture frames straight from the drone (wifi_cam), bypassing web_server,
FRAME_HUB, /mjpeg and the frontend. JPEG bytes are saved exactly as the drone
sent them (no decode/re-encode/rotation), as:

  <outdir>/<unix_ns>.jpg   individual frames (same naming record_frames.py uses)
  <outdir>/raw.mjpeg       all frames concatenated
  <outdir>/raw.mkv         raw.mjpeg muxed with `-c copy` (needs ffmpeg), no re-encode

Stop the backend first: the drone streams to one client at a time.

    python3 slam/capture_raw.py slam/recordings/raw_test --seconds 15
"""
import argparse
import os
import shutil
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "backend"))
from protocols.wifi_cam_video_protocol import WifiCamVideoProtocolAdapter  # noqa: E402


def jpeg_info(d: bytes):
    """(width, height, exif_orientation or None) from raw JPEG bytes."""
    w = h = None
    orient = None
    i = 2
    while i + 4 <= len(d):
        if d[i] != 0xFF:
            break
        m = d[i + 1]
        n = struct.unpack(">H", d[i + 2:i + 4])[0]
        seg = d[i + 4:i + 2 + n]
        if m in (0xC0, 0xC1, 0xC2):
            h, w = struct.unpack(">HH", seg[1:5])
            break
        if m == 0xE1 and seg.startswith(b"Exif\x00\x00"):
            t = seg[6:]
            e = "<" if t[:2] == b"II" else ">"
            off = struct.unpack(e + "I", t[4:8])[0]
            cnt = struct.unpack(e + "H", t[off:off + 2])[0]
            for k in range(cnt):
                ent = t[off + 2 + 12 * k:off + 14 + 12 * k]
                if struct.unpack(e + "H", ent[:2])[0] == 0x0112:
                    orient = struct.unpack(e + "H", ent[8:10])[0]
        i += 2 + n
    return w, h, orient


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--ip", default=WifiCamVideoProtocolAdapter.DEFAULT_DRONE_IP)
    ap.add_argument("--rotate", type=int, default=-90, help="degrees CW (default -90 = 90 CCW upright; 0 = untouched)")
    a = ap.parse_args()

    os.makedirs(a.outdir, exist_ok=True)
    ad = WifiCamVideoProtocolAdapter(a.ip, rotate_degrees=a.rotate)
    ad.start()
    n = 0
    sizes = {}
    t0 = time.monotonic()
    with open(os.path.join(a.outdir, "raw.mjpeg"), "wb") as mj:
        try:
            while time.monotonic() - t0 < a.seconds:
                fr = ad.get_frame(timeout=1.0)
                if fr is None:
                    continue
                data = bytes(fr.data)
                with open(os.path.join(a.outdir, f"{time.time_ns()}.jpg"), "wb") as f:
                    f.write(data)
                mj.write(data)
                n += 1
                key = jpeg_info(data)
                sizes[key] = sizes.get(key, 0) + 1
        except KeyboardInterrupt:
            pass
    ad.stop()
    dur = time.monotonic() - t0
    fps = n / max(dur, 1e-9)
    print(f"{n} frames in {dur:.1f}s ({fps:.1f} fps)")
    for (w, h, o), c in sizes.items():
        print(f"  {c} frames: {w}x{h} (w x h), EXIF orientation: {o}")
    if n and shutil.which("ffmpeg"):
        out = os.path.join(a.outdir, "raw.mkv")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "mjpeg", "-framerate", f"{max(fps, 1):.2f}",
                        "-i", os.path.join(a.outdir, "raw.mjpeg"), "-c", "copy", out], check=False)
        print("video:", out)
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
