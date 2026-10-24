#!/usr/bin/env python3
"""
Record the backend's /mjpeg stream to disk so SLAM can be run offline
(drone powered off). Each frame is saved as <outdir>/<unix_ns>.jpg; the
filename is the receive timestamp and is used for real-time replay.

Usage:
    python slam/record_frames.py recordings/run1 [--url http://127.0.0.1:8000/mjpeg] [--seconds 120]
"""
import argparse
import os
import sys
import time

import requests

SOI, EOI = b"\xff\xd8", b"\xff\xd9"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--url", default="http://127.0.0.1:8000/mjpeg")
    ap.add_argument("--seconds", type=float, default=0, help="0 = until Ctrl-C")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    start = time.monotonic()
    n = 0
    buf = b""
    try:
        with requests.get(args.url, stream=True, timeout=10) as r:
            r.raise_for_status()
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
                    jpg, buf = buf[a:b + 2], buf[b + 2:]
                    with open(os.path.join(args.outdir, f"{time.time_ns()}.jpg"), "wb") as f:
                        f.write(jpg)
                    n += 1
                    if n % 100 == 0:
                        print(f"{n} frames", file=sys.stderr)
                if args.seconds and time.monotonic() - start >= args.seconds:
                    break
    except KeyboardInterrupt:
        pass
    dur = time.monotonic() - start
    print(f"saved {n} frames in {dur:.1f}s ({n / max(dur, 1e-9):.1f} fps) -> {args.outdir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
