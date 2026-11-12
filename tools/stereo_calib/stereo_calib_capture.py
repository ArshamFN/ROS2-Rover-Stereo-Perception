#!/usr/bin/env python3
"""
stereo_calib_capture.py

Captures checkerboard image pairs for stereo calibration of the IMX219-83.
Uses the same camera pipeline as stereo_test.py (imported from the same folder), so
pass the same --flip you will use afterwards. A left/right swap is detected and
corrected later by stereo_calibrate.py.

A pair is saved automatically when the board is found in both images, has been held
still for a moment (the two sensors are not hardware-synchronised, so a moving board
gives mismatched pairs), and is in a new pose compared with the last saved pair.
Green tint shows where saved boards have covered each image. Aim to fill both.

Keys:
  space     force-save the current pair (board must be found in both)
  q / Esc   quit
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stereo_test import HINTS, Camera, check_gstreamer, gst_pipeline, put  # noqa: E402

WIN = "Calibration capture"
STILL_PX = 0.7       # mean corner motion between frames that counts as still
STILL_FRAMES = 6     # consecutive still frames needed before an auto-save
NEW_POSE_PX = 40.0   # mean corner distance from the last saved pair that counts as new


def parse_args():
    p = argparse.ArgumentParser(description="Stereo calibration pair capture")
    p.add_argument("--left", type=int, default=0)
    p.add_argument("--right", type=int, default=1)
    p.add_argument("--flip", type=int, default=0)
    p.add_argument("--cols", type=int, default=9, help="inner corners per row")
    p.add_argument("--rows", type=int, default=6, help="inner corners per column")
    p.add_argument("--out", default="~/stereo_calib/pairs")
    p.add_argument("--cap-width", type=int, default=1640)
    p.add_argument("--cap-height", type=int, default=1232)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--manual", action="store_true", help="disable auto-capture")
    return p.parse_args()


def find(gray, pattern):
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    return corners.reshape(-1, 2) if ok else None


def mean_dist(a, b):
    return float(np.mean(np.linalg.norm(a - b, axis=1)))


def tint(img, mask):
    out = img.copy()
    sel = mask > 0
    out[sel] = (0.65 * img[sel] + 0.35 * np.array([0, 255, 0])).astype(np.uint8)
    return out


def main():
    args = parse_args()
    pattern = (args.cols, args.rows)
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)

    settings = {k: getattr(args, k) for k in
                ("left", "right", "flip", "cols", "rows", "cap_width", "cap_height",
                 "width", "height")}
    settings_path = os.path.join(out_dir, "capture_settings.json")
    if os.path.exists(settings_path):
        with open(settings_path) as f:
            old = json.load(f)
        if old != settings:
            sys.exit(f"{out_dir} already holds pairs captured with different settings:\n"
                     f"  existing: {old}\n  now:      {settings}\n"
                     "Use the same settings, or move that folder aside first.")
    else:
        with open(settings_path, "w") as f:
            json.dump(settings, f, indent=2)

    idx = len([n for n in os.listdir(out_dir) if n.endswith("_left.png")])
    check_gstreamer()

    W, H = args.width, args.height
    cams = []
    for sid in (args.left, args.right):
        cam = Camera(sid, gst_pipeline(sid, args.cap_width, args.cap_height, args.fps,
                                       W, H, args.flip))
        print(f"sensor-id={sid}: {'opened' if cam.ok else 'FAILED to open'}")
        cams.append(cam)
    if not all(c.ok for c in cams):
        for c in cams:
            c.stop()
        sys.exit("Both cameras are needed for calibration." + HINTS)
    for c in cams:
        c.start()

    cov = [np.zeros((H, W), np.uint8), np.zeros((H, W), np.uint8)]
    prev, last_saved = None, None
    still = 0
    last_counts = (-1, -1)
    flash = 0
    print(f"Saving to {out_dir} (starting at pair {idx}). "
          "Hold the board still in each pose. q to quit.")

    try:
        while True:
            (fl, _, nl), (fr, _, nr) = cams[0].latest(), cams[1].latest()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if fl is None or fr is None or (nl, nr) == last_counts:
                continue
            last_counts = (nl, nr)

            gl = cv2.cvtColor(fl, cv2.COLOR_BGR2GRAY)
            gr = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
            cl, cr = find(gl, pattern), find(gr, pattern)
            both = cl is not None and cr is not None

            if both and prev is not None:
                motion = max(mean_dist(cl, prev[0]), mean_dist(cr, prev[1]))
                still = still + 1 if motion < STILL_PX else 0
            else:
                still = 0
            prev = (cl, cr) if both else None

            new_pose = both and (last_saved is None or mean_dist(cl, last_saved) > NEW_POSE_PX)
            auto_ok = not args.manual and both and still >= STILL_FRAMES and new_pose
            if auto_ok or (key == ord(" ") and both):
                cv2.imwrite(os.path.join(out_dir, f"pair_{idx:03d}_left.png"), fl)
                cv2.imwrite(os.path.join(out_dir, f"pair_{idx:03d}_right.png"), fr)
                for m, c in ((cov[0], cl), (cov[1], cr)):
                    cv2.fillConvexPoly(m, cv2.convexHull(c.astype(np.int32)), 255)
                print(f"saved pair {idx:03d}")
                idx += 1
                last_saved = cl
                still = 0
                flash = 3

            views = []
            for img, corners, mask in ((fl, cl, cov[0]), (fr, cr, cov[1])):
                v = tint(img, mask)
                if corners is not None:
                    cv2.drawChessboardCorners(v, pattern, corners.reshape(-1, 1, 2), True)
                views.append(v)
            vis = np.hstack(views)
            cv2.line(vis, (W, 0), (W, H), (255, 255, 255), 2)

            if not both:
                status, col = "board must be fully visible in BOTH images", (0, 0, 255)
            elif not new_pose:
                status, col = "move to a new pose", (0, 200, 255)
            elif still < STILL_FRAMES:
                status, col = f"hold still {still}/{STILL_FRAMES}", (0, 255, 255)
            else:
                status, col = "ready (space to save)" if args.manual else "saving", (0, 255, 0)
            put(vis, f"pairs: {idx}   L: {'OK' if cl is not None else '--'}   "
                     f"R: {'OK' if cr is not None else '--'}   {status}", (10, 22), col)
            put(vis, "fill both images with green: near, far, tilted, corners and edges",
                (10, H - 12))
            if flash:
                cv2.rectangle(vis, (0, 0), (2 * W - 1, H - 1), (255, 255, 255), 8)
                flash -= 1
            cv2.imshow(WIN, vis)
    except KeyboardInterrupt:
        pass
    finally:
        for c in cams:
            c.stop()
        cv2.destroyAllWindows()
        print(f"\n{idx} pairs in {out_dir}")


if __name__ == "__main__":
    main()
