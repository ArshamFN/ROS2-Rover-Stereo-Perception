#!/usr/bin/env python3
"""
stereo_align_view.py

Quick vertical alignment check, no calibration target needed.
Measures the vertical offset between the two cameras from matched features, shifts the
right image to cancel it, and shows the depth map before and after side by side.

Keys:
  q / Esc   quit
  a         re-measure the offset (point at a busy scene first)
  w / x     nudge the shift up / down by 1 px
  s         swap left and right
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stereo_test import (HINTS, Camera, build_matcher, check_gstreamer,  # noqa: E402
                         colorize, gst_pipeline, put)

WIN = "Stereo: vertical alignment"


def measure_offset(gl, gr):
    """Median (dy, dx) = left minus right over RANSAC-filtered ORB matches."""
    orb = cv2.ORB_create(4000)
    k1, d1 = orb.detectAndCompute(gl, None)
    k2, d2 = orb.detectAndCompute(gr, None)
    if d1 is None or d2 is None:
        return None
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d1, d2, k=2)
    good = [a for a, b in (p for p in pairs if len(p) == 2) if a.distance < 0.75 * b.distance]
    if len(good) < 30:
        return None
    p1 = np.float32([k1[g.queryIdx].pt for g in good])
    p2 = np.float32([k2[g.trainIdx].pt for g in good])
    _, mask = cv2.findFundamentalMat(p1, p2, cv2.FM_RANSAC, 1.5, 0.99)
    if mask is None or mask.sum() < 20:
        return None
    inl = mask.ravel().astype(bool)
    return (float(np.median(p1[inl, 1] - p2[inl, 1])),
            float(np.median(p1[inl, 0] - p2[inl, 0])), int(inl.sum()))


def shift(img, dy):
    h, w = img.shape[:2]
    return cv2.warpAffine(img, np.float32([[1, 0, 0], [0, 1, dy]]), (w, h))


def main():
    p = argparse.ArgumentParser(description="Vertical alignment check")
    p.add_argument("--left", type=int, default=0)
    p.add_argument("--right", type=int, default=1)
    p.add_argument("--flip", type=int, default=0)
    args = p.parse_args()
    check_gstreamer()

    W, H = 640, 480
    cams = {sid: Camera(sid, gst_pipeline(sid, 1640, 1232, 30, W, H, args.flip))
            for sid in (args.left, args.right)}
    if not all(c.ok for c in cams.values()):
        for c in cams.values():
            c.stop()
        sys.exit("Both cameras are needed." + HINTS)
    for c in cams.values():
        c.start()
    time.sleep(2.0)

    left_id, right_id = args.left, args.right
    matcher = build_matcher(64, 5)
    dy, meas = 0.0, None
    remeasure = True
    try:
        while True:
            fl, fr = cams[left_id].latest()[0], cams[right_id].latest()[0]
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("a"):
                remeasure = True
            elif key == ord("w"):
                dy -= 1
            elif key == ord("x"):
                dy += 1
            elif key == ord("s"):
                left_id, right_id = right_id, left_id
                remeasure = True
            if fl is None or fr is None:
                continue

            gl = cv2.cvtColor(fl, cv2.COLOR_BGR2GRAY)
            gr = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
            if remeasure:
                m = measure_offset(gl, gr)
                if m is not None:
                    meas = m
                    dy = round(m[0])
                    print(f"measured offset {m[0]:+.1f} px, disparity {m[1]:+.1f} px, "
                          f"{m[2]} matches -> shift {dy:+.0f} px")
                remeasure = False

            fr_s, gr_s = shift(fr, dy), shift(gr, dy)
            top = np.hstack([fl, fr_s])
            for y in range(0, H, 40):
                cv2.line(top, (0, y), (2 * W, y), (0, 255, 0), 1)
            cv2.line(top, (W, 0), (W, H), (255, 255, 255), 2)

            raw_c, raw_v = colorize(matcher.compute(gl, gr).astype(np.float32) / 16.0, 64)
            al_c, al_v = colorize(matcher.compute(gl, gr_s).astype(np.float32) / 16.0, 64)
            put(raw_c, f"BEFORE (raw)   valid {100 * raw_v.mean():.0f}%", (10, 22))
            put(al_c, f"AFTER (shift {dy:+.0f} px)   valid {100 * al_v.mean():.0f}%", (10, 22))
            bottom = np.hstack([raw_c, al_c])

            if meas is None:
                msg, col = "no offset measured yet: point at a busy scene, press a", (0, 0, 255)
            elif meas[1] < 0:
                msg, col = "disparity NEGATIVE: left/right swapped, press s", (0, 0, 255)
            else:
                msg, col = (f"left=sensor {left_id}  right=sensor {right_id}  "
                            f"measured {meas[0]:+.1f} px from {meas[2]} matches"), (0, 255, 0)
            put(top, msg, (10, 22), col)
            put(top, "same object should sit on the same green line in both images",
                (10, H - 12))
            cv2.imshow(WIN, np.vstack([top, bottom]))
    except KeyboardInterrupt:
        pass
    finally:
        for c in cams.values():
            c.stop()
        cv2.destroyAllWindows()
        print(f"final shift {dy:+.0f} px with left=sensor {left_id}, right=sensor {right_id}")


if __name__ == "__main__":
    main()
