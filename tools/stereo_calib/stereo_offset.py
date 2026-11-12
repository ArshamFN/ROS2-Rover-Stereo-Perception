#!/usr/bin/env python3
"""
stereo_offset.py

Measures the vertical misalignment between the left and right images of every capture
pair saved by stereo_test.py (the 'c' key) in ~/stereo_test_captures/.

For each pair: ORB feature matches, ratio test, RANSAC on the fundamental matrix, then
dy = y_left - y_right (vertical offset) and dx = x_left - x_right (disparity) over the
inlier matches. A stable dy across captures means a fixed mechanical/optical offset that
calibration can remove. A dy that changes between captures means something is loose.

Usage: python3 stereo_offset.py [capture_dir]
"""

import glob
import os
import sys

import cv2
import numpy as np


def measure(left_path, right_path):
    gl = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    gr = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
    if gl is None or gr is None:
        return None
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
    if mask is None:
        return None
    inl = mask.ravel().astype(bool)
    if inl.sum() < 20:
        return None
    dx = p1[inl, 0] - p2[inl, 0]
    dy = p1[inl, 1] - p2[inl, 1]
    x = p1[inl, 0]
    core = np.abs(dy - np.median(dy)) < 3.0  # fit roll on the consistent core only
    slope = np.polyfit(x[core] - x[core].mean(), dy[core], 1)[0] if core.sum() >= 10 else 0.0
    return {
        "n": int(inl.sum()),
        "dy": float(np.median(dy)),
        "dy_spread": float(np.percentile(dy, 90) - np.percentile(dy, 10)),
        "dx": float(np.median(dx)),
        "roll_deg": float(np.degrees(np.arctan(slope))),
        "height": gl.shape[0],
    }


def main():
    cap_dir = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/stereo_test_captures")
    lefts = sorted(glob.glob(os.path.join(cap_dir, "*_left.png")))
    if not lefts:
        sys.exit(f"No *_left.png captures in {cap_dir}. Press 'c' in stereo_test.py first.")
    print(f"{'capture':<17} {'inliers':>7} {'dy px':>7} {'dy % H':>7} {'spread':>7} "
          f"{'dx px':>7} {'roll deg':>8}")
    dys = []
    for lp in lefts:
        stamp = os.path.basename(lp)[:-len("_left.png")]
        rp = os.path.join(cap_dir, f"{stamp}_right.png")
        r = measure(lp, rp) if os.path.exists(rp) else None
        if r is None:
            print(f"{stamp:<17} not enough texture or matches, point at a busier scene")
            continue
        dys.append(r["dy"])
        print(f"{stamp:<17} {r['n']:>7} {r['dy']:>7.1f} {100 * r['dy'] / r['height']:>6.1f}% "
              f"{r['dy_spread']:>7.1f} {r['dx']:>7.1f} {r['roll_deg']:>8.2f}")
    if len(dys) >= 2:
        print(f"\ndy range across captures: {max(dys) - min(dys):.1f} px "
              f"(min {min(dys):.1f}, max {max(dys):.1f})")
    print("\ndx should be positive (left/right assigned correctly).")


if __name__ == "__main__":
    main()
