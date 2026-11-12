#!/usr/bin/env python3
"""
stereo_pair_errors.py

Per-pair rectified row error for an existing calibration, worst first.
stereo_calibrate.py reports one mean over every corner of every pair; this shows
which pairs that error comes from. A pair whose board moved between the left and
right exposures (the sensors are not synchronised) has a large row error even when
each image on its own fits well.

  python3 stereo_pair_errors.py
  python3 stereo_pair_errors.py --calib ~/stereo_calib/stereo_calib.yaml --pairs ~/stereo_calib/pairs

Imports load_settings, load_pairs and rectified_rows from stereo_calibrate.py, so
both files must be in the same folder. Reads only; changes nothing.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stereo_calibrate import load_pairs, load_settings, rectified_rows  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="Per-pair rectified row error")
    p.add_argument("--calib", default="~/stereo_calib/stereo_calib.yaml")
    p.add_argument("--pairs", default="~/stereo_calib/pairs")
    args = p.parse_args()

    calib = os.path.expanduser(args.calib)
    if not os.path.exists(calib):
        sys.exit(f"No calibration at {calib}")
    fs = cv2.FileStorage(calib, cv2.FILE_STORAGE_READ)
    M = {k: fs.getNode(k).mat() for k in ("K1", "D1", "K2", "D2", "R1", "R2", "P1", "P2")}
    left_id = int(fs.getNode("left_sensor_id").real())
    baseline = fs.getNode("baseline_mm").real()
    fs.release()

    pairs_dir = os.path.expanduser(args.pairs)
    st = load_settings(pairs_dir)
    names, img_l, img_r, _ = load_pairs(pairs_dir, (st["cols"], st["rows"]))
    if not names:
        sys.exit("No usable pairs.")
    if st["left"] != left_id:
        img_l, img_r = img_r, img_l

    fx = M["P1"][0, 0]
    rows, all_dy = [], []
    for n, cl, cr in zip(names, img_l, img_r):
        dy, dx = rectified_rows([cl], [cr], M["K1"], M["D1"], M["K2"], M["D2"],
                                M["R1"], M["R2"], M["P1"], M["P2"])
        all_dy.append(dy)
        d = float(np.median(dx))
        z = fx * baseline / d if d > 0 else float("nan")
        rows.append((float(np.mean(np.abs(dy))), float(np.mean(dy)), float(np.std(dy)), z, n))

    rows.sort(reverse=True)
    print(f"{'pair':10s} {'|dy| mean':>10s} {'signed':>8s} {'spread':>8s} {'distance':>9s}")
    for mean_abs, signed, spread, z, n in rows:
        flag = "  <- over 0.5 px" if mean_abs > 0.5 else ""
        print(f"{n:10s} {mean_abs:8.3f} px {signed:+7.3f} {spread:7.3f}  {z / 1000:6.2f} m{flag}")

    dy_all = np.abs(np.concatenate(all_dy))
    print(f"\nAll pairs: mean {dy_all.mean():.3f} px, 95% {np.percentile(dy_all, 95):.3f} px")
    good = [r for r in rows if r[0] <= 0.5]
    print(f"{len(rows) - len(good)} of {len(rows)} pairs are over 0.5 px.")


if __name__ == "__main__":
    main()
