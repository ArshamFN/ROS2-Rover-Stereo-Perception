#!/usr/bin/env python3
"""
stereo_calibrate.py

Stereo calibration of the IMX219-83 from the pairs saved by stereo_calib_capture.py.

  python3 stereo_calibrate.py --square-mm 25.0
      Calibrates each camera, then the pair, rectifies, prints a quality report, and
      writes ~/stereo_calib/stereo_calib.yaml plus rectified_check.png.

  python3 stereo_calibrate.py --square-mm 24.9 --free-k3
      Same, but also estimates the third radial distortion term.

  python3 stereo_calibrate.py --check --pairs ~/stereo_calib/check_pairs
      Uses an existing calibration on NEW pairs and reports the rectified row error.
      If it has grown, something moved and the calibration is no longer valid.

Conventions: OpenCV camera frame (x right, y down, z forward), units in mm.
A correct left/right assignment gives T[0] < 0. If it comes out positive, the script
swaps the roles, recalibrates, and records the corrected sensor-ids.
"""

import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
SPEC_BASELINE_MM = 60.0


def parse_args():
    p = argparse.ArgumentParser(description="IMX219-83 stereo calibration")
    p.add_argument("--pairs", default="~/stereo_calib/pairs")
    p.add_argument("--square-mm", type=float, help="measured checkerboard square size")
    p.add_argument("--out", default="~/stereo_calib/stereo_calib.yaml")
    p.add_argument("--check", action="store_true", help="check an existing calibration")
    p.add_argument("--free-k3", action="store_true", help="also estimate the third radial term")
    return p.parse_args()


def load_settings(pairs_dir):
    path = os.path.join(pairs_dir, "capture_settings.json")
    if not os.path.exists(path):
        sys.exit(f"Missing {path}. Capture pairs with stereo_calib_capture.py.")
    with open(path) as f:
        return json.load(f)


def detect(path, pattern):
    # same conversion and flags as stereo_calib_capture.py, so a pair the capture
    # accepted is found again here
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        return None, None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    ok, c = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return gray.shape[::-1], None
    # refinement window smaller than half a square, so it cannot reach the next corner
    pts = c.reshape(pattern[1], pattern[0], 2)
    sq = float(np.median(np.linalg.norm(np.diff(pts, axis=1), axis=2)))
    win = int(np.clip(0.4 * sq, 2, 5))
    c = cv2.cornerSubPix(gray, c, (win, win), (-1, -1), CRIT)
    return gray.shape[::-1], c.reshape(-1, 1, 2).astype(np.float32)


def load_pairs(pairs_dir, pattern):
    names, img_l, img_r, size = [], [], [], None
    for lp in sorted(glob.glob(os.path.join(pairs_dir, "*_left.png"))):
        rp = lp[:-len("_left.png")] + "_right.png"
        if not os.path.exists(rp):
            continue
        sl, cl = detect(lp, pattern)
        sr, cr = detect(rp, pattern)
        if cl is None or cr is None:
            print(f"  skip {os.path.basename(lp)}: board not found in both")
            continue
        # guard against the detector ordering the two boards in opposite directions
        if np.mean(np.linalg.norm(cl - cr[::-1], axis=2)) < np.mean(np.linalg.norm(cl - cr, axis=2)):
            cr = cr[::-1].copy()
        size = sl
        names.append(os.path.basename(lp)[:-len("_left.png")])
        img_l.append(cl)
        img_r.append(cr)
    return names, img_l, img_r, size


def view_errors(obj, imgs, K, D, rvecs, tvecs):
    errs = []
    for o, i, r, t in zip(obj, imgs, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, D)
        errs.append(float(np.sqrt(np.mean(np.sum((proj - i) ** 2, axis=2)))))
    return np.array(errs)


def calibrate_mono(obj, imgs, size, free_k3=False):
    # also returns the standard deviations of fx, fy, cx, cy, k1, ... from the solver
    flags = 0 if free_k3 else cv2.CALIB_FIX_K3
    rms, K, D, rv, tv, sd, _, _ = cv2.calibrateCameraExtended(
        obj, imgs, size, None, None, flags=flags, criteria=CRIT)
    return rms, K, D, view_errors(obj, imgs, K, D, rv, tv), sd.ravel()


def rectified_rows(img_l, img_r, K1, D1, K2, D2, R1, R2, P1, P2):
    dy, dx = [], []
    for cl, cr in zip(img_l, img_r):
        ul = cv2.undistortPoints(cl, K1, D1, R=R1, P=P1).reshape(-1, 2)
        ur = cv2.undistortPoints(cr, K2, D2, R=R2, P=P2).reshape(-1, 2)
        dy.append(ul[:, 1] - ur[:, 1])
        dx.append(ul[:, 0] - ur[:, 0])
    return np.concatenate(dy), np.concatenate(dx)


def grade(value, good, ok):
    return "GOOD" if value <= good else ("OK" if value <= ok else "BAD")


def calibrate(args):
    if args.square_mm is None:
        sys.exit("--square-mm is required: measure the printed squares with calipers.")
    pairs_dir = os.path.expanduser(args.pairs)
    st = load_settings(pairs_dir)
    pattern = (st["cols"], st["rows"])
    names, img_l, img_r, size = load_pairs(pairs_dir, pattern)
    if len(names) < 12:
        sys.exit(f"Only {len(names)} usable pairs. Capture at least 20.")

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * args.square_mm
    obj = [objp] * len(names)

    # per-camera intrinsics, then drop clear outlier views once
    fk3 = args.free_k3
    rms_l, K1, D1, el, sd_l = calibrate_mono(obj, img_l, size, fk3)
    rms_r, K2, D2, er, sd_r = calibrate_mono(obj, img_r, size, fk3)
    worst = np.maximum(el, er)
    keep = worst <= max(1.0, 3.0 * float(np.median(worst)))
    if not keep.all():
        print("Dropped outlier pairs: " + ", ".join(n for n, k in zip(names, keep) if not k))
        names = [n for n, k in zip(names, keep) if k]
        img_l = [c for c, k in zip(img_l, keep) if k]
        img_r = [c for c, k in zip(img_r, keep) if k]
        obj = [objp] * len(names)
        rms_l, K1, D1, el, sd_l = calibrate_mono(obj, img_l, size, fk3)
        rms_r, K2, D2, er, sd_r = calibrate_mono(obj, img_r, size, fk3)

    left_id, right_id = st["left"], st["right"]

    def stereo(il, ir, Ka, Da, Kb, Db):
        return cv2.stereoCalibrate(obj, il, ir, Ka, Da, Kb, Db, size,
                                   flags=cv2.CALIB_FIX_INTRINSIC, criteria=CRIT)

    rms_s, K1, D1, K2, D2, R, T, E, F = stereo(img_l, img_r, K1, D1, K2, D2)
    swapped = False
    if T[0, 0] > 0:
        swapped = True
        img_l, img_r = img_r, img_l
        K1, D1, K2, D2 = K2, D2, K1, D1
        rms_l, rms_r = rms_r, rms_l
        el, er = er, el
        sd_l, sd_r = sd_r, sd_l
        left_id, right_id = right_id, left_id
        rms_s, K1, D1, K2, D2, R, T, E, F = stereo(img_l, img_r, K1, D1, K2, D2)

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    dy, dx = rectified_rows(img_l, img_r, K1, D1, K2, D2, R1, R2, P1, P2)

    baseline = float(np.linalg.norm(T))
    rot_deg = np.degrees(cv2.Rodrigues(R)[0].ravel())
    w, h = size
    vfov = 2 * np.degrees(np.arctan(h / 2 / K1[1, 1]))
    hfov = 2 * np.degrees(np.arctan(w / 2 / K1[0, 0]))
    vfov_r = 2 * np.degrees(np.arctan(h / 2 / P1[1, 1]))
    hfov_r = 2 * np.degrees(np.arctan(w / 2 / P1[0, 0]))
    base_err = abs(baseline - SPEC_BASELINE_MM) / SPEC_BASELINE_MM * 100

    print(f"\nPairs used: {len(names)}   image size: {w}x{h}")
    if swapped:
        print(f"LEFT/RIGHT WERE SWAPPED in capture. Corrected: left=sensor-id {left_id}, "
              f"right=sensor-id {right_id}. Use these from now on.")
    else:
        print(f"Left/right correct: left=sensor-id {left_id}, right=sensor-id {right_id}")
    print("\nQuality")
    print(f"  left  reprojection RMS   {rms_l:6.3f} px  {grade(rms_l, 0.5, 1.0)}")
    print(f"  right reprojection RMS   {rms_r:6.3f} px  {grade(rms_r, 0.5, 1.0)}")
    print(f"  stereo RMS               {rms_s:6.3f} px  {grade(rms_s, 0.7, 1.5)}")
    print(f"  rectified row error      mean {np.mean(np.abs(dy)):.3f} px, "
          f"95% {np.percentile(np.abs(dy), 95):.3f} px  {grade(np.mean(np.abs(dy)), 0.5, 1.0)}")
    print(f"  baseline                 {baseline:6.2f} mm (spec {SPEC_BASELINE_MM:.0f}, "
          f"off {base_err:.1f}%)  {grade(base_err, 3.0, 6.0)}")
    print("\nGeometry")
    print(f"  principal point left     ({K1[0, 2]:.1f}, {K1[1, 2]:.1f})")
    print(f"  principal point right    ({K2[0, 2]:.1f}, {K2[1, 2]:.1f})   "
          f"cy difference {K2[1, 2] - K1[1, 2]:+.1f} px")
    print(f"  right vs left rotation   pitch {rot_deg[0]:+.2f} deg, yaw {rot_deg[1]:+.2f} deg, "
          f"roll {rot_deg[2]:+.2f} deg")
    print(f"  focal length left        fx {K1[0, 0]:.1f} +/- {sd_l[0]:.1f}, fy {K1[1, 1]:.1f} +/- {sd_l[1]:.1f} px")
    print(f"  focal length right       fx {K2[0, 0]:.1f} +/- {sd_r[0]:.1f}, fy {K2[1, 1]:.1f} +/- {sd_r[1]:.1f} px")
    print(f"  distortion model         {'k1 k2 p1 p2 k3' if fk3 else 'k1 k2 p1 p2 (k3 fixed at 0)'}")
    print(f"  distortion left          {np.array2string(D1.ravel(), precision=4)}")
    print(f"  distortion right         {np.array2string(D2.ravel(), precision=4)}")
    print(f"  FOV raw                  {hfov:.1f} x {vfov:.1f} deg")
    print(f"  FOV after rectification  {hfov_r:.1f} x {vfov_r:.1f} deg")
    print(f"  rectified disparity sign {'positive (correct)' if np.median(dx) > 0 else 'NEGATIVE (wrong)'}")

    print("\nWorst pairs (max of left/right reprojection error)")
    order = np.argsort(-np.maximum(el, er))[:5]
    for i in order:
        print(f"  {names[i]}  left {el[i]:.3f}  right {er[i]:.3f}")

    out = os.path.expanduser(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fs = cv2.FileStorage(out, cv2.FILE_STORAGE_WRITE)
    fs.write("date", time.strftime("%Y-%m-%d %H:%M:%S"))
    fs.write("left_sensor_id", int(left_id))
    fs.write("right_sensor_id", int(right_id))
    for k in ("flip", "cap_width", "cap_height", "width", "height", "cols", "rows"):
        fs.write(k, int(st[k]))
    fs.write("square_mm", float(args.square_mm))
    fs.write("free_k3", int(fk3))
    fs.write("pairs_used", int(len(names)))
    fs.write("rms_left", float(rms_l))
    fs.write("rms_right", float(rms_r))
    fs.write("rms_stereo", float(rms_s))
    fs.write("rect_row_error_mean", float(np.mean(np.abs(dy))))
    fs.write("baseline_mm", baseline)
    for k, v in (("K1", K1), ("D1", D1), ("K2", K2), ("D2", D2), ("R", R), ("T", T),
                 ("E", E), ("F", F), ("R1", R1), ("R2", R2), ("P1", P1), ("P2", P2),
                 ("Q", Q), ("roi1", np.array(roi1)), ("roi2", np.array(roi2))):
        fs.write(k, v)
    fs.release()
    print(f"\nSaved {out}")

    # rectified preview of the first pair: rows should line up across the divider
    lp = os.path.join(pairs_dir, f"{names[0]}_{'right' if swapped else 'left'}.png")
    rp = os.path.join(pairs_dir, f"{names[0]}_{'left' if swapped else 'right'}.png")
    il, ir = cv2.imread(lp), cv2.imread(rp)
    m1 = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_16SC2)
    m2 = cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_16SC2)
    rect = np.hstack([cv2.remap(il, *m1, cv2.INTER_LINEAR), cv2.remap(ir, *m2, cv2.INTER_LINEAR)])
    raw = np.hstack([il, ir])
    for img in (raw, rect):
        for y in range(0, h, 30):
            cv2.line(img, (0, y), (2 * w, y), (0, 255, 0), 1)
    preview = os.path.join(os.path.dirname(out), "rectified_check.png")
    cv2.imwrite(preview, np.vstack([raw, rect]))
    print(f"Saved {preview} (top: raw, bottom: rectified)")


def check(args):
    calib = os.path.expanduser(args.out)
    if not os.path.exists(calib):
        sys.exit(f"No calibration at {calib}")
    fs = cv2.FileStorage(calib, cv2.FILE_STORAGE_READ)
    M = {k: fs.getNode(k).mat() for k in ("K1", "D1", "K2", "D2", "R1", "R2", "P1", "P2")}
    left_id = int(fs.getNode("left_sensor_id").real())
    flip = int(fs.getNode("flip").real())
    base = fs.getNode("rect_row_error_mean").real()
    fs.release()

    pairs_dir = os.path.expanduser(args.pairs)
    st = load_settings(pairs_dir)
    if st["flip"] != flip:
        sys.exit(f"Pairs were captured with --flip {st['flip']}, calibration used {flip}.")
    names, img_l, img_r, _ = load_pairs(pairs_dir, (st["cols"], st["rows"]))
    if not names:
        sys.exit("No usable pairs.")
    if st["left"] != left_id:
        img_l, img_r = img_r, img_l
    dy, dx = rectified_rows(img_l, img_r, M["K1"], M["D1"], M["K2"], M["D2"],
                            M["R1"], M["R2"], M["P1"], M["P2"])
    mean = float(np.mean(np.abs(dy)))
    print(f"Pairs checked: {len(names)}")
    print(f"Rectified row error: mean {mean:.3f} px (at calibration {base:.3f} px), "
          f"95% {np.percentile(np.abs(dy), 95):.3f} px, signed mean {np.mean(dy):+.3f} px")
    print(f"Disparity sign: {'positive (correct)' if np.median(dx) > 0 else 'NEGATIVE (wrong)'}")
    if mean <= max(0.5, 1.5 * base):
        print("STILL VALID")
    elif mean <= 1.0:
        print("DRIFTING: recalibrate soon")
    else:
        print("INVALID: something moved or the settings differ. Recalibrate.")


if __name__ == "__main__":
    a = parse_args()
    check(a) if a.check else calibrate(a)
