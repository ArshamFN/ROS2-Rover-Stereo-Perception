#!/usr/bin/env python3
"""
stereo_depth_view.py

Calibrated depth from the IMX219-83, for checking it against a tape measure.
Reads ~/stereo_calib/stereo_calib.yaml (written by stereo_calibrate.py), so the
camera settings (flip, left/right sensor-ids, sensor mode, output size) come from
the calibration itself.

  python3 stereo_depth_view.py
      Live: rectified left image with a measurement box, and the depth map.
  python3 stereo_depth_view.py --images LEFT.png RIGHT.png
      Same processing on one saved pair (raw, as saved by stereo_calib_capture.py).

Depth is Z in mm along the left camera's optical axis, measured from its optical
centre, which sits a few mm behind the front of the lens.

Readouts:
  box     median depth of the valid pixels in the box, their spread (25 to 75%),
          and the share of the box that has a depth at all.
  board   (key b) if the checkerboard is in view: its distance from its own
          printed size (left camera only, no baseline), the stereo depth at its
          corners, and the ratio between the two. A ratio away from 1 points at
          the baseline.

Keys (live):
  click     move the measurement box
  b         board readout on/off
  c         log the current readings to ~/stereo_calib/depth_log.csv
  q / Esc   quit
"""

import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np

WIN = "Stereo depth (calibrated)"
BOX = 10           # half-size of the measurement box in px
Z_MIN, Z_MAX = 250.0, 4000.0   # colour range of the depth map in mm


def load_calib(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        sys.exit(f"No calibration at {path}")
    c = {k: fs.getNode(k).mat() for k in ("K1", "D1", "K2", "D2", "R1", "R2", "P1", "P2")}
    for k in ("left_sensor_id", "right_sensor_id", "flip", "cap_width", "cap_height",
              "width", "height", "cols", "rows"):
        c[k] = int(fs.getNode(k).real())
    for k in ("square_mm", "baseline_mm"):
        c[k] = fs.getNode(k).real()
    fs.release()
    return c


class Depth:
    def __init__(self, c, num_disp, block):
        self.c = c
        size = (c["width"], c["height"])
        self.m1 = cv2.initUndistortRectifyMap(c["K1"], c["D1"], c["R1"], c["P1"], size, cv2.CV_16SC2)
        self.m2 = cv2.initUndistortRectifyMap(c["K2"], c["D2"], c["R2"], c["P2"], size, cv2.CV_16SC2)
        self.fx = float(c["P1"][0, 0])
        self.fb = float(-c["P2"][0, 3])          # focal length x baseline, px*mm
        self.Krect = c["P1"][:, :3].astype(np.float64)
        self.num_disp = num_disp
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=num_disp, blockSize=block,
            P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        pattern = (c["cols"], c["rows"])
        self.pattern = pattern
        objp = np.zeros((pattern[0] * pattern[1], 3), np.float64)
        objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * c["square_mm"]
        self.objp = objp

    def process(self, left_bgr, right_bgr):
        rl = cv2.remap(left_bgr, *self.m1, cv2.INTER_LINEAR)
        rr = cv2.remap(right_bgr, *self.m2, cv2.INTER_LINEAR)
        gl = cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        disp = self.sgbm.compute(gl, gr).astype(np.float32) / 16.0
        z = np.full(disp.shape, np.nan, np.float32)
        ok = disp > 0.5
        z[ok] = self.fb / disp[ok]
        return rl, gl, disp, z

    def box_reading(self, z, cx, cy):
        h, w = z.shape
        x0, x1 = max(0, cx - BOX), min(w, cx + BOX + 1)
        y0, y1 = max(0, cy - BOX), min(h, cy + BOX + 1)
        patch = z[y0:y1, x0:x1]
        vals = patch[np.isfinite(patch)]
        valid = vals.size / patch.size if patch.size else 0.0
        if vals.size == 0:
            return None, None, None, valid
        p25, med, p75 = np.percentile(vals, [25, 50, 75])
        return float(med), float(p25), float(p75), valid

    def board_reading(self, gl, z):
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(gl, self.pattern, flags)
        if not found:
            return None
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
        corners = cv2.cornerSubPix(gl, corners, (3, 3), (-1, -1), crit).reshape(-1, 2)
        ok, rvec, tvec = cv2.solvePnP(self.objp, corners.astype(np.float64), self.Krect, None)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        pts_cam = (R @ self.objp.T + tvec).T          # corners in the camera frame, mm
        z_pnp = pts_cam[:, 2]
        h, w = z.shape
        z_st = []
        for (u, v) in corners:
            ui, vi = int(round(u)), int(round(v))
            patch = z[max(0, vi - 2):min(h, vi + 3), max(0, ui - 2):min(w, ui + 3)]
            vals = patch[np.isfinite(patch)]
            z_st.append(np.median(vals) if vals.size else np.nan)
        z_st = np.array(z_st)
        good = np.isfinite(z_st)
        if good.sum() < 10:
            return {"corners": corners, "pnp": float(np.median(z_pnp)), "stereo": None,
                    "ratio": None, "used": int(good.sum())}
        ratio = float(np.median(z_st[good] / z_pnp[good]))
        return {"corners": corners, "pnp": float(np.median(z_pnp[good])),
                "stereo": float(np.median(z_st[good])), "ratio": ratio, "used": int(good.sum())}


def colour_depth(z):
    zc = np.clip(np.nan_to_num(z, nan=Z_MAX), Z_MIN, Z_MAX)
    norm = ((Z_MAX - zc) / (Z_MAX - Z_MIN) * 255).astype(np.uint8)   # near = red
    vis = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    vis[~np.isfinite(z)] = 0
    return vis


def fmt_mm(v):
    return "--" if v is None else f"{v / 1000:.3f} m"


def readout_lines(box, board, num_disp, fb):
    med, p25, p75, valid = box
    lines = [f"box: {fmt_mm(med)}  (25-75%: {fmt_mm(p25)} .. {fmt_mm(p75)})  valid {valid * 100:.0f}%"]
    if board is not None:
        if board["ratio"] is None:
            lines.append(f"board: size {fmt_mm(board['pnp'])}  stereo -- (only {board['used']} corners)")
        else:
            lines.append(f"board: size {fmt_mm(board['pnp'])}  stereo {fmt_mm(board['stereo'])}  "
                         f"ratio {board['ratio']:.4f} ({(board['ratio'] - 1) * 100:+.2f}%)")
    lines.append(f"closest measurable: {fmt_mm(fb / num_disp)} ({num_disp} disparities)")
    return lines


def draw(rl, z, box_c, box, board, num_disp, fb, fps=None):
    W = rl.shape[1]
    left = rl.copy()
    cx, cy = box_c
    cv2.rectangle(left, (cx - BOX, cy - BOX), (cx + BOX, cy + BOX), (255, 255, 255), 1)
    if board is not None:
        for (u, v) in board["corners"]:
            cv2.circle(left, (int(u), int(v)), 2, (0, 255, 0), -1)
    vis = np.hstack([left, colour_depth(z)])
    cv2.line(vis, (W, 0), (W, vis.shape[0]), (255, 255, 255), 2)
    lines = readout_lines(box, board, num_disp, fb)
    if fps is not None:
        lines.append(f"{fps:.1f} fps")
    for i, t in enumerate(lines):
        (tw, th), base = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        y = 22 + 22 * i
        cv2.rectangle(vis, (6, y - th - 4), (14 + tw, y + base + 2), (0, 0, 0), -1)
        cv2.putText(vis, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    return vis


def log_row(path, box_c, box, board):
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["time", "box_x", "box_y", "box_median_mm", "box_p25_mm", "box_p75_mm",
                        "box_valid", "board_size_mm", "board_stereo_mm", "board_ratio"])
        med, p25, p75, valid = box
        b = board or {}
        w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), box_c[0], box_c[1],
                    "" if med is None else f"{med:.1f}", "" if p25 is None else f"{p25:.1f}",
                    "" if p75 is None else f"{p75:.1f}", f"{valid:.2f}",
                    "" if b.get("pnp") is None else f"{b['pnp']:.1f}",
                    "" if b.get("stereo") is None else f"{b['stereo']:.1f}",
                    "" if b.get("ratio") is None else f"{b['ratio']:.4f}"])


def main():
    p = argparse.ArgumentParser(description="Calibrated stereo depth viewer")
    p.add_argument("--calib", default="~/stereo_calib/stereo_calib.yaml")
    p.add_argument("--disparities", type=int, default=96, help="multiple of 16")
    p.add_argument("--block", type=int, default=5)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--log", default="~/stereo_calib/depth_log.csv")
    p.add_argument("--images", nargs=2, metavar=("LEFT", "RIGHT"))
    args = p.parse_args()
    if args.disparities % 16:
        sys.exit("--disparities must be a multiple of 16")

    c = load_calib(os.path.expanduser(args.calib))
    d = Depth(c, args.disparities, args.block)
    W, H = c["width"], c["height"]
    box_c = [W // 2, H // 2]
    log_path = os.path.expanduser(args.log)

    if args.images:
        il, ir = cv2.imread(args.images[0]), cv2.imread(args.images[1])
        if il is None or ir is None:
            sys.exit("Could not read both images.")
        rl, gl, disp, z = d.process(il, ir)
        box = d.box_reading(z, *box_c)
        board = d.board_reading(gl, z)
        for t in readout_lines(box, board, args.disparities, d.fb):
            print(t)
        out = os.path.join(os.path.dirname(log_path), "depth_view_offline.png")
        cv2.imwrite(out, draw(rl, z, box_c, box, board, args.disparities, d.fb))
        print(f"Saved {out}")
        return

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from stereo_test import HINTS, Camera, check_gstreamer, gst_pipeline  # noqa: E402
    check_gstreamer()
    cams = []
    for sid in (c["left_sensor_id"], c["right_sensor_id"]):
        cam = Camera(sid, gst_pipeline(sid, c["cap_width"], c["cap_height"], args.fps,
                                       W, H, c["flip"]))
        print(f"sensor-id={sid}: {'opened' if cam.ok else 'FAILED to open'}")
        cams.append(cam)
    if not all(cam.ok for cam in cams):
        for cam in cams:
            cam.stop()
        sys.exit("Both cameras are needed." + HINTS)
    for cam in cams:
        cam.start()

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            box_c[0] = int(np.clip(x % W, BOX, W - BOX - 1))
            box_c[1] = int(np.clip(y, BOX, H - BOX - 1))

    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse)
    show_board = False
    want_log = False
    last_counts = (-1, -1)
    t_prev, fps = time.time(), 0.0
    print(f"Running. Depth from {args.calib}. Keys: click box | b board | c log | q quit")
    try:
        while True:
            (fl, _, nl), (fr, _, nr) = cams[0].latest(), cams[1].latest()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("b"):
                show_board = not show_board
            if key == ord("c"):
                want_log = True
            if fl is None or fr is None or (nl, nr) == last_counts:
                continue
            last_counts = (nl, nr)
            rl, gl, disp, z = d.process(fl, fr)
            box = d.box_reading(z, *box_c)
            board = d.board_reading(gl, z) if show_board else None
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-3)
            t_prev = now
            if want_log:
                want_log = False
                log_row(log_path, box_c, box, board)
                print("logged: " + " | ".join(readout_lines(box, board, args.disparities, d.fb)[:2]))
            cv2.imshow(WIN, draw(rl, z, box_c, box, board, args.disparities, d.fb, fps))
    except KeyboardInterrupt:
        pass
    finally:
        for cam in cams:
            cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
