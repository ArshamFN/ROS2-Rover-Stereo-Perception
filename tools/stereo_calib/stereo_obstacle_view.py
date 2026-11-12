#!/usr/bin/env python3
"""
stereo_obstacle_view.py

Height above the floor for every stereo point, live. Each point is classed by its
height above a fixed floor plane:
  red      30 to 180 mm: obstacles the lidar cannot see
  yellow   above 180 mm: the lidar's job
  green    floor, within 30 mm
  blue     more than 30 mm below the floor: reflections and mismatches (dropped)
The floor plane is read from ~/stereo_calib/floor_plane.yaml (measured once per
mount); the stereo calibration from ~/stereo_calib/stereo_calib.yaml.

Matching: SGBM 3WAY on the rectified pair, at half resolution (320 x 240, 48
disparities, block 5) by default or full resolution (640 x 480, 96 disparities,
block 7); key h switches. Half resolution is several times cheaper and doubles
the depth noise. With floor fill on (key w), a second pass matches the right
image after shifting it by the disparity the floor should have there. That fills
most of the floor the first pass leaves empty. The first pass wins wherever it
has a value, so objects keep their first-pass depth.

  python3 stereo_obstacle_view.py
  python3 stereo_obstacle_view.py --images LEFT.png RIGHT.png
      (one raw pair, as saved by stereo_test.py; runs every combination)

Keys (live):
  w         floor fill on/off
  h         half / full resolution matching
  s         save the raw pair and the view to ~/stereo_calib/obstacle_captures/
  q / Esc   quit
Every 3 s the terminal prints fps and the time spent in each stage.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stereo_depth_view import colour_depth, load_calib  # noqa: E402

WIN = "Stereo obstacles (height above floor)"
BELOW, FLOOR_TOL, BAND_TOP = -30.0, 30.0, 180.0     # mm, class limits
Z_NEAR, Z_FAR = 250.0, 4000.0                        # mm, depth kept
MIN_BLOB = 300        # px; smaller obstacle-band blobs are ignored
SHOW_WITHIN = 2500.0  # mm; obstacle blobs boxed and listed up to this distance
DEPTH_JUMP = 0.08     # neighbouring pixels more than 8% apart in depth belong to different blobs
COLS = {1: (255, 80, 0), 2: (0, 200, 0), 3: (0, 0, 255), 4: (0, 220, 255)}
NAMES = {3: "30 to 180 mm: stereo obstacle", 4: "above 180 mm: lidar sees these",
         2: "floor, within 30 mm", 1: "below floor: dropped"}


def load_floor(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        sys.exit(f"No floor plane at {path}")
    n = fs.getNode("floor_normal").mat().ravel().astype(np.float64)
    h = fs.getNode("camera_height_mm").real()
    fs.release()
    return n / np.linalg.norm(n), h


class Obstacles:
    def __init__(self, c, normal_cam, height, block, num_disp, scale=1):
        W, H = c["width"], c["height"]
        self.size, self.scale = (W, H), scale
        self.m1 = cv2.initUndistortRectifyMap(c["K1"], c["D1"], c["R1"], c["P1"], (W, H), cv2.CV_16SC2)
        self.m2 = cv2.initUndistortRectifyMap(c["K2"], c["D2"], c["R2"], c["P2"], (W, H), cv2.CV_16SC2)
        # matching grid: the rectified image shrunk by `scale` (pixel centres move by (scale - 1) / 2)
        off = (scale - 1) / (2.0 * scale)
        f = float(c["P1"][0, 0]) / scale
        cx, cy = float(c["P1"][0, 2]) / scale - off, float(c["P1"][1, 2]) / scale - off
        self.fb = float(-c["P2"][0, 3]) / scale
        self.h0 = height
        self.num_disp = num_disp
        n = c["R1"] @ normal_cam                     # floor normal in the rectified left frame
        uu, vv = np.meshgrid(np.arange(W // scale, dtype=np.float32), np.arange(H // scale, dtype=np.float32))
        # a pixel's point is Z * (ax, ay, 1); its depth below the camera along n is Z * s
        self.s = (((uu - cx) / f) * n[0] + ((vv - cy) / f) * n[1] + n[2]).astype(np.float32)
        # disparity the floor has at each pixel (0 at and above the horizon)
        self.dfl = np.where(self.s > 0, self.fb * self.s / height, 0).astype(np.float32)
        self.wx, self.wy = uu - self.dfl, vv

        def sgbm(min_d, n_d):
            return cv2.StereoSGBM_create(
                minDisparity=min_d, numDisparities=n_d, blockSize=block,
                P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
                uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.plain = sgbm(0, num_disp)
        self.fill_min = -16 // scale
        self.fill = sgbm(self.fill_min, num_disp + 16)

    def process(self, left_bgr, right_bgr, use_fill):
        t0 = time.perf_counter()
        rl = cv2.remap(left_bgr, *self.m1, cv2.INTER_LINEAR)
        rr = cv2.remap(right_bgr, *self.m2, cv2.INTER_LINEAR)
        gl = cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        if self.scale > 1:
            small = (self.size[0] // self.scale, self.size[1] // self.scale)
            gl = cv2.resize(gl, small, interpolation=cv2.INTER_AREA)
            gr = cv2.resize(gr, small, interpolation=cv2.INTER_AREA)
        t1 = time.perf_counter()
        d = self.plain.compute(gl, gr).astype(np.float32) / 16.0
        t2 = time.perf_counter()
        if use_fill:
            shifted = cv2.remap(gr, self.wx, self.wy, cv2.INTER_LINEAR)
            r = self.fill.compute(gl, shifted).astype(np.float32) / 16.0
            d = np.where(d > 0.5, d, np.where(r > self.fill_min - 0.5, r + self.dfl, -1.0))
        t3 = time.perf_counter()
        valid = d > 0.5
        valid[:, :self.num_disp] = False
        z = self.fb / np.where(valid, d, 1.0)
        valid &= (z > Z_NEAR) & (z < Z_FAR)
        h = self.h0 - z * self.s
        cls = np.zeros(d.shape, np.uint8)
        cls[valid & (h < BELOW)] = 1
        cls[valid & (np.abs(h) <= FLOOR_TOL)] = 2
        cls[valid & (h > FLOOR_TOL) & (h <= BAND_TOP)] = 3
        cls[valid & (h > BAND_TOP)] = 4
        z = np.where(valid, z, np.nan).astype(np.float32)
        if self.scale > 1:                           # back to the display size for drawing and blobs
            z = cv2.resize(z, self.size, interpolation=cv2.INTER_NEAREST)
            h = cv2.resize(h.astype(np.float32), self.size, interpolation=cv2.INTER_NEAREST)
            cls = cv2.resize(cls, self.size, interpolation=cv2.INTER_NEAREST)
        blobs = self.band_blobs(cls, z, h)
        t4 = time.perf_counter()
        ms = {"rectify": t1 - t0, "match": t2 - t1, "fill": t3 - t2, "classify": t4 - t3}
        return rl, z, cls, blobs, {k: 1000 * v for k, v in ms.items()}

    @staticmethod
    def band_blobs(cls, z, h):
        # cut the obstacle band at depth jumps, so a near object and a far wall base that touch
        # in the image stay separate blobs
        zz = np.nan_to_num(z, nan=0.0)
        jump = np.zeros(zz.shape, bool)
        jump[:, 1:] |= np.abs(zz[:, 1:] - zz[:, :-1]) > DEPTH_JUMP * np.maximum(zz[:, 1:], 1.0)
        jump[1:, :] |= np.abs(zz[1:, :] - zz[:-1, :]) > DEPTH_JUMP * np.maximum(zz[1:, :], 1.0)
        band = ((cls == 3) & ~jump).astype(np.uint8)
        band = cv2.morphologyEx(band, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(band, connectivity=8)
        out = []
        for i in range(1, n):
            x, y, w, hh, area = stats[i]
            if area < MIN_BLOB:
                continue
            m = lab[y:y + hh, x:x + w] == i
            zi = z[y:y + hh, x:x + w][m]
            near = float(np.nanpercentile(zi, 10))
            if near > SHOW_WITHIN:
                continue
            top = float(np.nanpercentile(h[y:y + hh, x:x + w][m], 98))
            out.append({"near": near, "top": top, "box": (int(x), int(y), int(w), int(hh)), "area": int(area)})
        return sorted(out, key=lambda b: b["near"])


def put(img, text, org, colour=(255, 255, 255), scale=0.5):
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = org
    cv2.rectangle(img, (x - 3, y - th - 4), (x + tw + 3, y + base + 2), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 1, cv2.LINE_AA)


def draw(rl, z, cls, blobs, use_fill, fps, ms, res="full"):
    W = rl.shape[1]
    over = rl.copy()
    for k in (1, 2, 3, 4):
        m = cls == k
        over[m] = (0.45 * over[m] + 0.55 * np.array(COLS[k])).astype(np.uint8)
    for b in blobs:
        x, y, w, h = b["box"]
        cv2.rectangle(over, (x, y), (x + w, y + h), (255, 255, 255), 1)
        top = "180+" if b["top"] >= BAND_TOP - 5 else f"{b['top']:.0f}"
        put(over, f"{b['near'] / 1000:.2f} m, top {top} mm", (x + 3, max(14, y - 4)), scale=0.42)
    y = 18
    for k in (3, 4, 2, 1):
        cv2.rectangle(over, (4, y - 13), (16, y - 1), COLS[k], -1)
        put(over, NAMES[k], (22, y), scale=0.42)
        y += 18
    vis = np.hstack([over, colour_depth(z)])
    cv2.line(vis, (W, 0), (W, vis.shape[0]), (255, 255, 255), 2)
    lines = [f"{res} res (h)   floor fill {'ON' if use_fill else 'off'} (w)   " + (f"{fps:.1f} fps" if fps else ""),
             "ms: rectify {rectify:.0f}  match {match:.0f}  fill {fill:.0f}  classify {classify:.0f}".format(**ms),
             f"nearest stereo obstacle: {blobs[0]['near'] / 1000:.2f} m" if blobs else "no stereo obstacle within 2.5 m"]
    for i, t in enumerate(lines):
        put(vis, t, (W + 10, 20 + 20 * i), (0, 255, 255))
    return vis


def summary(cls, blobs):
    valid = cls > 0
    tot = max(1, int(valid.sum()))
    parts = [f"{NAMES[k].split(':')[0]} {100 * (cls == k).sum() / tot:.1f}%" for k in (3, 4, 2, 1)]
    lines = [f"valid {100 * valid.mean():.1f}% of the image; of those: " + ", ".join(parts)]
    low = cls[cls.shape[0] * 3 // 4:]
    lines.append(f"bottom quarter of the image: floor {100 * (low == 2).mean():.1f}%, "
                 f"below floor {100 * (low == 1).mean():.1f}%, empty {100 * (low == 0).mean():.1f}%")
    for b in blobs:
        top = "180+" if b["top"] >= BAND_TOP - 5 else f"{b['top']:.0f}"
        lines.append(f"obstacle: nearest {b['near'] / 1000:.2f} m, top {top} mm, {b['area']} px, box {b['box']}")
    return lines


def main():
    p = argparse.ArgumentParser(description="Stereo height-above-floor viewer")
    p.add_argument("--calib", default="~/stereo_calib/stereo_calib.yaml")
    p.add_argument("--floor", default="~/stereo_calib/floor_plane.yaml")
    p.add_argument("--disparities", type=int, default=96, help="multiple of 16")
    p.add_argument("--block", type=int, default=7, help="block size at full resolution")
    p.add_argument("--half-block", type=int, default=5, help="block size at half resolution")
    p.add_argument("--full", action="store_true", help="start at full resolution")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--no-fill", action="store_true", help="start with floor fill off")
    p.add_argument("--images", nargs=2, metavar=("LEFT", "RIGHT"))
    args = p.parse_args()
    if args.disparities % 16:
        sys.exit("--disparities must be a multiple of 16")

    c = load_calib(os.path.expanduser(args.calib))
    normal, height = load_floor(os.path.expanduser(args.floor))
    obs = {"full": Obstacles(c, normal, height, args.block, args.disparities, 1),
           "half": Obstacles(c, normal, height, args.half_block, args.disparities // 2, 2)}
    if args.disparities // 2 % 16:
        sys.exit("--disparities must be a multiple of 32 (half resolution uses half as many)")
    out_dir = os.path.join(os.path.dirname(os.path.expanduser(args.calib)), "obstacle_captures")

    if args.images:
        il, ir = cv2.imread(args.images[0]), cv2.imread(args.images[1])
        if il is None or ir is None:
            sys.exit("Could not read both images.")
        os.makedirs(out_dir, exist_ok=True)
        for res in ("full", "half"):
            for use_fill in (False, True):
                rl, z, cls, blobs, ms = obs[res].process(il, ir, use_fill)
                print(f"--- {res} res, floor fill {'on' if use_fill else 'off'}: "
                      + "  ".join(f"{k} {v:.0f} ms" for k, v in ms.items()))
                for t in summary(cls, blobs):
                    print("  " + t)
                out = os.path.join(out_dir, f"offline_{res}_fill_{'on' if use_fill else 'off'}.png")
                cv2.imwrite(out, draw(rl, z, cls, blobs, use_fill, None, ms, res))
                print(f"  saved {out}")
        return

    from stereo_test import HINTS, Camera, check_gstreamer, gst_pipeline  # noqa: E402
    check_gstreamer()
    W, H = c["width"], c["height"]
    cams = []
    for sid in (c["left_sensor_id"], c["right_sensor_id"]):
        cam = Camera(sid, gst_pipeline(sid, c["cap_width"], c["cap_height"], args.fps, W, H, c["flip"]))
        print(f"sensor-id={sid}: {'opened' if cam.ok else 'FAILED to open'}")
        cams.append(cam)
    if not all(cam.ok for cam in cams):
        for cam in cams:
            cam.stop()
        sys.exit("Both cameras are needed." + HINTS)
    for cam in cams:
        cam.start()

    use_fill = not args.no_fill
    res = "full" if args.full else "half"
    last_counts = (-1, -1)
    t_prev, fps = time.time(), 0.0
    acc, n_acc, t_print = {}, 0, time.time()
    print(f"Running. Floor plane from {args.floor} (camera {height:.0f} mm up). Keys: w fill | h half/full | s save | q quit")
    try:
        while True:
            (fl, _, nl), (fr, _, nr) = cams[0].latest(), cams[1].latest()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("w"):
                use_fill = not use_fill
                acc, n_acc, t_print = {}, 0, time.time()
                print(f"floor fill {'on' if use_fill else 'off'}")
            if key == ord("h"):
                res = "full" if res == "half" else "half"
                acc, n_acc, t_print = {}, 0, time.time()
                print(f"{res} resolution")
            if fl is None or fr is None or (nl, nr) == last_counts:
                continue
            last_counts = (nl, nr)
            rl, z, cls, blobs, ms = obs[res].process(fl, fr, use_fill)
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-3)
            t_prev = now
            for k, v in ms.items():
                acc[k] = acc.get(k, 0.0) + v
            n_acc += 1
            if now - t_print > 3 and n_acc:
                print(f"{res} fill {'on ' if use_fill else 'off'}: {fps:4.1f} fps | "
                      + "  ".join(f"{k} {v / n_acc:3.0f} ms" for k, v in acc.items()))
                acc, n_acc, t_print = {}, 0, now
            vis = draw(rl, z, cls, blobs, use_fill, fps, ms, res)
            if key == ord("s"):
                os.makedirs(out_dir, exist_ok=True)
                stamp = time.strftime("%Y%m%d_%H%M%S")
                for name, img in (("left", fl), ("right", fr), ("view", vis)):
                    cv2.imwrite(os.path.join(out_dir, f"{stamp}_{name}.png"), img)
                print(f"saved {out_dir}/{stamp}_*.png ({res} res, fill {'on' if use_fill else 'off'})")
                for t in summary(cls, blobs):
                    print("  " + t)
            cv2.imshow(WIN, vis)
    except KeyboardInterrupt:
        pass
    finally:
        for cam in cams:
            cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
