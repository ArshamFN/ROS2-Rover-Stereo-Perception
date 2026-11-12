#!/usr/bin/env python3
"""
stereo_test.py

Acceptance test for the Waveshare IMX219-83 stereo camera on the Jetson Orin Nano.

Window 1 "Stereo: side by side": both live feeds with per-camera FPS, sharpness,
brightness and colour means, plus horizontal guide lines for checking vertical alignment.
Window 2 "Stereo: disparity": StereoSGBM disparity map from the two feeds, with a rough
centre distance estimate from the spec-sheet baseline and FOV. The pair is NOT calibrated
or rectified, so this window is a sanity check, not a pass/fail criterion.

Keys:
  q / Esc   quit
  s         swap left and right
  g         toggle guide lines
  c         save a capture (raw left, raw right, side by side, disparity)
            to ~/stereo_test_captures/
"""

import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np

WIN_SBS = "Stereo: side by side"
WIN_DISP = "Stereo: disparity"
TB_NDISP = "numDisp x16"
TB_BLOCK = "blockSize"

BASELINE_M = 0.060      # spec sheet baseline
HFOV_DEG = 73.0         # spec sheet horizontal FOV
STALL_S = 1.0           # no new frame for this long = STALLED
CMAP = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)

HINTS = """
Troubleshooting:
  1. ls /dev/video*                     expect video0 and video1
  2. sudo dmesg | grep imx219           both sensors should bind with no probe errors
  3. sudo /opt/nvidia/jetson-io/jetson-io.py
       -> Configure Jetson 24pin CSI Connector -> Camera IMX219 Dual -> save and reboot
  4. sudo systemctl restart nvargus-daemon
       a camera that worked on a previous run and now fails is usually the Argus daemon
  5. Swap the ribbon cables between CAM0 and CAM1 (power off first).
       Failure follows the camera  -> suspect the module or its cable.
       Failure stays on the port   -> suspect config or the Jetson side.
"""


def parse_args():
    p = argparse.ArgumentParser(description="IMX219-83 stereo camera test")
    p.add_argument("--left", type=int, default=0, help="sensor-id used as the left camera")
    p.add_argument("--right", type=int, default=1, help="sensor-id used as the right camera")
    p.add_argument("--cap-width", type=int, default=1640, help="sensor mode width")
    p.add_argument("--cap-height", type=int, default=1232, help="sensor mode height")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--width", type=int, default=640, help="per-camera output width")
    p.add_argument("--height", type=int, default=480, help="per-camera output height")
    p.add_argument("--flip", type=int, default=0,
                   help="nvvidconv flip-method: 0 none, 2 rotate 180 (module mounted upside down)")
    return p.parse_args()


def gst_pipeline(sensor_id, cap_w, cap_h, fps, out_w, out_h, flip):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={cap_w}, height={cap_h}, "
        f"framerate={fps}/1, format=NV12 ! "
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw, width={out_w}, height={out_h}, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


def check_gstreamer():
    print(f"OpenCV {cv2.__version__} loaded from {cv2.__file__}")
    for line in cv2.getBuildInformation().splitlines():
        if "GStreamer" in line:
            if "YES" in line:
                return
            break
    print("\nERROR: this OpenCV build has no GStreamer support, so it cannot open CSI cameras.")
    print("This is a software problem, not the camera. A pip-installed opencv-python")
    print("(usually in ~/.local) is shadowing the JetPack OpenCV. Check the path above.")
    sys.exit(1)


class Camera:
    """Grabs frames on its own thread so both sensors are read concurrently."""

    def __init__(self, sensor_id, pipeline):
        self.sensor_id = sensor_id
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        self.ok = self.cap.isOpened()
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = 0.0
        self.count = 0
        self.read_failures = 0
        self.max_gap = 0.0
        self.running = False
        self.thread = None

    def start(self):
        if not self.ok:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            ok, img = self.cap.read()
            if not ok or img is None:
                self.read_failures += 1
                time.sleep(0.01)
                continue
            now = time.monotonic()
            with self.lock:
                if self.count > 10:  # ignore startup gaps
                    self.max_gap = max(self.max_gap, now - self.stamp)
                self.frame = img
                self.stamp = now
                self.count += 1

    def latest(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            return frame, self.stamp, self.count

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.cap.release()


def put(img, text, org, color=(255, 255, 255), scale=0.5):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def no_signal(w, h, text):
    img = np.zeros((h, w, 3), np.uint8)
    put(img, text, (20, h // 2), (0, 0, 255), 0.6)
    return img


def frame_stats(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
    b, g, r = (float(c) for c in cv2.mean(img)[:3])
    return gray, sharpness, (b, g, r)


def build_matcher(num_disp, block):
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block,
        P1=8 * block * block,
        P2=32 * block * block,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def colorize(disp, num_disp):
    valid = disp > 0
    vis = np.zeros(disp.shape, np.uint8)
    vis[valid] = np.clip(disp[valid] * (255.0 / num_disp), 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(vis, CMAP)
    color[~valid] = 0
    return color, valid


def center_depth(disp, f_px, r=10):
    h, w = disp.shape
    patch = disp[h // 2 - r:h // 2 + r + 1, w // 2 - r:w // 2 + r + 1]
    good = patch[patch > 0]
    if good.size < 20:
        return None
    return f_px * BASELINE_M / float(np.median(good))


def save_capture(raws, sbs, disp_color):
    out_dir = os.path.expanduser("~/stereo_test_captures")
    os.makedirs(out_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    for name, img in (("left", raws[0]), ("right", raws[1])):
        if img is not None:
            cv2.imwrite(os.path.join(out_dir, f"{ts}_{name}.png"), img)
    cv2.imwrite(os.path.join(out_dir, f"{ts}_sbs.png"), sbs)
    cv2.imwrite(os.path.join(out_dir, f"{ts}_disparity.png"), disp_color)
    print(f"Saved capture {ts} to {out_dir}")


def main():
    args = parse_args()
    if args.left == args.right:
        sys.exit("--left and --right must be different sensor-ids")
    check_gstreamer()

    W, H = args.width, args.height
    cams = {}
    for sid in (args.left, args.right):
        pipe = gst_pipeline(sid, args.cap_width, args.cap_height, args.fps, W, H, args.flip)
        cam = Camera(sid, pipe)
        print(f"sensor-id={sid}: {'opened' if cam.ok else 'FAILED to open'}")
        cams[sid] = cam

    if not any(c.ok for c in cams.values()):
        print("\nNo camera opened." + HINTS)
        sys.exit(1)
    if not all(c.ok for c in cams.values()):
        print("\nOne camera failed to open. Continuing with the other." + HINTS)

    for c in cams.values():
        c.start()

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if all(c.latest()[0] is not None for c in cams.values() if c.ok):
            break
        time.sleep(0.05)
    for sid, c in cams.items():
        if c.ok and c.latest()[0] is None:
            print(f"sensor-id={sid} opened but delivered no frames in 5 s." + HINTS)

    cv2.namedWindow(WIN_SBS, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(WIN_DISP, cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar(TB_NDISP, WIN_DISP, 4, 12, lambda v: None)
    cv2.createTrackbar(TB_BLOCK, WIN_DISP, 5, 21, lambda v: None)

    left_id, right_id = args.left, args.right
    show_guides = True
    windows_seen = False
    matcher, last_params = None, None
    f_px = (W / 2.0) / np.tan(np.radians(HFOV_DEG / 2.0))

    fps = {sid: 0.0 for sid in cams}
    last_counts = {sid: cams[sid].latest()[2] for sid in cams}
    t_fps = time.monotonic()
    loop_n, loop_fps = 0, 0.0
    t_start = time.monotonic()
    start_counts = dict(last_counts)

    print("\nRunning. Keys: q quit | s swap L/R | g guides | c save capture")
    try:
        while True:
            now = time.monotonic()
            loop_n += 1
            if now - t_fps >= 1.0:
                for sid, c in cams.items():
                    n = c.latest()[2]
                    fps[sid] = (n - last_counts[sid]) / (now - t_fps)
                    last_counts[sid] = n
                loop_fps = loop_n / (now - t_fps)
                loop_n = 0
                t_fps = now

            views, grays, raws = [], [], []
            for side, sid in (("LEFT", left_id), ("RIGHT", right_id)):
                frame, stamp, _ = cams[sid].latest()
                if frame is None:
                    views.append(no_signal(W, H, f"{side} sensor-id={sid}: NO SIGNAL"))
                    grays.append(None)
                    raws.append(None)
                    continue
                raws.append(frame.copy())
                gray, sharp, (b, g, r) = frame_stats(frame)
                grays.append(gray)
                stalled = (now - stamp) > STALL_S
                col = (0, 0, 255) if stalled else (0, 255, 0)
                put(frame, f"{side}  sensor-id={sid}  {fps[sid]:.1f} fps"
                    + ("  STALLED" if stalled else ""), (10, 22), col)
                put(frame, f"sharpness {sharp:.0f}   brightness {(b + g + r) / 3:.0f}", (10, 44))
                put(frame, f"mean B/G/R {b:.0f}/{g:.0f}/{r:.0f}", (10, 66))
                views.append(frame)

            sbs = np.hstack(views)
            if show_guides:
                for y in range(H // 8, H, H // 8):
                    cv2.line(sbs, (0, y), (2 * W, y), (200, 200, 200), 1)
            cv2.line(sbs, (W, 0), (W, H), (255, 255, 255), 2)

            nd = max(1, cv2.getTrackbarPos(TB_NDISP, WIN_DISP)) * 16
            bs = max(3, cv2.getTrackbarPos(TB_BLOCK, WIN_DISP)) | 1
            if (nd, bs) != last_params:
                matcher = build_matcher(nd, bs)
                last_params = (nd, bs)

            if grays[0] is not None and grays[1] is not None:
                disp = matcher.compute(grays[0], grays[1]).astype(np.float32) / 16.0
                disp_color, valid = colorize(disp, nd)
                z = center_depth(disp, f_px)
                cx, cy = W // 2, H // 2
                cv2.drawMarker(disp_color, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
                put(disp_color, f"numDisp {nd}  block {bs}  valid {100.0 * valid.mean():.0f}%"
                    f"  {loop_fps:.1f} fps", (10, 22))
                put(disp_color, "centre: no match" if z is None
                    else f"centre ~{z:.2f} m (uncalibrated estimate)", (10, 44))
                put(disp_color, "warm = near, cool = far, black = no match", (10, H - 12))
            else:
                disp_color = no_signal(W, H, "Disparity needs both cameras")

            cv2.imshow(WIN_SBS, sbs)
            cv2.imshow(WIN_DISP, disp_color)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                left_id, right_id = right_id, left_id
                print(f"Swapped: left=sensor-id {left_id}, right=sensor-id {right_id}")
            elif key == ord("g"):
                show_guides = not show_guides
            elif key == ord("c"):
                save_capture(raws, sbs, disp_color)

            # Some OpenCV/GTK builds report -1 (unsupported) or 0 before the window is
            # mapped, so a window only counts as closed after it has been seen open.
            vis_sbs = cv2.getWindowProperty(WIN_SBS, cv2.WND_PROP_VISIBLE)
            vis_disp = cv2.getWindowProperty(WIN_DISP, cv2.WND_PROP_VISIBLE)
            if vis_sbs >= 1 and vis_disp >= 1:
                windows_seen = True
            elif windows_seen:
                break
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = max(time.monotonic() - t_start, 1e-6)
        for c in cams.values():
            c.stop()
        cv2.destroyAllWindows()
        print(f"\nSummary over {elapsed:.0f} s")
        for sid, c in cams.items():
            n = c.count - start_counts[sid]
            print(f"  sensor-id={sid}: opened={c.ok}  frames={n}  avg {n / elapsed:.1f} fps  "
                  f"read failures={c.read_failures}  longest frame gap={c.max_gap * 1000:.0f} ms")


if __name__ == "__main__":
    main()
