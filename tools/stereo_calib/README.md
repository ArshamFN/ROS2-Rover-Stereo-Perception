# Stereo camera tools

Python scripts used in Session 001 to test, calibrate, and check the IMX219-83 stereo
camera on the Jetson. They run on the Jetson itself, outside ROS2, and import each other,
so they stay together in this folder.

| Script | What it does |
|---|---|
| `stereo_test.py` | Both live feeds side by side with frame rate, sharpness, and guide lines, plus an uncalibrated disparity map. The camera pipeline the other scripts import. |
| `stereo_offset.py` | Vertical offset and roll between the left and right images of captures saved by `stereo_test.py`. |
| `stereo_align_view.py` | Live vertical alignment check without a target: measures the offset from matched features and shows the depth map before and after shifting the right image. |
| `stereo_calib_capture.py` | Saves checkerboard pairs for calibration, automatically when the board is held still in a new pose. |
| `stereo_calibrate.py` | Calibrates each camera and then the pair, rectifies, reports the quality, and writes `stereo_calib.yaml`. `--check` tests an existing calibration on new pairs. |
| `stereo_pair_errors.py` | Rectified row error for each calibration pair, worst first. |
| `stereo_depth_view.py` | Calibrated depth, live or from a saved pair, with a measurement box and a checkerboard readout for checking against a tape measure. |
| `stereo_obstacle_view.py` | Height above the floor for every stereo point, live or from a saved pair. Obstacles from 3 to 18 cm tall, below the LiDAR's scan plane, show in red. |
| `imu_test.py` | Reads the ICM20948 on the camera board over I2C bus 7: identity, accelerometer, gyro, temperature, and magnetometer. |

## Requirements

- JetPack's OpenCV, which is built with GStreamer. A pip-installed `opencv-python` cannot
  open the CSI cameras; `stereo_test.py` checks for this.
- numpy
- `smbus2` for `imu_test.py`: `pip3 install smbus2`

## Data

The scripts keep their data outside the repository. `stereo_test.py` saves captures to
`~/stereo_test_captures/`. Calibration pairs, `stereo_calib.yaml`, `floor_plane.yaml`, and
the viewers' captures live in `~/stereo_calib/`. The floor plane was measured once, in
Session 001, with a checkerboard on a box top; there is no script for it yet.

## Order of use

```bash
python3 stereo_test.py
python3 stereo_calib_capture.py
python3 stereo_calibrate.py --square-mm 24.9
python3 stereo_pair_errors.py
python3 stereo_depth_view.py
python3 stereo_obstacle_view.py
```

Measure the printed squares with calipers and pass that size to `stereo_calibrate.py`.
