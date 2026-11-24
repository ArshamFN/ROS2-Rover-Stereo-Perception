# Stereo camera tools

Python scripts to test, calibrate, and check the IMX219-83 stereo camera on the Jetson.
The Session 001 scripts run on the Jetson itself, outside ROS2, and import each other,
so they stay together in this folder. `lidar_camera_yaw.py` (Session 002) is a ROS2
script.

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
| `lidar_camera_yaw.py` | ROS2. Finds a flat target of known width in the LiDAR scan and in the stereo point cloud, and reports the camera yaw that makes the two agree. See the last section. |

## Requirements

- JetPack's OpenCV, which is built with GStreamer. A pip-installed `opencv-python` cannot
  open the CSI cameras; `stereo_test.py` checks for this.
- numpy
- `smbus2` for `imu_test.py`: `pip3 install smbus2`
- ROS2 Humble with `rclpy` and `tf2_ros` for `lidar_camera_yaw.py`

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

## Camera yaw against the LiDAR

`lidar_camera_yaw.py` checks the camera yaw in the URDF against the LiDAR. Put a flat,
textured target of known width upright in front of the rover, 0.5 to 1 m away, with
nothing beside it at the same distance. A laptop screen showing a random pattern works.
With bringup and the stereo pipeline running and the rover standing still, in a shell
with this workspace sourced:

    python3 tools/stereo_calib/lidar_camera_yaw.py --width 0.365

It reports where each sensor puts the target (forward distance, sideways offset, angle,
width) and the camera yaw two ways: from the position of the target seen from the
camera, and from the angle of its surface. Use the position yaw, and run it at two
distances. A position yaw that changes with distance points to the mounting position of
the camera in the URDF. A surface angle difference that grows with distance points to
the stereo calibration. At 0.5 m the stereo pipeline needs `disparity_range:=128`.
