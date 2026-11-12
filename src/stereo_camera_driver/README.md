# stereo_camera_driver

ROS2 driver for a two-sensor CSI stereo camera on an NVIDIA Jetson, written for the
Waveshare IMX219-83 and free of anything rover-specific.

- Both sensors run in one GStreamer pipeline (`nvarguscamerasrc`, then `nvvidconv` to the
  calibrated output size), so their timestamps share one clock and can be compared.
- Each frame is stamped with the time its buffer carries, converted to ROS time.
- `camera_info` comes from a per-unit calibration file written by
  `tools/stereo_calib/stereo_calibrate.py` (OpenCV YAML in millimetres). The projection's
  translation terms are converted to metres.
- Left and right frames are not paired here: the sensors free-run, and a consumer such as
  `stereo_image_proc` pairs them by stamp (`approximate_sync`).
- No OpenCV and no cv_bridge: frames are copied from GStreamer straight into the message,
  and the calibration file is read with yaml-cpp. On a Jetson, NVIDIA's OpenCV and the one
  ROS packages are built against are different versions, and this keeps them apart.

## Topics

| Topic | Type |
|---|---|
| `left/image_raw`, `right/image_raw` | `sensor_msgs/Image`, `bgr8` or `mono8` |
| `left/camera_info`, `right/camera_info` | `sensor_msgs/CameraInfo`, same stamp as its image |

Published reliable, with a queue of 2.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `calibration_file` | none, required | the calibration file |
| `encoding` | `bgr8` | `bgr8` or `mono8` (`mono8` comes from `nvvidconv` directly, almost no CPU) |
| `sensor_mode` | `3` | Argus sensor mode; must be the mode the calibration was made in (`-1` lets Argus choose) |
| `fps` | `30` | frame rate |
| `left_frame_id` | `stereo_left_optical_frame` | the rectified left camera frame |
| `right_frame_id` | `stereo_right_optical_frame` | |
| `report_period` | `5.0` | seconds between `STATS` lines in the log |
| `source` | `argus` | `test` replaces both cameras with a test pattern, for checking the node without cameras |

Sensor ids, flip and image sizes come from the calibration file, so the driver cannot run the
cameras in a mode the calibration does not describe.

## The STATS line

Every `report_period` the node logs each camera's frame rate; the time from each frame's stamp
to its arrival in the node (min/median/max; a few ms means the stamp is the arrival time, tens
of ms means it is close to the capture time); and the gap between each left frame and the
nearest right frame (right minus left). The sensors are not synchronised in hardware, so the
gap stays roughly fixed while they run and changes every time they start.

## Run

```bash
ros2 launch stereo_camera_driver stereo_camera.launch.py calibration_file:=<path>
```

Arguments: `calibration_file`, `encoding`, `sensor_mode`, `fps`, `source`, `namespace`
(default `stereo`). The launch file logs the calibration file's sha256.
