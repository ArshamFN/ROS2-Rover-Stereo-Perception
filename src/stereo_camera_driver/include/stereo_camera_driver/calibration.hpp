// Copyright (c) 2026 Arsham Faghihnasiri. MIT License.
#ifndef STEREO_CAMERA_DRIVER__CALIBRATION_HPP_
#define STEREO_CAMERA_DRIVER__CALIBRATION_HPP_

#include <string>

#include "sensor_msgs/msg/camera_info.hpp"

namespace stereo_camera_driver
{

// One stereo calibration as written by tools/stereo_calib/stereo_calibrate.py:
// OpenCV FileStorage YAML, distances in millimetres.
struct StereoCalibration
{
  int left_sensor_id = 0;
  int right_sensor_id = 1;
  int flip = 0;          // nvvidconv flip-method used for the calibration
  int cap_width = 0;     // sensor mode size
  int cap_height = 0;
  int width = 0;         // output size the calibration is valid for
  int height = 0;
  sensor_msgs::msg::CameraInfo left;   // ROS units: the projection's translation in metres
  sensor_msgs::msg::CameraInfo right;
  double baseline_m = 0.0;             // from the right camera's projection matrix
};

// Reads the file and converts it to CameraInfo (K, D as plumb_bob, R1/R2, P1/P2 with the
// translation terms divided by 1000). Throws std::runtime_error with the reason if the file
// is missing, malformed, or describes an impossible camera.
StereoCalibration load_calibration(const std::string & path);

}  // namespace stereo_camera_driver

#endif  // STEREO_CAMERA_DRIVER__CALIBRATION_HPP_
