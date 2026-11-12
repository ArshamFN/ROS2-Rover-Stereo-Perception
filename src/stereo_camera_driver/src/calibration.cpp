// Copyright (c) 2026 Arsham Faghihnasiri. MIT License.
#include "stereo_camera_driver/calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace stereo_camera_driver
{
namespace
{

// OpenCV starts its YAML files with "%YAML:1.0", which is not a valid YAML directive.
std::string read_without_opencv_directive(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open calibration file " + path);
  }
  std::ostringstream out;
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (first && line.rfind("%YAML", 0) == 0) {
      first = false;
      continue;
    }
    first = false;
    out << line << '\n';
  }
  return out.str();
}

int read_int(const YAML::Node & root, const std::string & key)
{
  const YAML::Node n = root[key];
  if (!n) {
    throw std::runtime_error("calibration: missing " + key);
  }
  return n.as<int>();
}

std::vector<double> read_matrix(
  const YAML::Node & root, const std::string & key, int rows, int cols)
{
  const YAML::Node m = root[key];
  if (!m || !m.IsMap() || !m["rows"] || !m["cols"] || !m["data"] || !m["data"].IsSequence()) {
    throw std::runtime_error("calibration: missing or malformed matrix " + key);
  }
  const int r = m["rows"].as<int>();
  const int c = m["cols"].as<int>();
  const YAML::Node data = m["data"];
  if (r * c != rows * cols || static_cast<int>(data.size()) != rows * cols) {
    throw std::runtime_error(
            "calibration: " + key + " is " + std::to_string(r) + "x" + std::to_string(c) +
            ", expected " + std::to_string(rows) + "x" + std::to_string(cols));
  }
  std::vector<double> v;
  v.reserve(data.size());
  for (const auto & x : data) {
    const double d = x.as<double>();
    if (!std::isfinite(d)) {
      throw std::runtime_error("calibration: " + key + " holds a value that is not finite");
    }
    v.push_back(d);
  }
  return v;
}

sensor_msgs::msg::CameraInfo camera_info(
  int width, int height, const std::vector<double> & k, const std::vector<double> & d,
  const std::vector<double> & r, const std::vector<double> & p)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.width = static_cast<uint32_t>(width);
  ci.height = static_cast<uint32_t>(height);
  ci.distortion_model = "plumb_bob";
  ci.d = d;
  for (int i = 0; i < 9; ++i) {
    ci.k[i] = k[i];
    ci.r[i] = r[i];
  }
  for (int i = 0; i < 12; ++i) {
    ci.p[i] = p[i];
  }
  // The projection's last column is focal length times a distance: millimetres in the file,
  // metres in ROS. Missing this gives depth 1000 times off without any error.
  ci.p[3] /= 1000.0;
  ci.p[7] /= 1000.0;
  ci.p[11] /= 1000.0;
  return ci;
}

void check_camera(const sensor_msgs::msg::CameraInfo & ci, const std::string & name)
{
  const double w = ci.width;
  const double h = ci.height;
  if (!(ci.k[0] > 0.0 && ci.k[4] > 0.0 && ci.p[0] > 0.0 && ci.p[5] > 0.0)) {
    throw std::runtime_error("calibration: " + name + " camera has a focal length that is not positive");
  }
  if (!(ci.k[2] > 0.0 && ci.k[2] < w && ci.k[5] > 0.0 && ci.k[5] < h &&
    ci.p[2] > 0.0 && ci.p[2] < w && ci.p[6] > 0.0 && ci.p[6] < h))
  {
    throw std::runtime_error("calibration: " + name + " camera's principal point is outside the image");
  }
}

}  // namespace

StereoCalibration load_calibration(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::Load(read_without_opencv_directive(path));
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("calibration file " + path + " is not valid YAML: " + e.what());
  }
  StereoCalibration c;
  try {
    c.left_sensor_id = read_int(root, "left_sensor_id");
    c.right_sensor_id = read_int(root, "right_sensor_id");
    c.flip = read_int(root, "flip");
    c.cap_width = read_int(root, "cap_width");
    c.cap_height = read_int(root, "cap_height");
    c.width = read_int(root, "width");
    c.height = read_int(root, "height");
    if (c.width <= 0 || c.height <= 0 || c.cap_width <= 0 || c.cap_height <= 0) {
      throw std::runtime_error("calibration: image sizes must be positive");
    }
    if (c.left_sensor_id == c.right_sensor_id) {
      throw std::runtime_error("calibration: left and right use the same sensor-id");
    }
    c.left = camera_info(
      c.width, c.height, read_matrix(root, "K1", 3, 3), read_matrix(root, "D1", 1, 5),
      read_matrix(root, "R1", 3, 3), read_matrix(root, "P1", 3, 4));
    c.right = camera_info(
      c.width, c.height, read_matrix(root, "K2", 3, 3), read_matrix(root, "D2", 1, 5),
      read_matrix(root, "R2", 3, 3), read_matrix(root, "P2", 3, 4));
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("calibration file " + path + ": " + e.what());
  }
  check_camera(c.left, "left");
  check_camera(c.right, "right");
  // Right camera: P[3] = -fx * baseline. A positive value means left and right are swapped.
  c.baseline_m = -c.right.p[3] / c.right.p[0];
  if (!(c.baseline_m > 0.0)) {
    throw std::runtime_error(
            "calibration: the right camera's baseline is not positive (left and right swapped?)");
  }
  return c;
}

}  // namespace stereo_camera_driver
