// Copyright (c) 2026 Arsham Faghihnasiri. MIT License.
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "stereo_camera_driver/calibration.hpp"

using stereo_camera_driver::load_calibration;

namespace
{

const std::string fixture = std::string(TEST_DATA_DIR) + "/calibration_fixture.yaml";

std::string read(const std::string & path)
{
  std::ifstream in(path);
  std::ostringstream s;
  s << in.rdbuf();
  return s.str();
}

// The fixture with one piece of text replaced, written to a temporary file.
std::string variant(const std::string & from, const std::string & to)
{
  std::string text = read(fixture);
  const auto pos = text.find(from);
  EXPECT_NE(pos, std::string::npos) << from;
  text.replace(pos, from.size(), to);
  char name[] = "/tmp/stereo_calib_testXXXXXX";
  const int fd = mkstemp(name);
  EXPECT_GE(fd, 0);
  std::FILE * f = fdopen(fd, "w");
  std::fputs(text.c_str(), f);
  std::fclose(f);
  return name;
}

}  // namespace

TEST(Calibration, ReadsOpenCvFileAndConvertsToMetres)
{
  const auto c = load_calibration(fixture);
  EXPECT_EQ(c.left_sensor_id, 0);
  EXPECT_EQ(c.right_sensor_id, 1);
  EXPECT_EQ(c.cap_width, 1640);
  EXPECT_EQ(c.width, 640);
  EXPECT_EQ(c.left.width, 640u);
  EXPECT_EQ(c.left.height, 480u);
  EXPECT_EQ(c.left.distortion_model, "plumb_bob");
  ASSERT_EQ(c.left.d.size(), 5u);
  EXPECT_DOUBLE_EQ(c.left.d[0], -0.03);
  EXPECT_DOUBLE_EQ(c.left.k[0], 420.0);
  EXPECT_DOUBLE_EQ(c.right.k[5], 270.0);
  EXPECT_DOUBLE_EQ(c.left.p[3], 0.0);
  EXPECT_DOUBLE_EQ(c.right.p[3], -30.0);   // -30000 mm px in the file
  EXPECT_DOUBLE_EQ(c.baseline_m, 0.06);
}

TEST(Calibration, MissingFileIsAnError)
{
  EXPECT_THROW(load_calibration("/nonexistent/stereo_calib.yaml"), std::runtime_error);
}

TEST(Calibration, SwappedCamerasAreAnError)
{
  EXPECT_THROW(load_calibration(variant("-3.0e+04", "3.0e+04")), std::runtime_error);
}

TEST(Calibration, WrongMatrixSizeIsAnError)
{
  EXPECT_THROW(
    load_calibration(variant("data: [ -3.0e-02, 1.8e-02, -4.0e-03, 5.0e-03, 0. ]",
    "data: [ -3.0e-02, 1.8e-02, -4.0e-03, 5.0e-03 ]")),
    std::runtime_error);
}

TEST(Calibration, MissingKeyIsAnError)
{
  EXPECT_THROW(load_calibration(variant("cap_width: 1640", "capture_width: 1640")), std::runtime_error);
}

TEST(Calibration, PrincipalPointOutsideImageIsAnError)
{
  EXPECT_THROW(
    load_calibration(variant("420., 0., 320.", "420., 0., 700.")), std::runtime_error);
}
