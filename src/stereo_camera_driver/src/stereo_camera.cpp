// Copyright (c) 2026 Arsham Faghihnasiri. MIT License.
//
// Publishes both cameras of a free-running stereo pair. One GStreamer pipeline holds a
// branch per sensor, so both share one clock and one start time and their timestamps can be
// compared. Each frame is stamped with the time its buffer carries (converted from the
// pipeline clock to ROS time), and published with the camera_info from the calibration file.
// Left and right frames are not paired here; stereo_image_proc pairs them by stamp.

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_camera_driver/calibration.hpp"

namespace stereo_camera_driver
{

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

namespace
{

struct Summary
{
  double min = 0.0;
  double median = 0.0;
  double max = 0.0;
};

Summary summarize(std::vector<double> v)
{
  Summary s;
  if (v.empty()) {
    return s;
  }
  std::sort(v.begin(), v.end());
  s.min = v.front();
  s.max = v.back();
  s.median = v[v.size() / 2];
  return s;
}

std::string gst_error_text(GError * err)
{
  std::string text = err ? err->message : "unknown error";
  if (err) {
    g_error_free(err);
  }
  return text;
}

}  // namespace

class StereoCamera : public rclcpp::Node
{
public:
  explicit StereoCamera(const rclcpp::NodeOptions & options)
  : rclcpp::Node("stereo_camera", options)
  {
    const auto calibration_file = declare_parameter<std::string>("calibration_file", "");
    encoding_ = declare_parameter<std::string>("encoding", "bgr8");
    sensor_mode_ = declare_parameter<int>("sensor_mode", 3);
    fps_ = declare_parameter<int>("fps", 30);
    source_ = declare_parameter<std::string>("source", "argus");
    const auto left_frame = declare_parameter<std::string>("left_frame_id", "stereo_left_optical_frame");
    const auto right_frame = declare_parameter<std::string>("right_frame_id", "stereo_right_optical_frame");
    const double report_period = declare_parameter<double>("report_period", 5.0);

    if (calibration_file.empty()) {
      throw std::invalid_argument("parameter calibration_file is required (no default on purpose)");
    }
    if (encoding_ != "bgr8" && encoding_ != "mono8") {
      throw std::invalid_argument("parameter encoding must be bgr8 or mono8, not " + encoding_);
    }
    if (source_ != "argus" && source_ != "test") {
      throw std::invalid_argument("parameter source must be argus or test, not " + source_);
    }
    if (fps_ <= 0 || report_period <= 0.0) {
      throw std::invalid_argument("parameters fps and report_period must be positive");
    }

    calib_ = load_calibration(calibration_file);
    sides_[0].name = "left";
    sides_[1].name = "right";
    sides_[0].info = calib_.left;
    sides_[1].info = calib_.right;
    sides_[0].frame_id = left_frame;
    sides_[1].frame_id = right_frame;

    RCLCPP_INFO(
      get_logger(), "calibration %s: output %dx%d from sensor mode %dx%d, left sensor-id %d, "
      "right sensor-id %d, flip %d, rectified focal length %.3f px, baseline %.4f m",
      calibration_file.c_str(), calib_.width, calib_.height, calib_.cap_width, calib_.cap_height,
      calib_.left_sensor_id, calib_.right_sensor_id, calib_.flip, calib_.right.p[0],
      calib_.baseline_m);

    // Reliable with a short queue: a reliable publisher reaches both reliable and best-effort
    // subscribers; a best-effort one would reach no subscriber that asks for reliable.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    for (auto & side : sides_) {
      side.image_pub = create_publisher<Image>(side.name + "/image_raw", qos);
      side.info_pub = create_publisher<CameraInfo>(side.name + "/camera_info", qos);
    }

    start_pipeline();
    report_timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period), [this]() {report();});
    bus_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {poll_bus();});
  }

  ~StereoCamera() override
  {
    stop_pipeline();
  }

private:
  struct Side
  {
    std::string name;
    std::string frame_id;
    CameraInfo info;
    GstElement * sink = nullptr;
    rclcpp::Publisher<Image>::SharedPtr image_pub;
    rclcpp::Publisher<CameraInfo>::SharedPtr info_pub;
    std::thread thread;
    // statistics since the last report, guarded by stats_mutex_
    std::deque<int64_t> stamps;          // recent stamps (ns), for the left/right gap
    std::vector<double> latency_ms;      // arrival time minus stamp
    uint64_t frames = 0;
    uint64_t frames_total = 0;
    uint64_t without_timestamp = 0;
  };

  std::string branch(const Side & side, int sensor_id) const
  {
    const char * format = encoding_ == "bgr8" ? "BGRx" : "GRAY8";
    char buf[512];
    if (source_ == "test") {
      std::snprintf(
        buf, sizeof(buf),
        "videotestsrc is-live=true pattern=ball ! "
        "video/x-raw,width=%d,height=%d,framerate=%d/1,format=%s ! "
        "appsink name=%s drop=true max-buffers=2 sync=false",
        calib_.width, calib_.height, fps_, format, side.name.c_str());
    } else {
      const std::string mode =
        sensor_mode_ >= 0 ? " sensor-mode=" + std::to_string(sensor_mode_) : std::string();
      std::snprintf(
        buf, sizeof(buf),
        "nvarguscamerasrc sensor-id=%d%s ! "
        "video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1,format=NV12 ! "
        "nvvidconv flip-method=%d ! video/x-raw,width=%d,height=%d,format=%s ! "
        "appsink name=%s drop=true max-buffers=2 sync=false",
        sensor_id, mode.c_str(), calib_.cap_width, calib_.cap_height, fps_, calib_.flip,
        calib_.width, calib_.height, format, side.name.c_str());
    }
    return buf;
  }

  void start_pipeline()
  {
    GError * err = nullptr;
    if (!gst_init_check(nullptr, nullptr, &err)) {
      throw std::runtime_error("GStreamer init failed: " + gst_error_text(err));
    }
    const std::string description =
      branch(sides_[0], calib_.left_sensor_id) + " " + branch(sides_[1], calib_.right_sensor_id);
    RCLCPP_INFO(get_logger(), "pipeline: %s", description.c_str());
    pipeline_ = gst_parse_launch(description.c_str(), &err);
    if (!pipeline_ || err) {
      const std::string text = gst_error_text(err);
      if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
      }
      throw std::runtime_error("cannot build the GStreamer pipeline: " + text);
    }
    for (auto & side : sides_) {
      side.sink = gst_bin_get_by_name(GST_BIN(pipeline_), side.name.c_str());
      if (!side.sink) {
        stop_pipeline();
        throw std::runtime_error("appsink " + side.name + " not found in the pipeline");
      }
    }
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      const std::string text = pop_error();
      stop_pipeline();
      throw std::runtime_error("the pipeline did not start: " + text);
    }
    // A live pipeline reports NO_PREROLL; wait for PLAYING so the clock and base time are set.
    GstState state = GST_STATE_NULL;
    gst_element_get_state(pipeline_, &state, nullptr, 10 * GST_SECOND);
    const std::string early = pop_error();
    if (state != GST_STATE_PLAYING || !early.empty()) {
      stop_pipeline();
      throw std::runtime_error(
              "the pipeline did not reach PLAYING" + (early.empty() ? std::string() : ": " + early));
    }
    clock_ = gst_element_get_clock(pipeline_);
    base_time_ = gst_element_get_base_time(pipeline_);
    if (!clock_) {
      stop_pipeline();
      throw std::runtime_error("the pipeline has no clock");
    }
    running_ = true;
    for (auto & side : sides_) {
      side.thread = std::thread([this, &side]() {capture_loop(side);});
    }
  }

  void stop_pipeline()
  {
    running_ = false;
    for (auto & side : sides_) {
      if (side.thread.joinable()) {
        side.thread.join();
      }
    }
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    for (auto & side : sides_) {
      if (side.sink) {
        gst_object_unref(side.sink);
        side.sink = nullptr;
      }
    }
    if (clock_) {
      gst_object_unref(clock_);
      clock_ = nullptr;
    }
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  // First error waiting on the bus, or an empty string.
  std::string pop_error()
  {
    if (!pipeline_) {
      return {};
    }
    GstBus * bus = gst_element_get_bus(pipeline_);
    std::string text;
    while (GstMessage * msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR)) {
      GError * err = nullptr;
      gst_message_parse_error(msg, &err, nullptr);
      if (text.empty()) {
        text = gst_error_text(err);
      } else if (err) {
        g_error_free(err);
      }
      gst_message_unref(msg);
    }
    gst_object_unref(bus);
    return text;
  }

  void poll_bus()
  {
    if (!pipeline_) {
      return;
    }
    GstBus * bus = gst_element_get_bus(pipeline_);
    const auto types = static_cast<GstMessageType>(
      GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS);
    while (GstMessage * msg = gst_bus_pop_filtered(bus, types)) {
      GError * err = nullptr;
      if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
        gst_message_parse_error(msg, &err, nullptr);
        RCLCPP_ERROR(get_logger(), "camera pipeline error: %s", gst_error_text(err).c_str());
        running_ = false;
      } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_WARNING) {
        gst_message_parse_warning(msg, &err, nullptr);
        RCLCPP_WARN(get_logger(), "camera pipeline warning: %s", gst_error_text(err).c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "camera pipeline ended (end of stream)");
        running_ = false;
      }
      gst_message_unref(msg);
    }
    gst_object_unref(bus);
  }

  void capture_loop(Side & side)
  {
    while (running_) {
      GstSample * sample =
        gst_app_sink_try_pull_sample(GST_APP_SINK(side.sink), 200 * GST_MSECOND);
      if (!sample) {
        continue;
      }
      try {
        publish(side, sample);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "%s camera: %s", side.name.c_str(), e.what());
      }
      gst_sample_unref(sample);
    }
  }

  void publish(Side & side, GstSample * sample)
  {
    // The pipeline clock and ROS time, read together, map the buffer's time to ROS time.
    const GstClockTime clock_now = gst_clock_get_time(clock_);
    const int64_t ros_now = now().nanoseconds();
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
      throw std::runtime_error("sample without a buffer");
    }
    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    int64_t stamp = ros_now;
    bool has_pts = GST_CLOCK_TIME_IS_VALID(pts) && GST_CLOCK_TIME_IS_VALID(clock_now);
    if (has_pts) {
      // running time + base time = the frame's time on the pipeline clock
      const int64_t frame_clock = static_cast<int64_t>(base_time_ + pts);
      stamp = ros_now - (static_cast<int64_t>(clock_now) - frame_clock);
    }

    GstVideoInfo vinfo;
    if (!gst_video_info_from_caps(&vinfo, gst_sample_get_caps(sample))) {
      throw std::runtime_error("cannot read the frame format");
    }
    const int w = GST_VIDEO_INFO_WIDTH(&vinfo);
    const int h = GST_VIDEO_INFO_HEIGHT(&vinfo);
    if (w != calib_.width || h != calib_.height) {
      throw std::runtime_error(
              "frame is " + std::to_string(w) + "x" + std::to_string(h) + ", the calibration is for " +
              std::to_string(calib_.width) + "x" + std::to_string(calib_.height));
    }
    const int channels = encoding_ == "bgr8" ? 3 : 1;
    const int in_bytes = encoding_ == "bgr8" ? 4 : 1;   // BGRx or GRAY8
    const size_t stride = GST_VIDEO_INFO_PLANE_STRIDE(&vinfo, 0);
    const size_t offset = GST_VIDEO_INFO_PLANE_OFFSET(&vinfo, 0);

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      throw std::runtime_error("cannot map the frame");
    }
    if (map.size < offset + stride * (h - 1) + static_cast<size_t>(w) * in_bytes) {
      gst_buffer_unmap(buffer, &map);
      throw std::runtime_error("frame buffer is smaller than its format says");
    }
    auto msg = std::make_unique<Image>();
    msg->header.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    msg->header.frame_id = side.frame_id;
    msg->height = static_cast<uint32_t>(h);
    msg->width = static_cast<uint32_t>(w);
    msg->encoding = encoding_;
    msg->is_bigendian = 0;
    msg->step = static_cast<uint32_t>(w * channels);
    msg->data.resize(static_cast<size_t>(msg->step) * h);
    for (int y = 0; y < h; ++y) {
      const uint8_t * src = map.data + offset + stride * y;
      uint8_t * dst = msg->data.data() + static_cast<size_t>(msg->step) * y;
      if (channels == 3) {
        for (int x = 0; x < w; ++x) {   // BGRx to BGR
          dst[3 * x] = src[4 * x];
          dst[3 * x + 1] = src[4 * x + 1];
          dst[3 * x + 2] = src[4 * x + 2];
        }
      } else {
        std::memcpy(dst, src, static_cast<size_t>(w));
      }
    }
    gst_buffer_unmap(buffer, &map);

    auto info = std::make_unique<CameraInfo>(side.info);
    info->header = msg->header;
    side.image_pub->publish(std::move(msg));
    side.info_pub->publish(std::move(info));

    std::lock_guard<std::mutex> lock(stats_mutex_);
    side.stamps.push_back(stamp);
    while (side.stamps.size() > 1000) {
      side.stamps.pop_front();
    }
    side.latency_ms.push_back((ros_now - stamp) / 1e6);
    side.frames++;
    side.frames_total++;
    if (!has_pts) {
      side.without_timestamp++;
    }
  }

  // Each camera's rate, stamp-to-arrival time, and the gap between each left frame and the
  // nearest right frame (right minus left), since the last report.
  void report()
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const auto t = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(t - last_report_).count();
    last_report_ = t;
    Side & l = sides_[0];
    Side & r = sides_[1];
    std::vector<double> gaps;
    const int64_t since = l.stamps.empty() ? 0 : l.stamps.back() - static_cast<int64_t>(elapsed * 1e9);
    for (const int64_t ts : l.stamps) {
      if (ts < since || r.stamps.empty()) {
        continue;
      }
      auto it = std::lower_bound(r.stamps.begin(), r.stamps.end(), ts);
      int64_t best = INT64_MAX;
      if (it != r.stamps.end()) {
        best = *it - ts;
      }
      if (it != r.stamps.begin()) {
        const int64_t d = *std::prev(it) - ts;
        if (std::llabs(d) < std::llabs(best)) {
          best = d;
        }
      }
      if (best != INT64_MAX) {
        gaps.push_back(best / 1e6);
      }
    }
    std::vector<double> abs_gaps(gaps.size());
    std::transform(gaps.begin(), gaps.end(), abs_gaps.begin(), [](double g) {return std::fabs(g);});
    const Summary g = summarize(gaps);
    const Summary ga = summarize(abs_gaps);
    const Summary ll = summarize(l.latency_ms);
    const Summary rl = summarize(r.latency_ms);
    RCLCPP_INFO(
      get_logger(),
      "STATS left_fps=%.1f right_fps=%.1f gap_median_ms=%+.1f gap_abs_min_ms=%.1f "
      "gap_abs_max_ms=%.1f pairs=%zu left_latency_ms=%.1f/%.1f/%.1f "
      "right_latency_ms=%.1f/%.1f/%.1f total_frames=%lu/%lu without_timestamp=%lu/%lu",
      elapsed > 0 ? l.frames / elapsed : 0.0, elapsed > 0 ? r.frames / elapsed : 0.0,
      g.median, ga.min, ga.max, gaps.size(), ll.min, ll.median, ll.max, rl.min, rl.median, rl.max,
      static_cast<unsigned long>(l.frames_total), static_cast<unsigned long>(r.frames_total),
      static_cast<unsigned long>(l.without_timestamp),
      static_cast<unsigned long>(r.without_timestamp));
    for (auto & side : sides_) {
      side.frames = 0;
      side.latency_ms.clear();
    }
  }

  StereoCalibration calib_;
  std::string encoding_;
  std::string source_;
  int sensor_mode_ = 3;
  int fps_ = 30;
  GstElement * pipeline_ = nullptr;
  GstClock * clock_ = nullptr;
  GstClockTime base_time_ = 0;
  std::array<Side, 2> sides_;
  std::atomic<bool> running_{false};
  std::mutex stats_mutex_;
  std::chrono::steady_clock::time_point last_report_ = std::chrono::steady_clock::now();
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::TimerBase::SharedPtr bus_timer_;
};

}  // namespace stereo_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_camera_driver::StereoCamera)
