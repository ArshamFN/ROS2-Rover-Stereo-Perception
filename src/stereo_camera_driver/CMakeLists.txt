cmake_minimum_required(VERSION 3.8)
project(stereo_camera_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

# No OpenCV and no cv_bridge: frames go from GStreamer straight into the ROS message,
# and the calibration file is read with yaml-cpp.
add_library(stereo_camera_component SHARED
  src/calibration.cpp
  src/stereo_camera.cpp
)
target_include_directories(stereo_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(stereo_camera_component PkgConfig::GST ${YAML_CPP_LIBRARIES})
ament_target_dependencies(stereo_camera_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(stereo_camera_component
  PLUGIN "stereo_camera_driver::StereoCamera"
  EXECUTABLE stereo_camera
)

install(TARGETS stereo_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_calibration test/test_calibration.cpp)
    target_link_libraries(test_calibration stereo_camera_component)
    target_compile_definitions(test_calibration PRIVATE
      TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  endif()
endif()

ament_package()
