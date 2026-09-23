cmake_minimum_required(VERSION 3.8)
project(perception_storage_ros2)

# 明确采用新版 find_package 行为，消除 PCL_ROOT 相关的 CMP0074 开发警告。
cmake_policy(SET CMP0074 NEW)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP)

add_definitions(${PCL_DEFINITIONS})

add_library(perception_storage_core
  src/perception_storage.cpp
  src/tof_detector.cpp
  src/lidar_detector.cpp
  src/tool_box.cpp
  src/points_filter.cpp
  src/depth_runner/depth_segmentor.cpp
  src/depth_runner/depth_edgedrawer.cpp
  src/depth_runner/depth_elsed.cpp
  src/depth_runner/depth_fullsegment.cpp
)

target_include_directories(perception_storage_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(perception_storage_core
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  cv_bridge
  pcl_conversions
)

target_link_libraries(perception_storage_core
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  Ceres::ceres
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(perception_storage_core OpenMP::OpenMP_CXX)
endif()

add_executable(perception_storage_node src/main.cpp)
target_link_libraries(perception_storage_node perception_storage_core)
ament_target_dependencies(perception_storage_node rclcpp)

install(TARGETS
  perception_storage_core
  perception_storage_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/start_hl2_gazebo.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)
install(PROGRAMS scripts/tof_pointcloud_noise.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs
  message_filters cv_bridge pcl_conversions
)

ament_package()
