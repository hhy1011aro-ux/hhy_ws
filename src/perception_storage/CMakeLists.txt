cmake_minimum_required(VERSION 3.0.2)
project(perception_storage)

SET(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  roslib
  tf
  tf_conversions
  tf2_ros
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs  
  message_filters
  hecbot_msgs
  hecbot_srvs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED system serialization filesystem thread)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS} 
        ${PCL_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

catkin_package(
        INCLUDE_DIRS
                include
                ${OpenCV_INCLUDE_DIRS}
                ${catkin_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIRS} 
                ${PCL_INCLUDE_DIRS}
                ${CERES_INCLUDE_DIRS}
                ${Boost_INCLUDE_DIRS}
        LIBRARIES
                ${PROJECT_NAME}
                mnano_gicp
                mnanoflann
        CATKIN_DEPENDS 
                cmake_modules
                roscpp
                roslib
                tf
                tf_conversions
                tf2_ros
                cv_bridge
                image_transport
                pcl_ros
                pcl_conversions
                std_msgs
                sensor_msgs
                geometry_msgs
                nav_msgs
                message_filters  
                hecbot_msgs
                hecbot_srvs
        DEPENDS PCL Eigen3 OpenCV Ceres Boost)

add_library(depth_runner 
            src/depth_runner/depth_segmentor.cpp 
            src/depth_runner/depth_edgedrawer.cpp 
            src/depth_runner/depth_elsed.cpp 
            src/depth_runner/depth_fullsegment.cpp)
target_link_libraries(depth_runner ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(depth_runner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(recognize_stage 
            src/perception_storage.cpp 
            src/tof_detector.cpp 
            src/lidar_detector.cpp 
            src/tool_box.cpp
            src/points_filter.cpp
            src/analyzer.cpp)
target_link_libraries(recognize_stage 
                      ${OpenCV_LIBRARIES} 
                      ${catkin_LIBRARIES} 
                      ${Eigen3_LIBRARIES} 
                      ${PCL_LIBRARIES} 
                      ${CERES_LIBRARIES} 
                      ${Boost_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      depth_runner)
add_dependencies(recognize_stage ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(mnanoflann STATIC src/pose_tracker/nano_gicp/nanoflann.cpp)
target_link_libraries(mnanoflann ${PCL_LIBRARIES})
add_library(mnano_gicp STATIC src/pose_tracker/nano_gicp/nano_gicp.cpp)
target_link_libraries(mnano_gicp ${PCL_LIBRARIES} ${EIGEN3_LIBS} ${OpenMP_LIBS} mnanoflann)

add_executable(perception_storage_node
        src/perception_node.cpp
        src/pose_tracker/pose_tracker.cpp
)
#monitor
add_executable(monitor
        src/main.cpp
        src/hecbot_storage_itof_monitor.cpp
)
add_dependencies(perception_storage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_storage_node
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        recognize_stage
        depth_runner
        mnano_gicp
) 
target_link_libraries(monitor
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
) 
install(TARGETS depth_runner recognize_stage mnanoflann mnano_gicp perception_storage_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
