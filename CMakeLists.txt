# SPDX-License-Identifier: BSD-2-Clause
cmake_minimum_required(VERSION 3.5)
project(mrg_slam)

# Can we use C++17 in indigo?
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2 -DNDEBUG)
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2 -O3")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_library(VGICP_CUDA_FOUND NAMES fast_vgicp_cuda)
message(STATUS "VGICP_CUDA_FOUND:" ${VGICP_CUDA_FOUND})
if(VGICP_CUDA_FOUND)
  add_definitions(-DUSE_VGICP_CUDA)
endif()

# ROS 2 integration, enable c++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Use -std=c++14 instead of -std=gnu++14
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -fno-omit-frame-pointer")

# Find necessary ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mrg_slam_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geodesy REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(message_filters REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(fast_gicp REQUIRED)
# ROS2 python depencencies
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# small_gicp
find_package(small_gicp REQUIRED)

if (ament_cmake_FOUND)
  add_definitions(-DROS_AVAILABLE=2)
endif ()

###########
## Build ##
###########

# Include local mrg_slam headers
include_directories(include)

###############################
## Floor Detection Component ##
###############################
add_library(floor_detection_component SHARED
  apps/floor_detection_component.cpp
  src/mrg_slam/registrations.cpp
  src/ros_utils.cpp
)
# Link non ament packages
target_link_libraries(floor_detection_component
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)
# Handles includes and linking of other ament targets
ament_target_dependencies(floor_detection_component
  rclcpp
  rclcpp_components
  pcl_ros
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  nav_msgs
  mrg_slam_msgs
  ndt_omp
  fast_gicp
)
# Register the component as part of mrg_slam (project) ComponentManager
rclcpp_components_register_nodes(floor_detection_component "mrg_slam::FloorDetectionComponent")
# Install the floor_detection_component (libfloor_detection_component.so) in workspace install folder
install(
  TARGETS floor_detection_component
  EXPORT floor_detection_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

######################################
## Scan Matching Odometry Component ##
######################################
add_library(scan_matching_odometry_component SHARED
  apps/scan_matching_odometry_component.cpp
  src/mrg_slam/registrations.cpp
  src/ros_utils.cpp
)
# Link non ament packages
target_link_libraries(scan_matching_odometry_component
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)
# Handles includes and linking of other ament targets
ament_target_dependencies(scan_matching_odometry_component
  rclcpp
  rclcpp_components
  pcl_ros
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mrg_slam_msgs
  ndt_omp
  fast_gicp
)
# Register the component as part of mrg_slam (project) ComponentManager
rclcpp_components_register_nodes(scan_matching_odometry_component "mrg_slam::ScanMatchingOdometryComponent")
# Install the scan_matching_odometry_component (scan_matching_odometry_component.so) in workspace install folder
install(
  TARGETS scan_matching_odometry_component
  EXPORT scan_matching_odometry_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

############################
## Prefiltering Component ##
############################
add_library(prefiltering_component SHARED
  apps/prefiltering_component.cpp
  src/ros_utils.cpp
)
# Link non ament packages
target_link_libraries(prefiltering_component
  ${PCL_LIBRARIES}
)
# Handles includes and linking of other ament targets
ament_target_dependencies(prefiltering_component
  rclcpp
  rclcpp_components
  pcl_ros
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  ndt_omp
  fast_gicp
)
# Register the component as part of mrg_slam (project) ComponentManager
rclcpp_components_register_nodes(prefiltering_component "mrg_slam::PrefilteringComponent")
# Install the prefiltering_component (prefiltering_component.so) in workspace install folder
install(
  TARGETS prefiltering_component
  EXPORT prefiltering_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

##############################
## HDL Graph Slam Component ##
##############################
add_library(mrg_slam_component SHARED
  apps/mrg_slam_component.cpp
  src/mrg_slam/graph_database.cpp
  src/mrg_slam/graph_slam.cpp
  src/mrg_slam/edge.cpp
  src/mrg_slam/loop_detector.cpp
  src/mrg_slam/keyframe.cpp
  src/mrg_slam/keyframe_updater.cpp
  src/mrg_slam/map_cloud_generator.cpp
  src/mrg_slam/registrations.cpp
  src/mrg_slam/information_matrix_calculator.cpp
  src/mrg_slam/imu_processor.cpp
  src/mrg_slam/gps_processor.cpp
  src/mrg_slam/floor_coeffs_processor.cpp
  src/mrg_slam/markers_publisher.cpp
  src/g2o/robust_kernel_io.cpp
  src/ros_utils.cpp
)
# Link non ament packages
target_link_libraries(mrg_slam_component
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
  ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
  small_gicp::small_gicp
)
# Handles includes and linking of other ament targets
ament_target_dependencies(mrg_slam_component
  rclcpp
  rclcpp_components
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geodesy
  message_filters
  builtin_interfaces
  std_msgs
  nmea_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  visualization_msgs
  mrg_slam_msgs
  fast_gicp
  ndt_omp
)
# Register the component as part of mrg_slam (project) ComponentManager
rclcpp_components_register_nodes(mrg_slam_component "mrg_slam::MrgSlamComponent")
# Install the mrg_slam_component (mrg_slam_component.so) in workspace install folder
install(
  TARGETS mrg_slam_component
  EXPORT mrg_slam_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


# Here we can export all downstream dependencies and include directories
# TODO do we need to mark EXPORT with _export as described here? https://github.com/ament/ament_cmake/issues/329#issuecomment-801187892
# TODO export package for downstream packages to use see: https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602/2
# also see: https://answers.ros.org/question/331277/ament_cmake-confused-about-ament_export_dependencies-and-ament_export_interfaces/
# ament_export_libraries(${PROJECT_NAME})
# ament_export_interfaces(${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

# Install necessary directories to install folder
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install ROS2 launch files
install(PROGRAMS
  launch/mrg_slam.launch.py
  launch/mrg_slam_debug.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install ROS2 rviz configs
install(FILES
  rviz/mrg_slam.rviz
  rviz/nebula.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install python nodes
ament_python_install_package(scripts)

# Install python nodes
install(PROGRAMS
  scripts/map2odom_publisher_ros2.py
  scripts/clock_publisher_ros2.py
  scripts/odom_to_file.py
  DESTINATION lib/${PROJECT_NAME}
)

# Finally create a pacakge
ament_package()