cmake_minimum_required(VERSION 3.5)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
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
# find_package(eigen_conversions REQUIRED) # TODO: deal with it later, not sure if available in ROS2
find_package(ndt_omp REQUIRED)
find_package(fast_gicp REQUIRED)
# ROS2 python depencencies
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

if (ament_cmake_FOUND)
  add_definitions(-DROS_AVAILABLE=2)
endif ()

########################
## message generation ##
########################
set(msg_files
  "msg/EdgeEstimate.msg"
  "msg/EdgeRos.msg"
  "msg/FloorCoeffs.msg"
  "msg/GraphEstimate.msg"
  "msg/GraphRos.msg"
  "msg/KeyFrameEstimate.msg"
  "msg/KeyFrameRos.msg"
  "msg/PoseWithName.msg"
  "msg/PoseWithNameArray.msg"
  "msg/ScanMatchingStatus.msg"
  # in case one unified branch for ROS1 and ROS2 is wanted, use the following message definitions instead.
  # This is due to the fact that 'duration' and 'time' (ROS1) cannot easily be bridged to 'builin_interfaces/Time' 'builin_interfaces/Duration'
  # see # https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html#message-service-and-action-definitions
  # "msg/KeyFrameRos2.msg"
  # "msg/KeyFrameEstimateRos2.msg"
  # "msg/GraphRos2.msg"
  # "msg/GraphEstimateRos2.msg"
)
  
set(srv_files
  "srv/DumpGraph.srv"
  "srv/GetGraphEstimate.srv"
  "srv/GetMap.srv"
  "srv/PublishGraph.srv"
  "srv/SaveMap.srv"
  # in case one unified branch for ROS1 and ROS2 is wanted, use the following message definitions instead
  # This is due to the fact that 'duration' and 'time' (ROS1) cannot easily be bridged to 'builin_interfaces/Time' 'builin_interfaces/Duration',
  # see # https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html#message-service-and-action-definitions
  # "srv/GetGraphEstimateRos2.srv"
  # "srv/GetMapRos2.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces std_msgs nmea_msgs sensor_msgs geometry_msgs nav_msgs geographic_msgs visualization_msgs tf2_geometry_msgs
)

###########
## Build ##
###########
# create ament index resource which references the libraries in the binary dir
# TODO what is this actually for?
set(node_plugins "")

# Include local hdl_graph_slam headers
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

###############################
## Floor Detection Component ##
###############################
add_library(floor_detection_component SHARED
  apps/floor_detection_component.cpp
  src/hdl_graph_slam/registrations.cpp
)
# Link non ament packages
target_link_libraries(floor_detection_component
  ${PCL_LIBRARIES}
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
  ndt_omp
  fast_gicp
)
# Make the component depend on custom messages in its own package.
rosidl_target_interfaces(floor_detection_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Register the component as part of hdl_graph_slam (project) ComponentManager
rclcpp_components_register_nodes(floor_detection_component "hdl_graph_slam::FloorDetectionComponent")
set(node_plugins "{node_plugins}hdl_graph_slam::FloorDetectionComponent;$<TARGET_FILE:floor_detection_component>\n")

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
  src/hdl_graph_slam/registrations.cpp
)
# Link non ament packages
target_link_libraries(scan_matching_odometry_component
  ${PCL_LIBRARIES}
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
  ndt_omp
  fast_gicp
)
# Make the component depend on custom messages in its own package.
rosidl_target_interfaces(scan_matching_odometry_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Register the component as part of hdl_graph_slam (project) ComponentManager
rclcpp_components_register_nodes(scan_matching_odometry_component "hdl_graph_slam::ScanMatchingOdometryComponent")
set(node_plugins "{node_plugins}hdl_graph_slam::ScanMatchingOdometryComponent;$<TARGET_FILE:scan_matching_odometry_component>\n")

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
  ndt_omp
  fast_gicp
)
# Register the component as part of hdl_graph_slam (project) ComponentManager
rclcpp_components_register_nodes(prefiltering_component "hdl_graph_slam::PrefilteringComponent")
set(node_plugins "{node_plugins}hdl_graph_slam::PrefilteringComponent;$<TARGET_FILE:prefiltering_component>\n")

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
add_library(hdl_graph_slam_component SHARED
  apps/hdl_graph_slam_component.cpp
  src/hdl_graph_slam/graph_slam.cpp
  src/hdl_graph_slam/edge.cpp
  src/hdl_graph_slam/global_id.cpp
  src/hdl_graph_slam/loop_detector.cpp
  src/hdl_graph_slam/keyframe.cpp
  src/hdl_graph_slam/keyframe_updater.cpp
  src/hdl_graph_slam/map_cloud_generator.cpp
  src/hdl_graph_slam/registrations.cpp
  src/hdl_graph_slam/information_matrix_calculator.cpp
  src/hdl_graph_slam/imu_processor.cpp
  src/hdl_graph_slam/gps_processor.cpp
  src/hdl_graph_slam/floor_coeffs_processor.cpp
  src/hdl_graph_slam/markers_publisher.cpp
  src/g2o/robust_kernel_io.cpp
)
# Link non ament packages
target_link_libraries(hdl_graph_slam_component
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
  ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
)
# Handles includes and linking of other ament targets
ament_target_dependencies(hdl_graph_slam_component
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
  fast_gicp
  ndt_omp
)
# The next line is needed for custom messages to be used within the same package.
# https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#link-against-the-interface
rosidl_target_interfaces(hdl_graph_slam_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Register the component as part of hdl_graph_slam (project) ComponentManager
rclcpp_components_register_nodes(hdl_graph_slam_component "hdl_graph_slam::HdlGraphSlamComponent")
set(node_plugins "{node_plugins}hdl_graph_slam::HdlGraphSlamComponent;$<TARGET_FILE:hdl_graph_slam_component>\n")
# Install the hdl_graph_slam_component (hdl_graph_slam_component.so) in workspace install folder
# TODO do we need to mark EXPORT with _export as described here? https://github.com/ament/ament_cmake/issues/329#issuecomment-801187892
install(
  TARGETS hdl_graph_slam_component
  EXPORT hdl_graph_slam_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
# ament_export_libraries(hdl_graph_slam_component)

# Install the config directory to work with parameter yaml files
install( 
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#######################################
## HDL Graph Slam manual composition ##
#######################################
option(BUILD_MANUAL_COMPOSITION "Build manual composition of components" OFF)
# only build if debugging hdl_graph_slam is needed, saving compile time
if (BUILD_MANUAL_COMPOSITION)
  add_executable(hdl_graph_slam_manual_composition
    apps/hdl_graph_slam_manual_composition.cpp
  )
  target_link_libraries(hdl_graph_slam_manual_composition
    prefiltering_component
    scan_matching_odometry_component
    floor_detection_component
    hdl_graph_slam_component
  )
  ament_target_dependencies(hdl_graph_slam_manual_composition
    rclcpp
  )
  install(TARGETS hdl_graph_slam_manual_composition
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Here we can export all downstream dependencies and include directories
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
# TODO: ament_export_targets

# Install python nodes
ament_python_install_package(../src/hdl_graph_slam)

# Install launch executables
install(PROGRAMS
  src/hdl_graph_slam/map2odom_publisher_ros2.py
  src/hdl_graph_slam/clock_publisher_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

# Finally create a pacakge
ament_package()