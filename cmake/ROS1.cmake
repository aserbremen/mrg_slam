cmake_minimum_required(VERSION 3.5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  geodesy
  nmea_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  interactive_markers
  eigen_conversions
  ndt_omp
  fast_gicp
)
catkin_python_setup()

if (catkin_FOUND) 
  add_definitions(-DROS_AVAILABLE=1)
endif ()

########################
## message generation ##
########################
add_message_files(FILES
  FloorCoeffs.msg
  ScanMatchingStatus.msg
  KeyFrameRos.msg
  EdgeRos.msg
  GraphRos.msg
  KeyFrameEstimate.msg
  EdgeEstimate.msg
  GraphEstimate.msg
  PoseWithName.msg
  PoseWithNameArray.msg
)

add_service_files(FILES
  SaveMap.srv
  GetMap.srv
  GetGraphEstimate.srv
  DumpGraph.srv
  PublishGraph.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hdl_graph_slam_nodelet
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# nodelets
add_library(prefiltering_nodelet apps/prefiltering_nodelet.cpp)
target_link_libraries(prefiltering_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_library(floor_detection_nodelet apps/floor_detection_nodelet.cpp)
target_link_libraries(floor_detection_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(floor_detection_nodelet ${PROJECT_NAME}_gencpp)


add_library(scan_matching_odometry_nodelet
  apps/scan_matching_odometry_nodelet.cpp
  src/hdl_graph_slam/registrations.cpp
)
target_link_libraries(scan_matching_odometry_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(scan_matching_odometry_nodelet ${PROJECT_NAME}_gencpp)


add_library(hdl_graph_slam_nodelet
  apps/hdl_graph_slam_nodelet.cpp
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
target_link_libraries(hdl_graph_slam_nodelet
  ${catkin_LIBRARIES}
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
add_dependencies(hdl_graph_slam_nodelet ${PROJECT_NAME}_gencpp)

catkin_install_python(
  PROGRAMS
    src/${PROJECT_NAME}/bag_player.py
    src/${PROJECT_NAME}/ford2bag.py
    src/${PROJECT_NAME}/map2odom_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  prefiltering_nodelet 
  floor_detection_nodelet
  scan_matching_odometry_nodelet 
  hdl_graph_slam_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)