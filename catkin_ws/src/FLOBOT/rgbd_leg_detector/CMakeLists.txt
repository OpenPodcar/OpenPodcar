cmake_minimum_required(VERSION 2.8.3)
project(rgbd_leg_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

add_executable(rgbd_leg_detector
  src/rgbd_leg_detector.cpp
  )

target_link_libraries(rgbd_leg_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS rgbd_leg_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
