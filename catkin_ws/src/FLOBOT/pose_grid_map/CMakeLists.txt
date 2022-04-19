cmake_minimum_required(VERSION 2.8.3)
project(pose_grid_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(pose_grid_map
  src/pose_grid_map.cpp
  )

target_link_libraries(pose_grid_map
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS pose_grid_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
