cmake_minimum_required(VERSION 2.8.3)
project(object3d_detector)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs visualization_msgs geometry_msgs pcl_conversions pcl_ros)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(object3d_detector src/object3d_detector.cpp src/svm.cpp)
target_link_libraries(object3d_detector ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector ${catkin_EXPORTED_TARGETS})
endif()

#############
## Install ##
#############

install(TARGETS object3d_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY libsvm
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
