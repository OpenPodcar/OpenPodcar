cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  laser_geometry
  tf
  pcl_conversions
  pcl_ros
  sensor_msgs
  message_generation
)


include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(podcarGazeboROSPlugin SHARED podcarGazeboROSPlugin.cpp)
target_link_libraries(podcarGazeboROSPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
