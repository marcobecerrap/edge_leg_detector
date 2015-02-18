cmake_minimum_required(VERSION 2.8.3)
project(edge_leg_detector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(detector src/edge_leg_detector/detector.cpp)

target_link_libraries(detector
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
