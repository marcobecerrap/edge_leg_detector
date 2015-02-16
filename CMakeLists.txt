cmake_minimum_required(VERSION 2.8.3)
project(leg_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
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

add_executable(leg_detector_node src/leg_detector_node.cpp)

target_link_libraries(leg_detector_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

