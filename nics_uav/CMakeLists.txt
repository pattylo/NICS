cmake_minimum_required(VERSION 3.0.2)
project(nics_uav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin COMPONENTS
  nodelet
  roscpp
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  ros_utilities
)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

roslaunch_add_file_check(launch)


add_message_files(
  FILES
  fsm.msg
  nics_ugv_blank.msg
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  geometry_msgs 
  mavros_msgs 
  roscpp
  std_msgs
  message_generation
)

set(Sophus_LIBRARIES libSophus.so)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ros_utilities
)

add_executable(planner_node
  src/planner_node.cpp
  src/planner_server.cpp
)

target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)

add_dependencies(planner_node
    airo_message_generate_messages_cpp
  )

  add_executable(ctrl_node
  src/ctrl_node.cpp
  src/ctrl_server.cpp
)

target_link_libraries(ctrl_node
  ${catkin_LIBRARIES}
)

add_dependencies(ctrl_node
    airo_message_generate_messages_cpp
  )


  add_executable(offb
  src/offb.cpp
)

target_link_libraries(offb
  ${catkin_LIBRARIES}
)
