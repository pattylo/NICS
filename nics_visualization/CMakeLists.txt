cmake_minimum_required(VERSION 3.0.2)
project(nics_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
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
  pcl_conversions
  pcl_ros 
  # ifopt
  # nlopt
  # qpOASES
  # OsqpEigen
  ros_utilities

)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(decomp_util REQUIRED)

# GTSAM
find_package(GTSAMCMakeTools) # Include GTSAM CMake tools
find_package(GTSAM REQUIRED) # Find GTSAM components

roslaunch_add_file_check(launch)

add_message_files(
 FILES
 nics_blank.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb
  CATKIN_DEPENDS 
  geometry_msgs 
  mavros_msgs 
  roscpp
  std_msgs
  message_generation
#  DEPENDS system_lib
)

set(Sophus_LIBRARIES libSophus.so)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${IPOPT_LIBRARIES}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ros_utilities
)

# add_executable(vis
#    src/vis.cpp
#    )
   
# target_link_libraries(vis
#   # ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OSQP_LIBRARIES}
# )

add_executable(data
src/data.cpp
)

target_link_libraries(data
${catkin_LIBRARIES}
)

add_dependencies(
    data
    nics_rse_generate_messages_cpp
  )
###################################################


