cmake_minimum_required(VERSION 3.0.2)
project(nics_ugv)

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

)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
#find_package(osqp REQUIRED)
#find_package(OsqpEigen REQUIRED)
#find_package(decomp_util REQUIRED)


# find_package(PkgConfig REQUIRED)
# pkg_check_modules(gtk3 REQUIRED IMPORTED_TARGET gtk+-3.0)
# pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
# pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0>=1.4)
# pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
# pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0>=1.4)


# pkg_get_variable(GST_VERSION gstreamer-1.0 version)
# message(STATUS "Gstreamer version:${GST_VERSION}")


# find_package(PCL 1.2 REQUIRED)

roslaunch_add_file_check(launch)


add_message_files(
  FILES
  nics_ugv_blank.msg
  #  AlanPlannerMsg.msg 
  #  StateMachine.msg
  #  Target.msg
  #  vector3D.msg
  #  vector4D.msg
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

# for ncnn
# FIND_PACKAGE( OpenMP REQUIRED)
#     if(OPENMP_FOUND)
#       message("OPENMP FOUND")
#       set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#       set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#     endif()

set(Sophus_LIBRARIES libSophus.so)

# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/nics_landing_planning/third_party/mosek/lib)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/nics_landing_planning/third_party/mosek/include)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
 # ${IPOPT_LIBRARIES}
 # ${DECOMP_UTIL_INCLUDE_DIRS} 

# ${CMAKE_CURRENT_SOURCE_DIR}/src/include/ncnn
)

add_executable(virtual_car
  src/virtual_car.cpp
  src/RosTopicConfigs.cpp
)

target_link_libraries(virtual_car
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  #${PCL_LIBRARIES}
 # ${OSQP_LIBRARIES}
  # mosek64
  # qpOASES
 # osqp::osqp
 # OsqpEigen#::OsqpEigen
)


add_executable(physique_car
  src/physique_car.cpp
  src/RosTopicConfigs.cpp
)

target_link_libraries(physique_car
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
 # ${PCL_LIBRARIES}
 # ${OSQP_LIBRARIES}
  # mosek64
  # qpOASES
  #osqp::osqp
  #OsqpEigen#::OsqpEigen
)
