/*
    This file is part of NICS - the non-robocentric dynamic landing system for quadrotor

    NICS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    NICS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NICS.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file essential.h
 * \date 28/07/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief frequently used header files of libraries
 */

#ifndef ESSENTIAL_H
#define ESSENTIAL_H
#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>

#include <Eigen/Dense>

#include <sophus/se3.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <math.h>
#include <stdio.h>
#include <numeric>
#include <chrono>
#include <iomanip>
#include <string>
#include <cmath>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include <queue>
#include <mutex>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/NavSatFix.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

#include <yaml-cpp/yaml.h>


namespace patty
{
  enum PRINT_COLOR
  {
    BLACK,
    RED,
    GREEN,
    YELLOW,
    BLUE,
    MAGENTA,
    CYAN,
    WHITE,
    ENDCOLOR
  };

  inline std::ostream& operator<<(std::ostream& os, PRINT_COLOR c)
  {
    switch(c)
    {
      case BLACK    : os << "\033[1;30m"; break;
      case RED      : os << "\033[1;31m"; break;
      case GREEN    : os << "\033[1;32m"; break;
      case YELLOW   : os << "\033[1;33m"; break;
      case BLUE     : os << "\033[1;34m"; break;
      case MAGENTA  : os << "\033[1;35m"; break;
      case CYAN     : os << "\033[1;36m"; break;
      case WHITE    : os << "\033[1;37m"; break;
      case ENDCOLOR : os << "\033[0m";    break;
      default       : os << "\033[1;37m";
    }
    return os;
  }

  inline void Debug(std::string debug_message)
  {
      ROS_INFO_STREAM(patty::RED << "DEBUG! -> " << debug_message << patty::ENDCOLOR);
      ros::shutdown();
  }
} //namespace patty

#define ROS_BLACK_STREAM(x)   ROS_INFO_STREAM(patty::BLACK   << x << patty::ENDCOLOR)
#define ROS_RED_STREAM(x)     ROS_INFO_STREAM(patty::RED     << x << patty::ENDCOLOR)
#define ROS_GREEN_STREAM(x)   ROS_INFO_STREAM(patty::GREEN   << x << patty::ENDCOLOR)
#define ROS_YELLOW_STREAM(x)  ROS_INFO_STREAM(patty::YELLOW  << x << patty::ENDCOLOR)
#define ROS_BLUE_STREAM(x)    ROS_INFO_STREAM(patty::BLUE    << x << patty::ENDCOLOR)
#define ROS_MAGENTA_STREAM(x) ROS_INFO_STREAM(patty::MAGENTA << x << patty::ENDCOLOR)
#define ROS_CYAN_STREAM(x)    ROS_INFO_STREAM(patty::CYAN    << x << patty::ENDCOLOR)

#define ROS_BLACK_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, patty::BLACK   << x << patty::ENDCOLOR)
#define ROS_RED_STREAM_COND(c, x)     ROS_INFO_STREAM_COND(c, patty::RED     << x << patty::ENDCOLOR)
#define ROS_GREEN_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, patty::GREEN   << x << patty::ENDCOLOR)
#define ROS_YELLOW_STREAM_COND(c, x)  ROS_INFO_STREAM_COND(c, patty::YELLOW  << x << patty::ENDCOLOR)
#define ROS_BLUE_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, patty::BLUE    << x << patty::ENDCOLOR)
#define ROS_MAGENTA_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, patty::MAGENTA << x << patty::ENDCOLOR)
#define ROS_CYAN_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, patty::CYAN    << x << patty::ENDCOLOR)

#define BLACK_STREAM(x)     std::cout<< patty::BLACK << x << std::endl << patty::ENDCOLOR
#define RED_STREAM(x)       std::cout<< patty::RED << x << std::endl << patty::ENDCOLOR
#define GREEN_STREAM(x)     std::cout<< patty::GREEN << x << std::endl << patty::ENDCOLOR
#define YELLOW_STREAM(x)    std::cout<< patty::YELLOW << x << std::endl << patty::ENDCOLOR
#define BLUE_STREAM(x)      std::cout<< patty::BLUE << x << std::endl << patty::ENDCOLOR
#define MAGNETA_STREAM(x)   std::cout<< patty::MAGNETA << x << std::endl << patty::ENDCOLOR
#define CYAN_STREAM(x)      std::cout<< patty::CYAN << x << std::endl << patty::ENDCOLOR


#else
#endif