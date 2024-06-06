/*
    This file is part of  NICS - learning dynamic factor for visual odometry

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
 * \file planner_server.h
 * \date 01/11/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for uav_path using airo_control_interface
 */

#ifndef PLANNER_SERVER_H
#define PLANNER_SERVER_H

#include <ros_utilities/ros_utilities.h>

#include "airo_message/FSMInfo.h"
#include "airo_message/TakeoffLandTrigger.h"
#include "airo_message/Reference.h"
#include "airo_message/ReferenceStamped.h"

#include "uavpath_lib.hpp"

// #define REPLAN "IDLE"

class planner_server : private RosUtilities
{
    enum State{
        TAKEOFF,
        COMMAND,
        LAND
    };

private:
    ros::NodeHandle nh;

    State state = TAKEOFF;

//ros related
    // subscriber
    ros::Subscriber uav_pose_sub, fsm_info_sub, ugv_pose_sub;

    geometry_msgs::PoseStamped uav_pose;
    inline void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        uav_pose = *msg;
    }

    airo_message::FSMInfo fsm_info;
    inline void fsmInfoCallback(const airo_message::FSMInfo::ConstPtr& msg)
    {
        fsm_info = *msg;
    }

    // publisher
    ros::Publisher command_pub, takeoff_land_pub;
    airo_message::TakeoffLandTrigger takeoff_land_trigger;

    // timer
    ros::Timer mainspin_timer;
    void mainspinCallback(const ros::TimerEvent &e);

// config
    bool traj_predefined = false;
    void config(ros::NodeHandle& _nh);
    int pub_freq;
    std::vector<geometry_msgs::Point> TRAJECTORY;

// exec_traj
    double last_request = 0;
    double starting_error = 0;
    airo_message::ReferenceStamped target_pose;

    void exec_predefined_traj();
    bool check_start_point();
    bool can_start = false;
    int traj_i = 0;

    void exec_online_traj();
    void check_collision();


public:
    planner_server(ros::NodeHandle& _nh);
    ~planner_server();

};

#endif