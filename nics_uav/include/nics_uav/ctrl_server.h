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
 * \file ctrl_server.h
 * \date 01/11/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for uav_path using airo_control_interface
 */

#ifndef CTRL_SERVER_H
#define CTRL_SERVER_H

#include <ros_utilities/ros_utilities.h>

#include "airo_message/FSMInfo.h"
#include "airo_message/TakeoffLandTrigger.h"
#include "airo_message/Reference.h"
#include "airo_message/ReferenceStamped.h"

#include "nics_uav/fsm.h"

#define IDLE "IDLE"
#define ARMED "ARMED"
#define TOOKOFF "TOOKOFF"
#define FOLLOW "FOLLOW"
#define LAND "LAND"
#define SHUTDOWN "SHUTDOWN"
#define MISSION_COMPLETE "MISSION_COMPLETE"

class ctrl_server : private rosUtilities
{

private:
    ros::NodeHandle nh;

//ros related
    // subscriber
    ros::Subscriber uav_state_sub, uav_pose_sub;
    int counter = 0;

    bool got_pose = false;
    geometry_msgs::PoseStamped uav_pose;
    Sophus::SE3d uavPoseSE3;
    inline void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        got_pose = true;
        uav_pose = *msg;
        uavPoseSE3 = posemsg_to_SE3(uav_pose.pose);
        uav_traj_pose = SE3_to_flat(uavPoseSE3);
    }
    
    inline void uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        uav_current_state = *msg;
    }
    
    // publisher
    ros::Publisher ctrlU_pub, pose_pub;
    mavros_msgs::AttitudeTarget ctrlU_msg;


    // timer
    ros::Timer mainspin_timer;
    void mainspinCallback(const ros::TimerEvent &e);

// config
    void config(ros::NodeHandle& _nh);

// mavros service
    ros::ServiceClient uav_arming_client;
    ros::ServiceClient uav_set_mode_client;

    mavros_msgs::SetMode uav_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::State uav_current_state;


// fsm
    void fsm_manager();
    bool get_ready();
    bool taking_off();
    std::string fsm_state = IDLE;
    nics_uav::fsm ctrl_fsm_object;

// position controller related
    void ctrl_pub();
    double thrust_coef;

    Eigen::Vector4d pid_controller(
        Eigen::Vector4d pose, 
        Eigen::Vector4d setpoint
    );
    Eigen::Vector4d pidff_controller(
        Eigen::Vector4d pose, 
        Eigen::Vector4d setpoint, 
        Eigen::Vector3d setpt_ff
    );
    Eigen::Vector4d acc_to_U(
        Eigen::Vector3d desired_acc,
        double desired_yaw
    );
    double g = 9.81;
    double hover_thrust;

    Eigen::Vector4d uav_traj_pose, target_traj_pose, uav_takeoff_pose;
    Eigen::Vector4d final_U;
    bool uav_traj_pose_initiated = false;
    bool ugv_traj_pose_initiated = false;
    double last_request;
    double pid_last_request = 0;
    bool print_or_not = true;
    double following_norm = 0;
    int land_fix_count = 0;

    XmlRpc::XmlRpcValue pid_gain_list;
    Eigen::Vector4d kp, ki, kd;
    Eigen::Vector4d last_error, integral;
    int _pub_freq = 0;
    double v_max, a_max;

    XmlRpc::XmlRpcValue ff_gain_list;
    Eigen::Vector3d kff;
    double meetup_thres;




public:
    ctrl_server(ros::NodeHandle& _nh);
    ~ctrl_server();

};

#endif