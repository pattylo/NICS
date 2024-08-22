/*
    This file is part of NICS - learning dynamic factor for visual odometry

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
 * \file ctrl_server.cpp
 * \date 26/10/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for nics_uav using airo_control_interface
 */

#include "nics_uav/ctrl_server.h"

ctrl_server::ctrl_server(ros::NodeHandle& _nh)
: nh(_nh), last_request(ros::Time::now().toSec())
{
    // subscribe
    uav_state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 1, &ctrl_server::uavStateCallback, this);
    uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 1, &ctrl_server::poseCallback, this);
    dist_sub = nh.subscribe<geometry_msgs::PointStamped>
            ("/nics_rse/dist", 1, &ctrl_server::distCallback, this);
    
    // check whether pose instantiate
    ros::Rate rate(20);
    while (ros::ok())
    {
        if(got_pose){
            ROS_BLUE_STREAM("GOT POSE FEEDBACK; STARTO!!!");
            break;
        }
            
        ros::spinOnce();
        rate.sleep();
    }

    config(nh);

    // publish  
    ctrlU_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 1, true);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/nics_uav/setpt", 1, true);
    
    // timer
    mainspin_timer = nh.createTimer(
        ros::Duration(1.0 / _pub_freq),
        &ctrl_server::mainspinCallback,
        this
    );

    //client
    uav_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");

    uav_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    // ros::shutdown();
}

ctrl_server::~ctrl_server()
{
    ROS_YELLOW_STREAM("CTRL_SERVER END!!!");
}

void ctrl_server::mainspinCallback(const ros::TimerEvent &e)
{
    fsm_manager();
    ctrl_pub();
}

void ctrl_server::config(ros::NodeHandle& _nh)
{
    ROS_INFO("Planner Server Launch...\n");

    nh.getParam("PID_gain", pid_gain_list);


    std::cout<<pid_gain_list.size()<<std::endl;
    std::cout<<"here! hi:..."<<std::endl;
    for(int i = 0; i < pid_gain_list.size(); i++)
    {
        std::cout<<"hi...."<<i<<std::endl;
        if(i == 0)
        {
            kp(0) = pid_gain_list[i]["x"];
            kp(1) = pid_gain_list[i]["y"];
            kp(2) = pid_gain_list[i]["z"];
            kp(3) = pid_gain_list[i]["yaw"];
        }
        else if(i == 1)
        {
            ki(0) = pid_gain_list[i]["x"];
            ki(1) = pid_gain_list[i]["y"];
            ki(2) = pid_gain_list[i]["z"];
            ki(3) = pid_gain_list[i]["yaw"];
        }
        else if(i == 2)
        {
            kd(0) = pid_gain_list[i]["x"];
            kd(1) = pid_gain_list[i]["y"];
            kd(2) = pid_gain_list[i]["z"];
            kd(3) = pid_gain_list[i]["yaw"];
        }
        std::cout<<i<<std::endl;            
    }

    std::cout<<"pid_gains..."<<std::endl;
    std::cout<<kp<<std::endl<<std::endl;
    std::cout<<ki<<std::endl<<std::endl;
    std::cout<<kd<<std::endl<<std::endl;

    nh.getParam("FF_gain", ff_gain_list);
    for(int i = 0; i < ff_gain_list.size(); i++)
    {
        if(i == 0)
        {
            kff(0) = ff_gain_list[i]["x"];
            kff(1) = ff_gain_list[i]["y"];
            kff(2) = ff_gain_list[i]["z"];
        }
    }
    std::cout<<"ff_gains..."<<std::endl;
    std::cout<<kff<<std::endl<<std::endl;

    nh.getParam("v_max", v_max);

    nh.getParam("meetup_thres", meetup_thres);

    double take_off_x, take_off_y, take_off_z;
    nh.getParam("take_off_x", take_off_x);
    nh.getParam("take_off_y", take_off_y);
    nh.getParam("take_off_z", take_off_z);
    
    uav_takeoff_pose(0) = uavPoseSE3.translation().x();
    uav_takeoff_pose(1) = uavPoseSE3.translation().y();

    uav_takeoff_pose(0) = take_off_x;
    uav_takeoff_pose(1) = take_off_y;
    uav_takeoff_pose(2) = take_off_z;
    
    uav_takeoff_pose(3) = 0.0;

    target_traj_pose = uav_takeoff_pose;

    std::cout<<"TAKEOFF POSE:\n"<<uav_takeoff_pose.head(3)<<std::endl;

    nh.getParam("hover_thrust", hover_thrust);
    std::cout<<hover_thrust<<std::endl;

    nh.getParam("pub_freq", _pub_freq);
    std::cout<<_pub_freq<<std::endl;

    nh.getParam("obs_on", obs_on);
    std::cout<<obs_on<<std::endl;
    // ros::shutdown();

    uav_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
}

void ctrl_server::fsm_manager()
{
    if(fsm_state == IDLE)
    {
        if(print_or_not)
        {
            ROS_YELLOW_STREAM(IDLE);
            print_or_not = false;
        }
        if(get_ready())
        {
            fsm_state = ARMED;
            print_or_not = true;

            std::cout<<"target takeoff position\n"<<target_traj_pose<<std::endl<<std::endl;
        }
    }
    else if(fsm_state == ARMED)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(ARMED);
            print_or_not = false;
        }
        if(taking_off())
        {
            fsm_state = TOOKOFF;
            print_or_not = true;
            last_request = ros::Time::now().toSec();
        }
    }
    else if(fsm_state == TOOKOFF)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(TOOKOFF);
            print_or_not = false;
        }
        if(ros::Time::now().toSec() - last_request > ros::Duration(2.0).toSec())
        {
            fsm_state = FOLLOW;
            print_or_not = true;
            last_request = ros::Time::now().toSec();
        }
    }
    else if(fsm_state == FOLLOW)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(FOLLOW);
            print_or_not = false;
        }
        if(false)
        {
            fsm_state = LAND;
            print_or_not = true;
            last_request = ros::Time::now().toSec();            
        }
    }
    // else if(fsm_state == LAND)
    // {
    //     if(print_or_not)
    //     {
    //         ROS_CYAN_STREAM(LAND);
    //         print_or_not = false;
    //     }

    //     if(land())
    //     {
    //         fsm_state = SHUTDOWN;
    //         print_or_not = true;
    //         last_request = ros::Time::now().toSec();
    //     }

    // }
    // else if(fsm_state == SHUTDOWN)
    // {
    //     if(print_or_not)
    //     {
    //         ROS_YELLOW_STREAM(SHUTDOWN);
    //         print_or_not = false;
    //     }
    //     if(shutdown())
    //     {
    //         fsm_state = MISSION_COMPLETE;
    //         print_or_not = true;
    //         last_request = ros::Time::now().toSec();
    //     }
    // }
    else if(fsm_state == MISSION_COMPLETE)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(MISSION_COMPLETE);
            print_or_not = false;
        }
        // ROS_GREEN_STREAM("NOW WAIT FOR EXIT...");
    }
    else
    {
        ROS_ERROR("Please Check System...");
    }

}

Eigen::Vector4d ctrl_server::pid_controller(
    Eigen::Vector4d pose, 
    Eigen::Vector4d setpoint
    )
{
    Eigen::Vector4d error, u_p, u_i, u_d, output, derivative;
    // error[0] = 1;
    double iteration_time = ros::Time::now().toSec() - pid_last_request;

    if(iteration_time > 1)
    {
        pid_last_request = ros::Time::now().toSec();
        return Eigen::Vector4d(0, 0, 0, 0);
    }

    error = setpoint - pose;
    // std::cout<<"ERROR!"<<std::endl<<std::endl;
    // std::cout<<error<<std::endl;


    if (error(3) >= M_PI)
    {
        error(3) -= 2 * M_PI;
    }

    if (error(3) <= -M_PI)
    {
        error(3) += 2  *M_PI;
    }
    // std::cout<<"error"<<std::endl<<error<<std::endl<<std::endl;
    using namespace std;

    for (int i = 0; i < 4; i++)
    { 
        //i = x,y,z
        integral(i) += (error(i) * iteration_time);

        if(integral(i) >  1)
        { 
            integral(i) = 1;
        }

        if(integral(i) < -1)
        { 
            integral(i) = -1;
        }

        derivative(i) = (error(i) - last_error(i)) / (iteration_time + 1e-10);

        u_p(i) = error(i) * kp(i);        //P controller
        u_i(i) = integral(i) * ki(i);     //I controller
        u_d(i) = derivative(i) * kd(i);   //D controller

        output(i) = u_p(i) + u_i(i) + u_d(i);        
    }

    for (int i = 0; i < 3; i++)
    {
        if(output(i) >  v_max)
            output(i) =  v_max;
        

        if(output(i) < -v_max)        
            output(i) = -v_max;
        
    }

    last_error = error;
    pid_last_request = ros::Time::now().toSec();

    return output;
}

Eigen::Vector4d ctrl_server::pidff_controller(
    Eigen::Vector4d pose, 
    Eigen::Vector4d setpoint, 
    Eigen::Vector3d setpt_ff    
)
{
    Eigen::Vector4d error, u_p, u_i, u_d, output, derivative;
    // error[0] = 1;
    double iteration_time = ros::Time::now().toSec() - pid_last_request;

    if(iteration_time > 1)
    {
        pid_last_request = ros::Time::now().toSec();
        return Eigen::Vector4d(0, 0, 0, 0);
    }

    error = setpoint - pose;


    if (error(3) >= M_PI)
    {
        error(3) -= 2 * M_PI;
    }

    if (error(3) <= -M_PI)
    {
        error(3) += 2  *M_PI;
    }

    for (int i = 0; i < 4; i++)
    { 
        //i = x,y,z
        integral(i) += (error(i) * iteration_time);

        if(integral(i) >  1)
        { 
            integral(i) = 1;
        }

        if(integral(i) < -1)
        { 
            integral(i) = -1;
        }

        derivative(i) = (error(i) - last_error(i)) / (iteration_time + 1e-10);

        u_p(i) = error(i) * kp(i);        //P controller
        u_i(i) = integral(i) * ki(i);     //I controller
        u_d(i) = derivative(i) * kd(i);   //D controller

        output(i) = u_p(i) + u_i(i) + u_d(i);
        
    }

    for (int i = 0; i < 3; i++)
    {
        if(output(i) >  v_max)
            { 
                output(i) =  v_max;
            }

        if(output(i) < -v_max)
        { 
            output(i) = -v_max;
        }
    }

    last_error = error;
    pid_last_request = ros::Time::now().toSec();

    return output;
}

void ctrl_server::ctrl_pub()
{
    Eigen::Vector3d u_I = pid_controller(
        uav_traj_pose,
        target_traj_pose
    ).head(3) - ext_dist;

    final_U = acc_to_U(
        u_I,
        target_traj_pose(3)
    );

    for(int i = 0; i < final_U.size();i++)
    {
        if(std::isnan(final_U(i)) && counter != 0 )
            patty::Debug("GOT STH WRONG");
    }
    counter++;

    ctrlU_msg.header.stamp = ros::Time::now();
    
    ctrlU_msg.header.frame_id = std::string("FCU");
    ctrlU_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    ctrlU_msg.orientation = rpy2qmsg(final_U.head(3));
    ctrlU_msg.thrust = final_U(3);

    ctrlU_pub.publish(ctrlU_msg);

    setpoint_msg.header.frame_id = "world";
    setpoint_msg.header.stamp = ros::Time::now();
    setpoint_msg.pose.position.x = target_traj_pose(0);
    setpoint_msg.pose.position.y = target_traj_pose(1);
    setpoint_msg.pose.position.z = target_traj_pose(2);

    pose_pub.publish(setpoint_msg);
}

bool ctrl_server::get_ready()
{
    bool return_state = false;

    if( uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now().toSec() - last_request > ros::Duration(2.0).toSec()))
        {
            if( uav_set_mode_client.call(uav_set_mode) &&
                uav_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }   
            last_request = ros::Time::now().toSec();
        } 
    else 
    {
        if( !uav_current_state.armed &&
            (ros::Time::now().toSec() - last_request > ros::Duration(2.0).toSec()))
        {
            if( uav_arming_client.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                ctrl_fsm_object.finite_state_machine = TOOKOFF;
                return_state = true;
            }
            else{
                ROS_ERROR("ARM FAIL");
            }
            last_request = ros::Time::now().toSec();
        }
    }

    if(return_state)
        return true;
    else
        return false;
}

bool ctrl_server::taking_off()
{
    double dis = 
        std::pow(
            uavPoseSE3.translation().x() - uav_takeoff_pose.x(),
            2
        )
        + std::pow(
            uavPoseSE3.translation().y() - uav_takeoff_pose.y(),
            2
        )
        + std::pow(
            uavPoseSE3.translation().z() - uav_takeoff_pose.z(),
            2
        );
    
    dis = std::sqrt(dis);

    if(dis < meetup_thres)
        return true;
    else    
        return false;
}

Eigen::Vector4d ctrl_server::acc_to_U(
    Eigen::Vector3d desired_acc,
    double desired_yaw
)
{
    Eigen::Vector4d Umsg; 
    // desired roll, pitch, yaw, thrust

    Umsg(3) = desired_acc.z() / g * hover_thrust + g / g * hover_thrust;
    // std::cout<<"HERE IS THE DESIRED ACC!!"<<std::endl;
    // std::cout<<desired_acc<<std::endl<<std::endl;
    // std::cout<<1.0 / desired_acc.z() * (
    //     desired_acc.x() * sin(desired_yaw) - desired_acc.y() * cos(desired_yaw) 
    // )<<std::endl;

    Umsg(0) = asin(std::min(
        std::max(
            1.0 / (desired_acc.z() + g) * 
            (
                desired_acc.x() * sin(desired_yaw) - desired_acc.y() * cos(desired_yaw) 
            ),
            -1.0
        ),
        1.0
    ));
    
    Umsg(1) = asin(std::min(
        std::max(
                1.0 / (desired_acc.z() + g) * 
                (
                    desired_acc.x() * cos(desired_yaw) + desired_acc.y() * sin(desired_yaw) 
                ) / cos(Umsg(0))
            ,
            -1.0
        ),
        1.0
    ));

    Umsg(0) = std::max(std::min(Umsg(0), M_PI/6),-M_PI/6);
    Umsg(1) = std::max(std::min(Umsg(1), M_PI/6),-M_PI/6);

    if(abs(Umsg(0)) > M_PI/6 || abs(Umsg(1)) > M_PI/6)
        patty::Debug("TO LARGE");

    // std::cout<<Umsg(0)<<std::endl;
    // std::cout<<Umsg(1)<<std::endl<<std::endl;;

    Umsg(2) = desired_yaw;
    
    return Umsg;
}
    