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
 * \file planner_server.cpp
 * \date 26/10/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for nics_uav using airo_control_interface
 */

#include "nics_uav/planner_server.h"

planner_server::planner_server(ros::NodeHandle& _nh)
: nh(_nh)
{
    // subscribe
    uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 1, &planner_server::poseCallback, this);
    fsm_info_sub = nh.subscribe<airo_message::FSMInfo>
            ("/airo_control/fsm_info", 1, &planner_server::fsmInfoCallback, this);
    ugv_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/gh034_car/pose", 1, &planner_server::ugvposeCallback, this);
    
    
    // publish            
    command_pub = nh.advertise<airo_message::ReferenceStamped>
            ("/airo_control/setpoint",1, true);
    takeoff_land_pub = nh.advertise<airo_message::TakeoffLandTrigger>
            ("/airo_control/takeoff_land_trigger",1, true);

    nh.getParam("predefined", traj_predefined);

    // rosparam for trajectory calling
    if(traj_predefined)
        config(nh);
    
    // timer
    mainspin_timer = nh.createTimer(
        ros::Duration(1.0 / pub_freq),
        &planner_server::mainspinCallback,
        this
    );
}

planner_server::~planner_server()
{

}

void planner_server::config(ros::NodeHandle& _nh)
{
    // traj type
    
    std::string TrajType;
    _nh.getParam("TrajType",TrajType);
    
    Eigen::Vector3d center;
    _nh.getParam("x0", center.x());
    _nh.getParam("y0", center.y());
    _nh.getParam("z0", center.z());

    _nh.getParam("pub_freq", pub_freq);

    int lap_no;
    _nh.getParam("lap_no", lap_no);

    bool ccw;
    _nh.getParam("ccw", ccw);


    bool z_moving;
    _nh.getParam("z_moving", z_moving);
    double z_moving_amp;
    _nh.getParam("z_moving_amp", z_moving_amp);
    double z_moving_period;
    _nh.getParam("z_moving_perioud", z_moving_period);

    std::unique_ptr<uav::uavpath> uavpath_ptr;
    if(TrajType == CIRCLE_TRAJ)
    {
        double velo;
        _nh.getParam("circle_velo",velo);
        double radius;
        _nh.getParam("circle_radius", radius);

        uavpath_ptr = std::make_unique<uav::uavpath>(
            center,
            pub_freq,
            lap_no,
            ccw,
            velo,
            radius,
            z_moving,
            z_moving_amp,
            z_moving_period,
            CIRCLE_TRAJ
        );
    }
    else if (TrajType == BLOCK_TRAJ)
    {
        double velo;
        _nh.getParam("block_velo",velo);
        double length;
        _nh.getParam("block_length", length);
        double aspect_ratio;
        _nh.getParam("block_AR", aspect_ratio);
        double T_angle; // dg
        _nh.getParam("block_T_angle", T_angle);

        uavpath_ptr = std::make_unique<uav::uavpath>(
            center,
            pub_freq,
            lap_no,
            ccw,
            velo,
            length,
            aspect_ratio,
            T_angle,
            z_moving,
            z_moving_amp,
            z_moving_period,
            BLOCK_TRAJ
        );
    }
    else if (TrajType == LEMNISCATE_TRAJ)
    {
        double amp;
        _nh.getParam("amp",amp);
        double velo;
        _nh.getParam("lemi_velo",velo);
        double timePerLap;
        _nh.getParam("timePerLap",timePerLap);

        uavpath_ptr = std::make_unique<uav::uavpath>(
            center,
            pub_freq,
            lap_no,
            ccw,
            amp,
            velo,
            timePerLap,
            z_moving,
            z_moving_amp,
            z_moving_period,
            LEMNISCATE_TRAJ
        );
    }
    else if (TrajType == HOVER)
    {
        
        wehover = true;
        hover_pt.x = center.x();
        hover_pt.y = center.y();
        hover_pt.z = center.z();
    }
    
    if(TrajType != HOVER)
        TRAJECTORY = uavpath_ptr->get_3Dtraj();
    

    _nh.getParam("starting_error", starting_error);

    double offset_x, offset_y, offset_z;
    _nh.getParam("global_offset_x", offset_x);
    _nh.getParam("global_offset_y", offset_y);
    _nh.getParam("global_offset_z", offset_z);
    
    global_offset = Eigen::Vector3d(offset_x, offset_y, offset_z);

}

void planner_server::mainspinCallback(const ros::TimerEvent &e)
{
    switch(state){
            case TAKEOFF:{
                
                if(fsm_info.is_landed == true){
                    ROS_CYAN_STREAM("TAKE OFF!");
                    takeoff_land_trigger.takeoff_land_trigger = true; // Takeoff
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                }
                if(fsm_info.is_waiting_for_command){
                    state = COMMAND;
                    ROS_CYAN_STREAM("TAKEOFF ==>> COMMAND!");
                    last_request = ros::Time::now().toSec();
                }
                break;
            }

            case COMMAND:{
                if(
                    fsm_info.is_waiting_for_command
                )
                {
                    if(wehover)
                    {
                        hover();                    
                    }
                    else
                    {
                        if(traj_predefined)
                            exec_predefined_traj();
                        else
                            exec_online_traj();
                    }
                    
                    target_pose.header.stamp = ros::Time::now();
                    command_pub.publish(target_pose);

                    if(traj_i == TRAJECTORY.size() - 1)
                    {
                        state = LAND;
                        ROS_CYAN_STREAM("COMMAND ==>> LAND!");
                    }
                        
                }
                break;
            }

            case LAND:{
                if(fsm_info.is_waiting_for_command){
                    takeoff_land_trigger.takeoff_land_trigger = false; // Land
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                }
                break;
            }
        }
}

void planner_server::hover()
{

    target_pose_Eigen = transform_to_non_inertial_frame(
        hover_pt
    );

    std::cout<<target_pose_Eigen<<std::endl;

    target_pose.ref.pose.position.x = target_pose_Eigen.x();
    target_pose.ref.pose.position.y = target_pose_Eigen.y();
    target_pose.ref.pose.position.z = target_pose_Eigen.z();

}

void planner_server::exec_predefined_traj()
{
    if(!can_start)
    {
        can_start = check_start_point();
    }
    else
    {
        traj_i ++;            
    }

    std::cout<<"here"<<std::endl;
    std::cout<<TRAJECTORY[traj_i]<<std::endl;
    std::cout<<std::endl<<std::endl;;

    target_pose_Eigen = transform_to_non_inertial_frame(TRAJECTORY[traj_i]);
    
    target_pose_Eigen = target_pose_Eigen;


    target_pose.ref.pose.position.x = target_pose_Eigen.x();
    target_pose.ref.pose.position.y = target_pose_Eigen.y();
    target_pose.ref.pose.position.z = target_pose_Eigen.z();

}

bool planner_server::check_start_point()
{    
    if(
        sqrt(
            abs(uav_pose.pose.position.x - TRAJECTORY[0].x)
            + abs(uav_pose.pose.position.y - TRAJECTORY[0].y)
            + abs(uav_pose.pose.position.z - TRAJECTORY[0].z)
        ) < starting_error
    )
    {
        ROS_GREEN_STREAM("START TRAJ!");
        return true;
    }        
    else
        return false;
}

void planner_server::exec_online_traj()
{
    check_collision();
}

void planner_server::check_collision()
{

}

Eigen::Vector3d planner_server::transform_to_non_inertial_frame(
    geometry_msgs::Point local_pt_b
)
{
    return SO3_rotate_vector(
        ugvPoseSE3.so3(), 
        Eigen::Vector3d(
            local_pt_b.x,
            local_pt_b.y,
            local_pt_b.z
        )
    ) + ugvPoseSE3.translation();
}
