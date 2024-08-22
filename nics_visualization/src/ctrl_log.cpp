// 2024 Rebuttal
/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file ctrl_log.cpp
 * \date 16/02/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief executable to record data
*/

#include <ros_utilities/ros_utilities.h>

static std::string log_file;
static double starting_time;

static std::shared_ptr<ros_utilities> ros_tools_ptr;

static geometry_msgs::PoseStamped setpt_pose;
static geometry_msgs::PoseStamped uav_pose;
static Eigen::Vector3d setpose_euler;
static Eigen::Vector3d uavpose_euler;

void setpt_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg
)
{
    setpt_pose = *msg;

    //
    std::ofstream save(log_file, std::ios::app);

    setpose_euler = ros_tools_ptr->q2rpy(
        ros_tools_ptr->Qmsg_to_Q(
            setpt_pose.pose.orientation
        )
    );

    uavpose_euler = ros_tools_ptr->q2rpy(
        ros_tools_ptr->Qmsg_to_Q(
            uav_pose.pose.orientation
        )
    );
     
    // save<<"t, set_x,set_y,set_z,set_ry, uav_x,uav_y,uav_z,uav_ry, e_x,e_y,e_z,e_ry"<<std::endl;
    
    save << msg->header.stamp.now().toSec() - starting_time << ","

         << setpt_pose.pose.position.x << ","
         << setpt_pose.pose.position.y << ","
         << setpt_pose.pose.position.z << ","
         << setpose_euler(2) << ","

         << uav_pose.pose.position.x << ","
         << uav_pose.pose.position.y << ","
         << uav_pose.pose.position.z << ","
         << uavpose_euler(2) << ","

         << setpt_pose.pose.position.x - uav_pose.pose.position.x << ","
         << setpt_pose.pose.position.y - uav_pose.pose.position.y << ","
         << setpt_pose.pose.position.z - uav_pose.pose.position.z << ","
         << setpose_euler(2) - uavpose_euler(2) << ","
         << std::endl;
    ;

    save.close();
}

void uavpose_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg
)
{
    uav_pose = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_log");
    ros::NodeHandle nh; 

    ros::Subscriber uav_setpt_sub = 
        nh.subscribe<geometry_msgs::PoseStamped>(
            "/nics_uav/setpt", 
            1, 
            &setpt_callback
    );

    ros::Subscriber uav_state_sub = 
        nh.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose", 
            1, 
            &uavpose_callback
    );

    std::string path;
    std::string filename;

//// led path
    nh.getParam("/ctrl_log/log_path", path);
    nh.getParam("/ctrl_log/filename1", filename);
    log_file = path + filename;

    remove(log_file.c_str());

    std::cout<<log_file<<std::endl;

    std::ofstream save(log_file, std::ios::app);
    save<<filename<<std::endl;
    save<<"t,set_x,set_y,set_z,set_ry,uav_x,uav_y,uav_z,uav_ry,e_x,e_y,e_z,e_ry,"<<std::endl;
    save.close();
     
    starting_time = ros::Time::now().toSec();                        

    ros::spin();
    return 0;

}