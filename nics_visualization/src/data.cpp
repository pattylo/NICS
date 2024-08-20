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
 * \file data.cpp
 * \date 16/02/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief executable to record data
*/

#include <ros_utilities/ros_utilities.h>

#include "nics_rse/vdrse_log.h"

static ros_utilities ros_tools;

static std::string log_file_1;
static std::string log_file_2;
static double starting_time;
static geometry_msgs::PoseStamped ugv_pose;
static geometry_msgs::TwistStamped ugv_vel;

static geometry_msgs::TwistStamped uav_vel;

void uav_vel_callback(
    const geometry_msgs::TwistStamped::ConstPtr& msg
)
{
    uav_vel = *msg;
}

void ugv_vel_callback(
    const geometry_msgs::TwistStamped::ConstPtr& msg
)
{
    ugv_vel = *msg;
}

void ugv_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg
)
{
    ugv_pose = *msg;
}

void msg_callback(
    const nics_rse::vdrse_log::ConstPtr& ledmsg,
    const nics_rse::vdrse_log::ConstPtr& uavmsg
)
{    
    std::ofstream save(log_file_1, std::ios::app);

    Eigen::Vector3d posi_est = Eigen::Vector3d(ledmsg->px, ledmsg->py, ledmsg->pz);
    
    
    Sophus::SE3d rot_ugv = ros_tools.posemsg_to_SE3(ugv_pose.pose);

    Eigen::Vector3d velo_est = Eigen::Vector3d(ledmsg->vx, ledmsg->vy, ledmsg->vz);
    Eigen::Vector3d velo_ugv = Eigen::Vector3d(ugv_vel.twist.linear.x, ugv_vel.twist.linear.y, 0.0);
    std::cout<<ugv_vel.twist.angular.z * 280<<std::endl<<std::endl;
    // std::cout<<velo_est * ugv_vel.twist.angular.z * 280<<std::endl<<std::endl;
    std::cout<<rot_ugv.rotationMatrix().inverse() * posi_est<<std::endl<<std::endl;

    velo_est = 
        velo_est 
        + 
        velo_ugv ;
        // + 
        // rot_ugv.rotationMatrix()
        // *
        // 280 
        //     * Eigen::Vector3d(
        //         ugv_vel.twist.angular.x,
        //         ugv_vel.twist.angular.y,
        //         ugv_vel.twist.angular.z
        //     ).cross(rot_ugv.rotationMatrix().inverse() * posi_est);

    std::cout<<"percentage here:"<< 
        (
            280 
            * Eigen::Vector3d(
                ugv_vel.twist.angular.x,
                ugv_vel.twist.angular.y,
                ugv_vel.twist.angular.z
            ).cross(rot_ugv.rotationMatrix().inverse() * posi_est)
        ).norm()
        / velo_est.norm() * 100 <<std::endl<<std::endl;;
    // std::cout<<"velo"<<std::endl;
    // std::cout<<rot_ugv.rotationMatrix().inverse() * posi_est * ugv_vel.twist.angular.z * 280<<std::endl<<std::endl;

    
    
    // save<<"x,y,z,r,p,y,ori,vx,vy,vz,
        // st_x,st_y,st_z,
        // ugvx,ugvy,ugvz,ugv_vx,ugv_vy,ugv_vyaw,
        // ms,dpth,t,"<<std::endl;
    // estimate -> setpt -> ugv -> misc
    save << ledmsg->px << "," 
         << ledmsg->py << "," 
         << ledmsg->pz << ","
         << ledmsg->roll  << ","
         << ledmsg->pitch << ","
         << ledmsg->yaw   << ","
         << ledmsg->orientation << ","
         
        //  << ledmsg->vx << "," 
        //  << ledmsg->vy << "," 
        //  << ledmsg->vz << "," 
         << velo_est.x() << "," 
         << velo_est.y() << "," 
         << velo_est.z() << "," 

         << ledmsg->set_px << ","
         << ledmsg->set_py << ","
         << ledmsg->set_pz << ","

         << ugv_pose.pose.position.x << ","
         << ugv_pose.pose.position.y << "," 
         << ugv_pose.pose.position.z << "," 
         << ugv_vel.twist.linear.x << ","
         << ugv_vel.twist.linear.y << ","
         << ugv_vel.twist.angular.z << ","

         << ledmsg->ms << ","
         << ledmsg->depth << ","
         << ledmsg->header.stamp.now().toSec() - starting_time << ","
         << std::endl;
    save.close();


    save = std::ofstream(log_file_2, std::ios::app);
    // save<<"x,y,z,r,p,y,ori,vx,vy,vz,ms,dpth,t"<<std::endl;
    save << uavmsg->px << "," 
         << uavmsg->py << "," 
         << uavmsg->pz << ","
         << uavmsg->roll  << ","
         << uavmsg->pitch << ","
         << uavmsg->yaw   << ","
         << uavmsg->orientation << ","
         << uav_vel.twist.linear.x << ","
         << uav_vel.twist.linear.y << ","
         << uav_vel.twist.linear.z << ","
         << uavmsg->ms << ","
         << uavmsg->depth << ","
         << ledmsg->header.stamp.now().toSec() - starting_time << "," << std::endl;
    
    save.close();

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_log");
    ros::NodeHandle nh;

    message_filters::Subscriber<nics_rse::vdrse_log> subled;
    message_filters::Subscriber<nics_rse::vdrse_log> subuav;
    typedef message_filters::sync_policies::ExactTime<nics_rse::vdrse_log, nics_rse::vdrse_log> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
    boost::shared_ptr<sync> sync_;   

    subled.subscribe(nh, "/alan_state_estimation/led/led_log", 1);                
    subuav.subscribe(nh, "/alan_state_estimation/led/uav_log", 1);                
    sync_.reset(new sync( MySyncPolicy(10), subled, subuav));
    sync_->registerCallback(boost::bind(&msg_callback, _1, _2));   

    std::string ugv_topic;
    nh.getParam("/data/ugv_topic", ugv_topic);
    std::cout<<ugv_topic<<std::endl;
    ros::Subscriber ugv_sub = 
        nh.subscribe<geometry_msgs::PoseStamped>(ugv_topic, 1, &ugv_callback);
    ros::Subscriber ugv_vel_sub = 
        nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/gh034_scout_fast/twist", 1, &ugv_vel_callback);
    ros::Subscriber uav_vel_sub = 
        nh.subscribe<geometry_msgs::TwistStamped>("/uav/mavros/local_position/velocity_local", 1, &uav_vel_callback);
    

    std::string path;
    std::string filename;

//// led path
    nh.getParam("/data/log_path", path);
    nh.getParam("/data/filename1", filename);
    log_file_1 = path + filename;

    remove(log_file_1.c_str());

    std::cout<<log_file_1<<std::endl;

    std::ofstream save(log_file_1, std::ios::app);
    save<<filename<<std::endl;
    save<<"x,y,z,r,p,y,ori,vx,vy,vz,st_x,st_y,st_z,ugvx,ugvy,ugvz,ugv_vx,ugv_vy,ugv_vyaw,ms,dpth,t,"<<std::endl;
    // estimate -> setpt -> ugv -> misc
    save.close();

//// uav path
    nh.getParam("/data/log_path", path);
    nh.getParam("/data/filename2", filename);
    log_file_2 = path + filename;

    remove(log_file_2.c_str());

    std::cout<<log_file_2<<std::endl;

    save = std::ofstream(log_file_2, std::ios::app);
    save<<filename<<std::endl;
    save<<"x,y,z,r,p,y,ori,vx,vy,vz,ms,dpth,t,"<<std::endl;
    save.close();
     

    starting_time = ros::Time::now().toSec();                        

    ros::spin();
    return 0;

}