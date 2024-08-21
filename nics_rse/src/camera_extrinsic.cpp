/*
    This file is part of NICS - a non-inertial control system

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
 * \file camera_extrinsic.cpp
 * \date 16/08/2024
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief lib - main process here
*/

#include <ros_utilities/ros_utilities.h>

// static ros_utilities ros_tools;
static std::shared_ptr<ros_utilities> ros_tools_ptr;

static std::deque<Eigen::Vector3d> imu_buff;
static std::mutex imu_buff_manage;
static Eigen::Matrix<double, 3, 3> R_cam_to_body;

static Eigen::Vector3d R_Euler;
static Eigen::Vector3d acc_B;
static double pitch, roll;

static Eigen::Vector3d acc_bias;

void imu_callback(
    const sensor_msgs::Imu::ConstPtr& msg
)
{
    imu_buff.push_back(
        Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        ) 
        - 
        acc_bias
    );  
}

void calculate_extrinsic()
{
    Eigen::Vector3d acc_mean, imu_cov;

    ros_tools_ptr->ComputeMeanVector(imu_buff, acc_mean);

    acc_B = R_cam_to_body * acc_mean;

    std::cout<<acc_B<<std::endl<<std::endl;

    for (int i = 0; i < acc_B.size(); i++)
        if(abs(acc_B(i)) < 0.1)
            acc_B(i) = abs(acc_B(i));

    std::cout<<acc_B<<std::endl<<std::endl;

    roll = atan2(acc_B.y(), acc_B.z());
    pitch = atan2( 
        - acc_B.x(), 
        sqrt(pow(acc_B.y(),2) + pow(acc_B.z(),2))
    );
    
    std::cout<<std::endl<<"COLLECTED IMU DATA SIZE: "<<imu_buff.size()<<std::endl;
    std::cout<<"RESULTS: "<<std::endl;
    std::cout<<"roll: "<< 0.0 * roll / M_PI * 180.0<<std::endl;
    std::cout<<"pitch: "<<pitch / M_PI * 180.0<<std::endl<<std::endl;

    std::cout<<"rot_mat\n"<<ros_tools_ptr->rpy2q(Eigen::Vector3d(0.0, pitch, M_PI)).toRotationMatrix()<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_extrinsic");
    ros::NodeHandle nh; 

    ros_tools_ptr = std::make_shared<ros_utilities>();

    ros::Subscriber imu_msg_sub = 
        nh.subscribe<sensor_msgs::Imu>(
            "/camera/imu", 
            1, 
            &imu_callback
    );

    acc_bias = Eigen::Vector3d(0.121071, -0.136015, 0.0312915);
    acc_bias.setZero();
    
    R_cam_to_body << 
        0,0,1,
        -1,0,0,
        0,-1,0;

    ros::spin();
    calculate_extrinsic();

    return 0;

}