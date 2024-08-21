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
 * \file sys_id.cpp
 * \date 16/08/2024
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief lib - main process here
*/

#include <ros_utilities/ros_utilities.h>

// static ros_utilities ros_tools;
static std::shared_ptr<ros_utilities> ros_tools_ptr;

static std::deque<double> thrust_buff;
static std::mutex thrust_buff_manage;
static double thrust_avg;

void thrust_callback(
    const mavros_msgs::AttitudeTarget::ConstPtr& msg
)
{
    thrust_buff.push_back(
        msg->thrust
    );  
}

void calculate_thrust()
{
    double thrust_mean;

    ros_tools_ptr->ComputeMeanScalar(thrust_buff, thrust_mean);
    std::cout<<"Thrust Mean: "<<thrust_mean<<std::endl;
}  

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_extrinsic");
    ros::NodeHandle nh; 

    ros_tools_ptr = std::make_shared<ros_utilities>();

    ros::Subscriber input_msg_sub = 
        nh.subscribe<mavros_msgs::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 
            1, 
            &thrust_callback
    );

    ros::spin();
    calculate_thrust();

    return 0;

}