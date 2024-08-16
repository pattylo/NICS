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
 * \file ledvo_nodelet.h
 * \date 16/08/2024
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief lib - led
*/

#include "nics_rse/vdrse_lib.h"


void nics::VdrseLib::map_SE3_to_publish(
    Sophus::SE3d pose_led_inCamera_SE3,
    Sophus::SE3d velo_led_inCamera_SE3,
    Eigen::MatrixXd cov_inCamera_SE3
)
{
    pose_led_inWorld_SE3 = 
        pose_cam_inWorld_SE3 
        * pose_cam_inGeneralBodySE3 
        * pose_led_inCamera_SE3;
    
    Sophus::SE3d temp = pose_cam_inWorld_SE3;
    
    std::cout<< velo_led_inCamera_SE3.translation()<<std::endl<<std::endl;
    std::cout<<pose_cam_inGeneralBodySE3.rotationMatrix()<<std::endl<<std::endl;
    std::cout<< pose_cam_inGeneralBodySE3.rotationMatrix() * velo_led_inCamera_SE3.translation()<<std::endl<<std::endl;
    
    temp.translation().setZero();
    velo_led_inWorld_SE3 = 
        temp
        * pose_cam_inGeneralBodySE3
        * velo_led_inCamera_SE3;

    std::cout<<pose_led_inWorld_SE3.translation()<<std::endl<<std::endl;;
    std::cout<<pose_uav_inWorld_SE3.translation()<<std::endl<<std::endl;    
    std::cout<<(pose_led_inWorld_SE3.translation() - pose_uav_inWorld_SE3.translation()).norm()<<std::endl;
    std::cout<<"huh"<<std::endl;
    
    led_pose_header.frame_id = "world";
    led_pose_estimated_msg = SE3_to_posemsg(
        pose_led_inWorld_SE3, 
        led_pose_header
    );

    ledpose_pub.publish(led_pose_estimated_msg);

    //odom publish
    led_odom_estimated_msg = SE3_to_odommsg(
        pose_led_inWorld_SE3,
        Sophus::Vector6d(
            // velo_led_inWorld_SE3.translation(), 
            // velo_led_inWorld_SE3.log().tail(3)
        ),
        led_pose_header
    );
    
    ledodom_pub.publish(led_odom_estimated_msg);
}

void nics::VdrseLib::set_image_to_publish(double freq, const sensor_msgs::CompressedImageConstPtr & rgbmsg)
{    
    char hz[40];
    char fps[10] = " fps";
    sprintf(hz, "%.2f", freq);
    strcat(hz, fps);
    // lala.

    char BA[40] = "BA: ";
    char BA_error_display[10];
    sprintf(BA_error_display, "%.2f", BA_error);
    strcat(BA, BA_error_display);

    char depth[40] = "DPTH: ";
    char depth_display[10];
    sprintf(depth_display, "%.2f", depth_avg_of_all);
    strcat(depth, depth_display);
    
    cv::putText(display, hz, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));  
    cv::putText(display, std::to_string(detect_no), cv::Point(720,460), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, BA, cv::Point(720,60), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, depth, cv::Point(20,460), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));

    cv::Mat imageoutput = display.clone();
    cv_bridge::CvImage for_visual;
    for_visual.header = rgbmsg->header;
    for_visual.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual.image = imageoutput;
    this->pubimage.publish(for_visual.toImageMsg());


    cv_bridge::CvImage for_visual_input;
    for_visual_input.header = rgbmsg->header;
    for_visual_input.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual_input.image = frame_input;
    this->pubimage_input.publish(for_visual_input.toImageMsg());   

}

void nics::VdrseLib::log(double ms)
{
    nics_rse::vdrse_log logdata_entry_led;
    
    logdata_entry_led.px = pose_led_inWorld_SE3.translation().x();
    logdata_entry_led.py = pose_led_inWorld_SE3.translation().y();
    logdata_entry_led.pz = pose_led_inWorld_SE3.translation().z();

    logdata_entry_led.set_px = uav_stpt_msg.pose.position.x;
    logdata_entry_led.set_py = uav_stpt_msg.pose.position.y;
    logdata_entry_led.set_pz = uav_stpt_msg.pose.position.z;

    logdata_entry_led.vx = velo_led_inWorld_SE3.translation().x();
    logdata_entry_led.vy = velo_led_inWorld_SE3.translation().y();
    logdata_entry_led.vz = velo_led_inWorld_SE3.translation().z();
    
    Eigen::Vector3d rpy ;
    // = q2rpy(
        // Eigen::Quaterniond(pose_led_inWorld_SE3.rotationMatrix())
    // );

    logdata_entry_led.roll  = rpy(0);
    logdata_entry_led.pitch = rpy(1);
    logdata_entry_led.yaw   = rpy(2);

    Eigen::AngleAxisd angle_axis_led = Eigen::AngleAxisd(pose_led_inWorld_SE3.rotationMatrix());
    logdata_entry_led.orientation = angle_axis_led.angle();

    logdata_entry_led.ms = ms;
    logdata_entry_led.depth = abs(
        sqrt(
            pow(
                pose_cam_inWorld_SE3.translation().x() - pose_uav_inWorld_SE3.translation().x(),
                2
            ) +
            pow(pose_cam_inWorld_SE3.translation().y() - pose_uav_inWorld_SE3.translation().y(),
                2
            ) +
            pow(pose_cam_inWorld_SE3.translation().z() - pose_uav_inWorld_SE3.translation().z(),
                2
            )
        )        
    );

    logdata_entry_led.ms = detect_no;

    logdata_entry_led.header.stamp = led_pose_header.stamp;

    record_led_pub.publish(logdata_entry_led);

    ///////////////////////////////////////////////////////////

    nics_rse::vdrse_log logdata_entry_uav;
    
    logdata_entry_uav.px = pose_uav_inWorld_SE3.translation().x();
    logdata_entry_uav.py = pose_uav_inWorld_SE3.translation().y();
    logdata_entry_uav.pz = pose_uav_inWorld_SE3.translation().z();
    
    rpy = q2rpy(
        Eigen::Quaterniond(pose_uav_inWorld_SE3.rotationMatrix())
    );

    logdata_entry_uav.roll  = rpy(0);
    logdata_entry_uav.pitch = rpy(1);
    logdata_entry_uav.yaw   = rpy(2);

    Eigen::AngleAxisd angle_axis_uav = Eigen::AngleAxisd(pose_uav_inWorld_SE3.rotationMatrix());
    logdata_entry_uav.orientation = angle_axis_uav.angle();

    logdata_entry_uav.header.stamp = led_pose_header.stamp;

    record_uav_pub.publish(logdata_entry_uav);
}

void nics::VdrseLib::terminal_msg_display(double hz)
{
    std::string LED_terminal_display = "DETECT_no: " + std::to_string(detect_no);

    std::ostringstream out1;
    out1.precision(2);
    out1<<std::fixed<<BA_error;
    std::string BA_terminal_display = " || BA_ERROR: " + out1.str();

    std::ostringstream out2;
    out2.precision(2);
    out2<<std::fixed<<depth_avg_of_all;
    std::string depth_terminal_display = " || depth: " + out2.str();

    std::ostringstream out3;
    out3.precision(2);
    out3<<std::fixed<<hz;
    std::string hz_terminal_display = " || hz: " + out3.str();

    std::string final_msg = LED_terminal_display 
        + BA_terminal_display 
        + depth_terminal_display
        + hz_terminal_display;

    std::string LED_tracker_status_display;

    if(LED_tracker_initiated_or_tracked)
    {
        final_msg = "LED GOOD || " + final_msg;
        ROS_GREEN_STREAM(final_msg);
    }
    else
    {
        final_msg = "LED BAD! || " + final_msg;
        ROS_RED_STREAM(final_msg);
        if(tracker_started)
            error_no ++;
    }
    std::cout<<"fail: "<< error_no<<" / "<<total_no<<std::endl;
}

void nics::VdrseLib::record_ms()
{
    std::ofstream save(
        "/home/patty/alan_rebuttal_ws/src/ALAN_rebuttal/alan_state_estimation/log/ms.csv", 
        std::ios::app
    );
    save << 
        init_ms << "," <<
        image_pre_ms << "," <<
        corres_ms << "," <<
        iekf_ms << "," <<
        std::endl;
    save.close();

}
