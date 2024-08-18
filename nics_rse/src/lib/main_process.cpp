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
 * \brief lib - main process here
*/

#include "nics_rse/vdrse_lib.h"

void nics::VdrseLib::camera_callback(
    const sensor_msgs::CompressedImage::ConstPtr& rgbmsg, 
    const sensor_msgs::Image::ConstPtr& depthmsg
)
{
    ROS_INFO("CAM");
    return;
    cv_bridge::CvImageConstPtr depth_ptr;
    led_pose_header = rgbmsg->header;

    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depthmsg, depthmsg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth = depth_ptr->image;

    try
    {
        frame = cv::imdecode(cv::Mat(rgbmsg->data), 1);
        display = frame.clone();
        hsv = frame.clone();
        frame_temp = frame.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    double tick = ros::Time::now().toSec(); 

    if(tick - last_request > ros::Duration(0.4).toSec() && nodelet_activated)
    {
        LED_tracker_initiated_or_tracked = false;
        printf("\033c");
        ROS_RED_STREAM("RESET TERMINAL!");
    }           

    std::cout<<frame.cols<<std::endl;
    std::cout<<frame.rows<<std::endl;

    std::cout<<depth.cols<<std::endl;
    std::cout<<depth.rows<<std::endl;

           
    solve_pose_w_LED(frame, depth);
    
    double tock = ros::Time::now().toSec();  

    if(tracker_started)
        total_no++;

    terminal_msg_display(1 / (tock - tick));

    set_image_to_publish(1 / (tock - tick), rgbmsg);

    if(LED_tracker_initiated_or_tracked)
    {
        log(tock - tick);
        record_ms();
    }
        
                    
    last_request = ros::Time::now().toSec();

    if(!nodelet_activated)
        nodelet_activated = true;

    led_pose_header_previous = led_pose_header;

} 

void nics::VdrseLib::eso_mainspinCallback(
    const ros::TimerEvent& e
)
{
    if(!eso_activated)
    {   
        t_eso_prev = ros::Time::now().toSec();
        return;
    }
        
    ROS_CYAN_STREAM("ESO HERE");

    set_dz();
    update_z();
    pub_z();
}

void nics::VdrseLib::solve_pose_w_LED(cv::Mat& frame, cv::Mat depth)
{    
    if(!LED_tracker_initiated_or_tracked)        
    {
        double tick0 = ros::Time::now().toSec();
        LED_tracker_initiated_or_tracked = initialization(frame, depth);
        // try to initialize here...
        double tock0 = ros::Time::now().toSec();
        init_ms = (tock0 - tick0) * 1000;
        std::cout<<"INITIALIZATION"<<init_ms<<std::endl;
        // ros::shutdown();

        if(LED_tracker_initiated_or_tracked)
        {
            printf("\n");
            ROS_GREEN_STREAM("TRACKER INITIALIZED!");
            ROS_GREEN_STREAM("SHOULD BE FINE...HOPEFULLY?\n");
            tracker_started = true;

            if(BA_error > LED_no * 2)
            {
                ROS_WARN("REPROJECTION_ERROR OVER @ INITIALIZATION %d", LED_no * 2);          
            }                    

            if(!kf_initiated)
            {
                kf_initiated = true;
                apiKF(kfINITIATE);                
            }
            else
                apiKF(kfREINITIATE);            
            
            map_SE3_to_publish(
                pose_global_sophus, 
                velo_global_sophus,
                covariance_global_sophus
            );
        }
        else
        {
            ROS_CYAN_STREAM("WAITING FOR INITIALIZATION...");
        }
    }
    else
    {
        recursive_filtering(frame, depth);

        if(!LED_tracker_initiated_or_tracked)
            ROS_RED_STREAM("TRACKER FAIL");
        else       
            map_SE3_to_publish(
                pose_global_sophus, 
                velo_global_sophus,
                covariance_global_sophus
            );
    }

}

void nics::VdrseLib::apiKF(int DOKF)
{
    switch (DOKF)
    {
    case kfINITIATE:
        initKF(pose_global_sophus);

        pose_global_sophus = XcurrentPosterori.X_SE3;
        velo_global_sophus = XcurrentPosterori.V_SE3;
        covariance_global_sophus = XcurrentPosterori.PCov;
        break;
    
    case kfREINITIATE:
        reinitKF(pose_global_sophus);

        pose_global_sophus = XcurrentPosterori.X_SE3;
        velo_global_sophus = XcurrentPosterori.V_SE3;
        covariance_global_sophus = XcurrentPosterori.PCov;
        break;

    case kfNORMALKF:
        run_AIEKF(
            led_pose_header.stamp.toSec() - led_pose_header_previous.stamp.toSec(),
            pts_on_body_frame_in_corres_order, 
            pts_detected_in_corres_order
        );

        pose_global_sophus = XcurrentPosterori.X_SE3;
        velo_global_sophus = XcurrentPosterori.V_SE3;
        covariance_global_sophus = XcurrentPosterori.PCov;

        break;
    
    default:
        // pc::pattyDebug();
        break;
    }
}

void nics::VdrseLib::recursive_filtering(cv::Mat& frame, cv::Mat depth)
{
    std::vector<Eigen::Vector2d> pts_2d_detect;

    double tick1 = ros::Time::now().toSec();
    pts_2d_detect = LED_extract_POI_alter(frame, depth);
    double tock1 = ros::Time::now().toSec();
    image_pre_ms = (tock1 - tick1) * 1000;

    detect_no = pts_2d_detect.size();

    if(detect_no < 3)
    {
        // std::cout<<i<<std::endl;
        LED_tracker_initiated_or_tracked = false;
        cv::imwrite("/home/patty/alan_ws/meas_less3" + std::to_string(detect_no) + "_"+  std::to_string(i) + ".jpg", frame_input);
        // i++;
        return;
    }

    double tick2 = ros::Time::now().toSec();
    get_correspondence(pts_2d_detect);
    double tock2 = ros::Time::now().toSec();
    corres_ms = (tock2 - tick2) * 1000;

    pointcloud_generate(pts_2d_detect, depth);
    detect_no = pts_detected_in_corres_order.size();

    if(detect_no < 3)
    {
        // std::cout<<i<<std::endl;
        LED_tracker_initiated_or_tracked = false;
        // cv::imwrite("/home/patty/alan_ws/kmeans_less3_" + std::to_string(detect_no) + "_"+ std::to_string(i) + ".jpg", frame_input);
        // i++;
        return;
    }

    double tick3 = ros::Time::now().toSec();
    apiKF(kfNORMALKF);
    double tock3 = ros::Time::now().toSec();
    iekf_ms = (tock3 - tick3) * 1000;

    BA_error = get_reprojection_error(
        pts_on_body_frame_in_corres_order,
        pts_detected_in_corres_order,
        pose_global_sophus,
        true
    );

    if(BA_error > 4 * LED_no)
    {
        LED_tracker_initiated_or_tracked = false;
        // cv::imwrite("/home/patty/alan_ws/BA" + std::to_string(BA_error) + ".jpg", frame_input);
    }
}
