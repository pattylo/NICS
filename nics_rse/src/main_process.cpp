/*
    This file is part of NICS - the non-robocentric dynamic landing system for quadrotor

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
 * \file main_process.cpp
 * \date 16/08/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#include "nics_rse/vdrse_lib.h"

void nics::VdrseLib::camera_callback(
    const sensor_msgs::CompressedImage::ConstPtr& rgbmsg, 
    const sensor_msgs::Image::ConstPtr& depthmsg
)
{
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


/* ================ Init. utilities function below ================ */

bool nics::VdrseLib::initialization(cv::Mat& frame, cv::Mat depth)
{
    std::get<1>(corres_global_current).clear();

    std::vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth); 
    std::vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

    //after above, I got:
    //pointcloud in {c}
    std::vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

    for(auto what :  pts_3d_pcl_detect)
    {
        norm_of_x_points.push_back(what.x());
        norm_of_y_points.push_back(what.y());
        norm_of_z_points.push_back(what.z());
    }

    // cout<<pts_3d_pcl_detect.size()<<endl;
    // cout<<LED_no<<endl;

    if(pts_3d_pcl_detect.size() == LED_no  //we got LED_no
        && calculate_MAD(norm_of_x_points) < MAD_x_threshold //no outlier
        && calculate_MAD(norm_of_y_points) < MAD_y_threshold 
        && calculate_MAD(norm_of_z_points) < MAD_z_threshold) 
    {
        Sophus::SE3d pose;

        int i = 0;

        //hsv detect color feature
        cv::cvtColor(hsv, hsv, CV_RGB2HSV);
        std::vector<bool> g_or_r; //g = true

        std::vector<int> corres_g;
        std::vector<int> corres_r;

        for(int i = 0 ; i < blobs_for_initialize.size(); i++)
        {
            cv::Point hsv_vertice1 = cv::Point(pts_2d_detect[i].x() - 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() - 2 * blobs_for_initialize[i].size);
            cv::Point hsv_vertice2 = cv::Point(pts_2d_detect[i].x() + 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() + 2 * blobs_for_initialize[i].size);

            cv::Rect letsgethsv(hsv_vertice1, hsv_vertice2);

            cv::Mat ROI(hsv, letsgethsv);

            int size = ROI.cols * ROI.rows;
            
            double accu = 0;

            cv::Vec3b hsv_value;

            for(int i = 0; i < ROI.rows; i++)
            {
                for(int j = 0; j < ROI.cols; j++)
                {
                    hsv_value = ROI.at<cv::Vec3b>(i, j);

                    if(hsv_value[0] == 0)                    
                        size = size - 1;                
                    else
                        accu = accu + hsv_value[0];
                }
            }

            if(accu/size < 100)
                corres_g.push_back(i);
            else   
                corres_r.push_back(i);
        }

        std::vector<int> corres(LED_no);

        if(corres_g.size() != LED_g_no || corres_r.size() != LED_r_no)
        {
            // cout<<"color"<<endl;
            // cout<<corres_g.size()<<endl;
            // cout<<corres_r.size()<<endl;
            return false;
        }

        std::vector<int> final_corres;
        double error_total = INFINITY;

        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        do
        {
            do
            {
                corres.clear();
                for(auto what : corres_g)
                    corres.push_back(what);
                for(auto what : corres_r)
                    corres.push_back(what);

                std::vector<Eigen::Vector2d> pts_2d_detect_temp;   

                for(auto what : corres)
                {
                    pts_2d_detect_temp.push_back(pts_2d_detect[what]);
                }
                                                        
                solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame);
                
                pose_global_sophus = pose_epnp_sophus;

                double e = get_reprojection_error(                    
                    pts_on_body_frame,
                    pts_2d_detect_temp,
                    pose_global_sophus,
                    false
                );

                if(e < error_total)
                {                    
                    error_total = e;
                    final_corres = corres;
                    
                    // if(error_total < 5)
                    //     break;
                }                        
            } while (next_permutation(corres_r.begin(), corres_r.end()));

        } while(next_permutation(corres_g.begin(), corres_g.end()));

                
        BA_error = error_total;

        if(BA_error > LED_no * 2)
        {
            ROS_WARN("HELLO?");
            return false;
        }

        correspondence::matchid corres_temp;
        
        pts_2d_detect_correct_order.clear();
        
        for(auto what : final_corres)
        {
            corres_temp.detected_indices = what;
            corres_temp.detected_ornot = true;
            corres_temp.pts_3d_correspond = pts_3d_pcl_detect[what];            
            corres_temp.pts_2d_correspond = pts_2d_detect[what];

            pts_2d_detect_correct_order.push_back(pts_2d_detect[what]); 

            std::get<1>(corres_global_current).push_back(corres_temp);
        }

        if(std::get<1>(corres_global_current).size() != LED_no)
        {
            ROS_RED_STREAM("PLEASE DEBUG");

        }

        camOptimize(
            pose_global_sophus, 
            pts_on_body_frame, 
            pts_2d_detect_correct_order,
            BA_error
        );

        detect_no = 6;
        std::get<0>(corres_global_current) = detect_no;
        corres_global_previous = corres_global_current;

        return true;
    }
    else
        return false;
}
