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
 * \brief lib header
*/

#ifndef VDRSE_H
#define VDRSE_H

#include <opencv2/calib3d.hpp>
#include <ros_utilities/ros_utilities.h>
#include "aiekf.hpp"
#include "nics_rse/vdrse_log.h"

// map definition for convinience
#define COLOR_SUB_TOPIC CAMERA_SUB_TOPIC_A
#define DEPTH_SUB_TOPIC CAMERA_SUB_TOPIC_B
#define UAV_POSE_SUB_TOPIC POSE_SUB_TOPIC_A
#define UGV_POSE_SUB_TOPIC POSE_SUB_TOPIC_B
#define UGV_VELO_SUB_TOPIC TWIST_SUB_TOPIC_A

#define LED_POSE_PUB_TOPIC POSE_PUB_TOPIC_A
#define UGV_POSE_PUB_TOPIC POSE_PUB_TOPIC_B
#define UAV_POSE_PUB_TOPIC POSE_PUB_TOPIC_C
#define CAM_POSE_PUB_TOPIC POSE_PUB_TOPIC_D

#define LED_ODOM_PUB_TOPIC ODOM_PUB_TOPIC_A

namespace correspondence
{
    typedef struct matchid
        {
            int detected_indices; //
            bool detected_ornot = false;
            Eigen::Vector3d pts_3d_correspond;
            Eigen::Vector2d pts_2d_correspond;
        }matchid;
}

namespace nics
{
    class VdrseLib : private kf::aiekf, private ros_utilities
    {
    public:
        VdrseLib(ros::NodeHandle& nh) : _nh(nh)
        {
            ROS_INFO("VDRSE Nodelet Initiated...");

            doALOTofConfigs(nh);
        };
        ~VdrseLib(){};

    private:
        ros::NodeHandle _nh;

//---------------------------------------------------------------------------------------
// config.cpp
            //extrinsics
            Eigen::VectorXd LEDEX;
            Eigen::VectorXd cameraEX;
            
            //ros related
            ros::Subscriber ugv_pose_sub, ugv_velo_sub, uav_pose_sub, uav_setpt_sub;
            ros::Publisher ledpose_pub, ledodom_pub, 
                           campose_pub, ugvpose_pub, uavpose_pub,
                           record_led_pub, record_uav_pub;
            image_transport::Publisher pubimage;
            image_transport::Publisher pubimage_input;
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;// (MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;                    

            //functions
            void doALOTofConfigs(ros::NodeHandle& nh);
        
            void POI_config(ros::NodeHandle& nh);
            void camIntrinsic_config(ros::NodeHandle& nh);
            void camExtrinsic_config(ros::NodeHandle& nh);
            void CamInGeneralBody_config(ros::NodeHandle& nh);
            void LEDInBodyAndOutlierSetting_config(ros::NodeHandle& nh);              
            void KF_config(ros::NodeHandle& nh);
            void ESO_config(ros::NodeHandle& nh);
            void subpub_config(ros::NodeHandle& nh);
            inline void other_config(ros::NodeHandle& nh){};

            void ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void ugv_velo_callback(const geometry_msgs::TwistStamped::ConstPtr& pose);
            void uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_setpt_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void u_callback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
//---------------------------------------------------------------------------------------
// eso.cpp
            Eigen::VectorXd z_I = Eigen::VectorXd::Zero(9);
            Eigen::VectorXd dz_I = Eigen::VectorXd::Zero(9);
            Eigen::Vector3d dist_I;
            
            Eigen::MatrixXd A_eso;
            Eigen::MatrixXd B_eso;
            Eigen::MatrixXd L_eso;
            Eigen::MatrixXd C_eso;
            Eigen::VectorXd u_input_I;
            Eigen::VectorXd y_eso;
            double t_eso;
            double t_eso_prev;
            double hover_thrust;
            double g = 9.81;
            geometry_msgs::PointStamped z_dist;
            
            ros::Subscriber u_sub;
            ros::Publisher z_dist_pub;
            ros::Timer eso_spinner;

            bool eso_activated = false;

            void eso_mainspinCallback(const ros::TimerEvent &e);
            
            void set_dz();
            void update_z();
            void get_gain();
            void pub_z();
//---------------------------------------------------------------------------------------
// led.cpp
            //objects
            bool LED_tracker_initiated_or_tracked = false;
            double MAD_dilate, MAD_max;
            double MAD_x_threshold = 0, MAD_y_threshold = 0, MAD_z_threshold = 0;
            double min_blob_size = 0;
            int LED_no;
            std::vector<Eigen::Vector2d> pts_2d_detect_correct_order;
            cv::Point3f pcl_center_point_wo_outlier_previous;
            Eigen::Vector3d led_3d_posi_in_camera_frame_depth;
            double EFFECTIVE_DISTANCE = 0;
            int BINARY_THRES = 0;

            std::vector<Eigen::Vector3d> pts_on_body_frame_in_corres_order;
            std::vector<Eigen::Vector2d> pts_detected_in_corres_order;
            
            //functions
            // initialization
            bool initialization(cv::Mat& frame, cv::Mat depth);
            void solve_pnp_initial_pose(
                std::vector<Eigen::Vector2d> pts_2d, 
                std::vector<Eigen::Vector3d> body_frame_pts
            );
            double calculate_MAD(std::vector<double> norm_of_points);

            // feature
            std::vector<Eigen::Vector2d> LED_extract_POI(cv::Mat& frame, cv::Mat depth);
            std::vector<Eigen::Vector2d> LED_extract_POI_alter(cv::Mat& frame, cv::Mat depth);
            std::vector<Eigen::Vector3d> pointcloud_generate(std::vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage);

            // correspondence
            void get_correspondence(
                std::vector<Eigen::Vector2d>& pts_2d_detected
            );
            std::vector<Eigen::Vector2d> shift2D(
                std::vector<Eigen::Vector2d>& pts_2D_previous,
                std::vector<Eigen::Vector2d>& pts_detect_current
            );
            void correspondence_search_2D2DCompare(
                std::vector<Eigen::Vector2d>& pts_2d_detected,
                std::vector<Eigen::Vector2d>& pts_2d_detected_previous
            );      

//---------------------------------------------------------------------------------------
// log.cpp
            //objects
            int error_no = 0;
            int total_no = 0;
            int detect_no = 0;
            double BA_error = 0;
            double depth_avg_of_all = 0;
            geometry_msgs::PoseStamped led_pose_estimated_msg;
            nav_msgs::Odometry led_odom_estimated_msg;
            double init_ms = 0;
            double image_pre_ms = 0;
            double corres_ms = 0;
            double iekf_ms = 0;

            bool DRSE_ON = false;
            bool ALL_INPUT_ON = false;

            //functions
            void map_SE3_to_publish(
                Sophus::SE3d pose, 
                Sophus::SE3d velo,
                Eigen::MatrixXd cov
            );
            void set_image_to_publish(
                double hz, 
                const sensor_msgs::CompressedImageConstPtr & rgbmsg
            );
            void log(double ms);   
            void terminal_msg_display(double hz);
            void record_ms();   
            double get_reprojection_error(
                std::vector<Eigen::Vector3d> pts_3d, 
                std::vector<Eigen::Vector2d> pts_2d, 
                Sophus::SE3d pose, 
                bool draw_reproject
            ) override
            {
                double e = 0;

                Eigen::Vector2d reproject, error;

                for(int i = 0; i < pts_3d.size(); i++)
                {
                    reproject = reproject_3D_2D(pts_3d[i], pose);
                    error = pts_2d[i] - reproject;
                    e = e + error.norm();

                    if(draw_reproject)
                        cv::circle(display, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);                    
                }

                return e;
            };   

//---------------------------------------------------------------------------------------
// main_process.cpp
            //poses
            Sophus::SE3d pose_global_sophus;
            Sophus::SE3d velo_global_sophus;
            Eigen::MatrixXd covariance_global_sophus;
            Sophus::SE3d pose_epnp_sophus, pose_depth_sophus;
            Sophus::SE3d pose_cam_inWorld_SE3;
            Sophus::SE3d pose_ugv_inWorld_SE3;
            Sophus::Vector6d velo_ugv_inWorld_SE3;
            Sophus::SE3d pose_uav_inWorld_SE3;
            Sophus::SE3d pose_led_inWorld_SE3;
            Sophus::SE3d velo_led_inWorld_SE3;
            Sophus::SE3d pose_cam_inGeneralBodySE3;
            Sophus::SE3d pose_cam_inUgvBody_SE3;
            geometry_msgs::PoseStamped ugv_pose_msg, 
                                       uav_pose_msg,
                                       uav_stpt_msg; 
            Eigen::VectorXd led_twist_current;

            //misc
            int i = 0;
            bool nodelet_activated = false;
            bool tracker_started = false;
            std_msgs::Header led_pose_header, led_pose_header_previous;
            double last_request = 0;

            //LED config and correspondences
            std::vector<Eigen::Vector3d> pts_on_body_frame;
                // detect_no, correspondences(in order of 0->5)
            std::tuple<int, std::vector<correspondence::matchid>> corres_global_current;
            std::tuple<int, std::vector<correspondence::matchid>> corres_global_previous;
            std::vector<cv::KeyPoint> blobs_for_initialize;
            
            //frames
            cv::Mat frame, display, hsv, frame_temp;
            cv::Mat frame_input;
            cv::Mat im_with_keypoints;
            cv::Mat frame_initial_thresholded;
            // int _width = 0, _height = 0;
            
            //functions
            void camera_callback(const sensor_msgs::CompressedImage::ConstPtr & rgbimage, const sensor_msgs::Image::ConstPtr & depth);
            void solve_pose_w_LED(cv::Mat& frame, cv::Mat depth);
            void apiKF(int DOKF);
            void recursive_filtering(cv::Mat& frame, cv::Mat depth); 
    };
}

#endif