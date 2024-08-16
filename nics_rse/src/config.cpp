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
 * \brief lib - configuration
*/

#include "nics_rse/vdrse_lib.h"

void nics::VdrseLib::doALOTofConfigs(ros::NodeHandle& nh)
{
	POI_config(nh);
	camIntrinsic_config(nh);
	camExtrinsic_config(nh);
	CamInGeneralBody_config(nh);
	LEDInBodyAndOutlierSetting_config(nh);
	KF_config(nh);
	subpub_config(nh);
	other_config(nh);
}

void nics::VdrseLib::POI_config(ros::NodeHandle& nh)
{
	// load POI_extract config
	nh.getParam("/nics_master/LANDING_DISTANCE", LANDING_DISTANCE);     
	nh.getParam("/nics_master/BINARY_threshold", BINARY_THRES);     
	nh.getParam("/nics_master/frame_width", _width);
	nh.getParam("/nics_master/frame_height", _height);
}

void nics::VdrseLib::camIntrinsic_config(ros::NodeHandle& nh)
{
	// load camera intrinsics
	Eigen::Vector4d intrinsics_value;
	XmlRpc::XmlRpcValue intrinsics_list;
	nh.getParam("/nics_master/cam_intrinsics_455", intrinsics_list);

	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
		{
			std::ostringstream ostr;
			ostr << intrinsics_list[3 * i+ j];
			std::istringstream istr(ostr.str());
			istr >> cameraMat(i, j);
		}
}

void nics::VdrseLib::camExtrinsic_config(ros::NodeHandle& nh)
{
	// load cam on ugv extrinsics
	XmlRpc::XmlRpcValue extrinsics_list_cam_ugv;            
	nh.getParam("/nics_master/cam_ugv_extrinsics", extrinsics_list_cam_ugv);                
	
	Eigen::Matrix4d extrinsic_temp;

	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
		{
			std::ostringstream ostr;
			ostr << extrinsics_list_cam_ugv[4 * i+ j];
			std::istringstream istr(ostr.str());
			istr >> extrinsic_temp(i, j);
		}

	extrinsic_temp.block<3,3>(0,0) = Eigen::Quaterniond(extrinsic_temp.block<3,3>(0,0)).normalized().toRotationMatrix();
	pose_cam_inUgvBody_SE3 = Sophus::SE3d(
		extrinsic_temp
	);
	Sophus::Vector6d temp;
	temp << -0.21379937,
			0.69665049,
			0.04347424,
			-0.70877894,
			-0.02166329,
			-2.8979549;

	std::cout<<Sophus::SE3d::exp(temp).matrix()<<std::endl;
}

void nics::VdrseLib::CamInGeneralBody_config(ros::NodeHandle& nh)
{
	// load cam in general body frame                
	Eigen::Matrix3d cam_to_body_rot;
	cam_to_body_rot << 
		0,0,1,
		-1,0,0,
		0,-1,0;

	pose_cam_inGeneralBodySE3 = Sophus::SE3d(
		cam_to_body_rot, 
		Eigen::Vector3d::Zero()
	);
}

void nics::VdrseLib::LEDInBodyAndOutlierSetting_config(ros::NodeHandle& nh)
{
	//load LED potisions in body frame
	XmlRpc::XmlRpcValue LED_list;
	double temp_delta;
	nh.getParam("/nics_master/LED_positions", LED_list); 
	nh.getParam("/nics_master/LED_temp", temp_delta); 

	std::vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

	std::cout<<"\nPts on body frame (X Y Z):\n";
	for(int i = 0; i < LED_list.size(); i++)
	{
		Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
		
		temp.x() -= temp_delta;

		norm_of_x_points.push_back(temp.x());
		norm_of_y_points.push_back(temp.y());
		norm_of_z_points.push_back(temp.z());                    
		std::cout<<"-----"<<std::endl;
		std::cout<<temp.x()<<" "<<temp.y()<<" "<<temp.z()<<" "<<std::endl; 
		
		pts_on_body_frame.push_back(temp);
	}   
	std::cout<<std::endl;

	LED_no = pts_on_body_frame.size();

	nh.getParam("/nics_master/LED_r_number", LED_r_no);
	nh.getParam("/nics_master/LED_g_number", LED_g_no);

	//load outlier rejection info
	nh.getParam("/nics_master/MAD_dilate", MAD_dilate);
	nh.getParam("/nics_master/MAD_max", MAD_max);

	MAD_x_threshold = (calculate_MAD(norm_of_x_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_x_points) * MAD_dilate);
	MAD_y_threshold = (calculate_MAD(norm_of_y_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_y_points) * MAD_dilate);
	MAD_z_threshold = (calculate_MAD(norm_of_z_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_z_points) * MAD_dilate);

	// std::cout<<MAD_x_threshold<<std::endl;
	// std::cout<<MAD_y_threshold<<std::endl;
	// std::cout<<MAD_z_threshold<<std::endl;

	LED_no = pts_on_body_frame.size();
}

void nics::VdrseLib::KF_config(ros::NodeHandle& nh)
{
	double Q_val;
	double R_val_p;
	double R_val_v;

	nh.getParam("/nics_master/Q_val", Q_val);
	nh.getParam("/nics_master/R_val_p", R_val_p);
	nh.getParam("/nics_master/R_val_v", R_val_v);
	nh.getParam("/nics_master/Q_alpha", QAdaptiveAlpha);
	nh.getParam("/nics_master/R_beta", RAdaptiveBeta);
	nh.getParam("/nics_master/kf_size", kf_size);
	nh.getParam("/nics_master/kfZ_size", kfZ_size);
	nh.getParam("/nics_master/velo_IIR_alpha", velo_IIR_alpha);
	nh.getParam("/nics_master/OPT_MAX_ITERATION", MAX_ITERATION);
	nh.getParam("/nics_master/CONVERGE_THRESHOLD", CONVERGE_THRESHOLD);

	nh.getParam("/nics_master/KF_ON", KF_ON);

	if(KF_ON)        
		ROS_GREEN_STREAM("KF IS ON!");                

	Q_init.resize(kf_size, kf_size);
	Q_init.setIdentity();
	Q_init = Q_init * Q_val;

	R_init.resize(kfZ_size, kfZ_size);
	R_init.setIdentity();
	R_init.block<2,2>(0,0) = R_init.block<2,2>(0,0) * R_val_p;
	R_init.block<3,3>(2,2) = R_init.block<3,3>(2,2) * R_val_v;
	
	led_twist_current.resize(6);
}

void nics::VdrseLib::subpub_config(ros::NodeHandle& nh)
{
	RosTopicConfigs configs(nh, "/nics_master");
	subimage.subscribe(nh, configs.getTopicName(COLOR_SUB_TOPIC), 1);                
	subdepth.subscribe(nh, configs.getTopicName(DEPTH_SUB_TOPIC), 1);                
	sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
	sync_->registerCallback(boost::bind(&VdrseLib::camera_callback, this, _1, _2));                                

	
	uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
	//only used for validation stage
		(configs.getTopicName(UAV_POSE_SUB_TOPIC), 1, &VdrseLib::uav_pose_callback, this);

	ugv_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
		(configs.getTopicName(UGV_POSE_SUB_TOPIC), 1, &VdrseLib::ugv_pose_callback, this);
	
	uav_setpt_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("/planner_server/traj/pose", 1, &VdrseLib::uav_setpt_callback, this);
//publish
	image_transport::ImageTransport image_transport_(nh);

	pubimage = image_transport_.advertise("/processed_image",1);
	pubimage_input = image_transport_.advertise("/input_image", 1);

	
	ledpose_pub = nh.advertise<geometry_msgs::PoseStamped>
	//only used for validation stage
					(configs.getTopicName(LED_POSE_PUB_TOPIC), 1, true);
	
	ledodom_pub = nh.advertise<nav_msgs::Odometry>
					(configs.getTopicName(LED_ODOM_PUB_TOPIC),1 , true);
	
	ugvpose_pub = nh.advertise<geometry_msgs::PoseStamped>
	//only used for validation stage
					(configs.getTopicName(UGV_POSE_PUB_TOPIC), 1, true); 
	
	
	uavpose_pub = nh.advertise<geometry_msgs::PoseStamped>
	//only used for validation stage
					(configs.getTopicName(UAV_POSE_PUB_TOPIC), 1, true);
	
	campose_pub = nh.advertise<geometry_msgs::PoseStamped>
	//only used for validation stage
					(configs.getTopicName(CAM_POSE_PUB_TOPIC), 1, true);    
	
	record_led_pub = nh.advertise<nics_rse::vdrse_log>
					("/alan_state_estimation/led/led_log", 1);
	
	record_uav_pub = nh.advertise<nics_rse::vdrse_log>
					("/alan_state_estimation/led/uav_log", 1);
}


void nics::VdrseLib::ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    pose_cam_inWorld_SE3 = pose_ugv_inWorld_SE3 * pose_cam_inUgvBody_SE3;

    geometry_msgs::PoseStamped cam_pose_msg = SE3_to_posemsg(
        pose_cam_inWorld_SE3, 
        pose->header
    );

    campose_pub.publish(cam_pose_msg);

    pose_ugv_inWorld_SE3 = posemsg_to_SE3(pose->pose); 
    
    ugv_pose_msg = *pose;
    ugv_pose_msg.header.frame_id = "world";
    ugvpose_pub.publish(ugv_pose_msg);
}

void nics::VdrseLib::uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    pose_uav_inWorld_SE3 = posemsg_to_SE3(pose->pose);
    
    uav_pose_msg = *pose;
    uav_pose_msg.header.frame_id = "world";
    uavpose_pub.publish(uav_pose_msg);
}

void nics::VdrseLib::uav_setpt_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uav_stpt_msg = *pose;
}


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



/* ================ UI utilities function below ================ */

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
