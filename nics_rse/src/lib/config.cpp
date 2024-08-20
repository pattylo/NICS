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
 * \file config.cpp
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
	ESO_config(nh);
	subpub_config(nh);
	other_config(nh);
}

void nics::VdrseLib::POI_config(ros::NodeHandle& nh)
{
	// load POI_extract config
	nh.getParam("/nics_master/EFFECTIVE_DISTANCE", EFFECTIVE_DISTANCE);     
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

	//load outlier rejection info
	nh.getParam("/nics_master/MAD_dilate", MAD_dilate);
	nh.getParam("/nics_master/MAD_max", MAD_max);

	MAD_x_threshold = (calculate_MAD(norm_of_x_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_x_points) * MAD_dilate);
	MAD_y_threshold = (calculate_MAD(norm_of_y_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_y_points) * MAD_dilate);
	MAD_z_threshold = (calculate_MAD(norm_of_z_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_z_points) * MAD_dilate);

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

void nics::VdrseLib::ESO_config(ros::NodeHandle& nh)
{
	nh.getParam("/nics_master/hover_thrust", hover_thrust);
	A_eso.resize(9,9);
	A_eso.setZero();
	A_eso.block<3,3>(0,3).setIdentity();
	A_eso.block<3,3>(3,6).setIdentity();

	B_eso.resize(9,3);
	B_eso.setZero();
	B_eso.block<3,3>(3,0).setIdentity();

	L_eso.resize(9,3);
	L_eso.setZero();
	L_eso.block<3,3>(0,0) = 6 * Eigen::Matrix3d::Identity();
	L_eso.block<3,3>(3,0) = 12 * Eigen::Matrix3d::Identity();
	L_eso.block<3,3>(6,0) = 24 * Eigen::Matrix3d::Identity();

	C_eso.resize(3,9);
	C_eso.setZero();
	C_eso.block<3,3>(0,0).setIdentity();

	u_input.resize(3);
	y_eso.resize(3);

	Eigen::MatrixXd F = (A_eso-L_eso*C_eso);
	// std::cout<<lala<<std::endl<<std::endl;

	// std::cout<<lala.eigenvalues()<<std::endl<<std::endl;

	for (int i = 0; i < F.eigenvalues().size(); i++)
	{
		if(F.eigenvalues()(i).real() >= 0)
			patty::Debug("POLE PLACEMENT WRONG!");
	}
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
					("/nics_rse/led_log", 1);
	
	record_uav_pub = nh.advertise<nics_rse::vdrse_log>
					("/nics_rse/uav_log", 1);
    
// spinner
    eso_spinner = nh.createTimer(
        ros::Duration(1.0/50.0), 
        &VdrseLib::eso_mainspinCallback, 
        this
    );

	u_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
					("/mavros/setpoint_raw/attitude", 1, &VdrseLib::u_callback, this);
	z_dist_pub = nh.advertise<geometry_msgs::PointStamped>
					("/nics_rse/dist", 1, true);
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

void nics::VdrseLib::u_callback(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{	
	u_input = Eigen::Quaterniond(
		msg->orientation.w,
		msg->orientation.x,
		msg->orientation.y,	
		msg->orientation.z
	).toRotationMatrix() 
		* 
	Eigen::Vector3d(
		0,
		0,
		msg->thrust / hover_thrust * g
	) - Eigen::Vector3d(0,0,g); 
	
	if(!eso_activated)
		z_I << 
			pose_uav_inWorld_SE3.translation(), 
			Eigen::Vector3d::Zero(),
			Eigen::Vector3d::Zero();
			
	eso_activated = true;
}
