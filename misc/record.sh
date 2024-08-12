rosbag record -O rse_0811_circle.bag \
/vrpn_client_node/gh034_car/pose \
/vrpn_client_node/gh034_car/twist \
/camera/color/image_raw/compressed \
/camera/aligned_depth_to_color/image_raw \
/camera/color/camera_info \
/mavros/vision_pose/pose \
/mavros/local_position/velocity_local \
/mavros/rc/out \
/mavros/setpoint_raw/attitude \
/mavros/imu/data \
/mavros/imu/data_raw
