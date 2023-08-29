rosbag record --tcpnodelay \
/px4ctrl/takeoff_land \
/mavros/setpoint_raw/attitude \
/mavros/local_position/pose \
/odom/remap \
/drone/planning/traj_follow \
/mavros/rc/in/remap \
/debugPx4ctrl \
/vrpn_client_ros/uav1/pose \
/traj_follow_start_trigger \
/desire_pose_current_traj \
/mavros/imu/data \
/mavros/actuator_control \
/visual/imuacc_x \
/visual/imuacc_y \
/visual/imuacc_z \
/analyse/rpy_des \
/analyse/rpy_truth
