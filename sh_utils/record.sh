rosbag record --tcpnodelay \
/px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand \
/mavros/setpoint_raw/attitude \
/mavros/local_position/pose \
/px4ctrl/odom_re_zero \
/drone/planning/traj_follow \
/mavros/rc/in/remap \
/debugPx4ctrl \
/mavros/vision_pose/pose \
/traj_follow_start_trigger \
/desire_pose_current_traj \
/mavros/imu/data \
/mavros/imu/data_raw \
/mavros/actuator_control \
/mavros/hil/controls \
/mavros/hil/actuator_controls \
/visual/imuacc_x \
/visual/imuacc_y \
/visual/imuacc_z \
/analyse/rpy_des \
/analyse/rpy_truth
