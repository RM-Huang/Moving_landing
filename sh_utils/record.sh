rosbag record --tcpnodelay \
/px4ctrl/takeoff_land \
//debugPx4ctrl \
/mavros/setpoint_raw/attitude \
/mavros/local_position/odom \
/mavros/local_position/velocity_local \
/mavros/global_position/raw/fix \
/drone0/planning/cmd \
/desire_pose_current_traj \
/mavros/imu/data \
/odom/remap \
/odom/remap/car \
/odom/remap/car/raw \
/vision_received \
/ir_pose_topic \
/analyse/posdiffer \
/analyse/yawdiffer \
/analyse/pitchdiffer \
/analyse/rolldiffer \
/analyse/desPath \
/analyse/truthPath \
/analyse/rpy_des \
/analyse/rpy_truth 
