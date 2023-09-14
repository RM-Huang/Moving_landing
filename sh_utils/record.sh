rosbag record --tcpnodelay \
/px4ctrl/takeoff_land \
/mavros/setpoint_raw/attitude \
/mavros/local_position/pose \
/odom/remap \
/drone/planning/traj_follow \
/mavros/rc/in/remap \
/debugPx4ctrl \
/traj_follow_start_trigger \
/drone/planning/traj_follow \
/desire_pose_current_traj \
/mavros/imu/data \
/mavros/actuator_control \
/mavros/local_position/odom \
/analyse/posdiffer \
/analyse/yawdiffer \
/analyse/pitchdiffer \
/analyse/rolldiffer \
/analyse/desPath \
/analyse/truthPath \
/analyse/rpy_des \
/analyse/rpy_truth \
/tag_detections \
/camera/color/image_raw
