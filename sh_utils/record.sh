rosbag record --tcpnodelay \
/px4ctrl/takeoff_land \
/debugPx4ctrl \
/mavros/setpoint_raw/attitude \
/mavros/local_position/odom \
/mavros/local_position/velocity_local \
/mavros/global_position/raw/fix \
/drone0/planning/cmd \
/drone0/planning/planner_debug \
/mavros/imu/data \
/odom/remap \
/odom/remap/car \
/odom/car_recover \
/car_recovery \
/odom/remap/car/raw \
/vision_received \
/gazebo/model_states \
/estimator_debug \
/estimator/sim_odom \
/smart/odom \
/odom_kalman/car \
# /drone0/planning/traj \
# /drone0/odom_visualization/robot \
# /drone0/planning/target_odom \
# /drone0/planning/tail_vel \
# /drone0/odom_visualization_plate/polygon \
# /drone0/planning/traj_wayPts \