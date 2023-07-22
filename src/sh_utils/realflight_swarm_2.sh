
gnome-terminal --window -e 'bash -c "cd shfiles\
./swarm2_rspx4.sh; exec bash"' \
--tab -e 'bash -c "sleep 20; rosrun mavros mavcmd -n drone2/mavros long 511 105 5000 0 0 0 0 0; "' \ ##修改ID为105的message(/mavros/imu/data_raw)频率为50Hz
--tab -e 'bash -c "sleep 20; rosrun mavros mavcmd -n drone2/mavros long 511 31 5000 0 0 0 0 0; "' \ ##修改ID为31的message(/mavros/imu/data)频率为50Hz
--tab -e 'bash -c "sleep 20; rostopic echo /drone2/vins_fusion/imu_propagate; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch px4ctrl swarm2_run_ctrl.launch; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch yolov5_trt det_human_realsense_yolo.launch; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch target_ekf target_ekf.launch; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch ego_planner run_swarm2.launch; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch odom_correction uwb_correction.launch; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch drone_detect drone_detect.launch; exec bash"' \
