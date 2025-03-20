gnome-terminal --window -e 'bash -c "roslaunch planning perching.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch uav_utils base_sim_single_vehicle_ground_truth.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 2; rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 2; rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 2; rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch ctrl_node run_ctrl.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch simulation_utils add_bias.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch simulation_utils simulation_odom.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch estimator estimator.launch; exec bash"' \
gnome-terminal --window -e --tab -e 'bash -c "sleep 2; rosrun simulation_utils cmdvel2rviz_keyboard.py; exec bash"' \

