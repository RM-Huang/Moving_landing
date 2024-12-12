gnome-terminal --window -e 'bash -c "roslaunch uav_utils base_sim_single_vehicle_ground_truth.launch x:=-4.22 y:=-0.23; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4ctrl run_ctrl.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 4; rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 4; rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 4; rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch planning perching.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch realflight_utils rc_remap_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 9; roslaunch realflight_utils odom_remap_sim.launch; exec bash"' \
gnome-terminal --window -e --tab -e 'bash -c "sleep 4; rosrun simulation_utils cmdvel2rviz_keyboard.py; exec bash"' \
