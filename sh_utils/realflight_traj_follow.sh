gnome-terminal --window -e 'bash -c "roslaunch gcopter_ml traj_follow_planning.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun rosserial_server socket_node /cmd_vel:=/turtle1/cmd_vel; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch mavros px4.launch fcu_url:="/dev/ttyACM1"; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 7; rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch realflight_utils rc_remap.launch; exec bash"' \
--tab -e 'bash -c "sleep 13; roslaunch px4ctrl run_ctrl.launch; exec bash"' \
--tab -e 'bash -c "sleep 16; roslaunch realflight_utils traj_analyse.launch; exec bash"' \