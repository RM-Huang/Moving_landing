gnome-terminal --window -e 'bash -c "roslaunch simulation_utils smartcar_display.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun simulation_utils control.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun simulation_utils cmdvel2gazebo_keyboard; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch realflight_utils traj_analyse.launch; exec bash"' \
