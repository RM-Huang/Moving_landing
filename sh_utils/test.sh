gnome-terminal --window -e 'bash -c "roslaunch estimator sim_odom_pub.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; rosrun simulation_utils cmdvel2rviz_keyboard.py; exec bash"' \

# gnome-terminal --window -e --tab -e 'bash -c "sleep 4; rosrun simulation_utils cmdvel2rviz_keyboard.py; exec bash"' \
