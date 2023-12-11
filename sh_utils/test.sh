gnome-terminal --window -e 'bash -c "roslaunch gazebo_ros empty_world.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch simulation_utils spawn_car.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun simulation_utils cmdvel2gazebo_keyboard; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch realflight_utils traj_analyse.launch; exec bash"' \
