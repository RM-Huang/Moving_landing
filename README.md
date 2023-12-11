# Moving_landing
This is traj follow experiment branch from HRM for quardrotor landing on a moving platform project.

## Getting start
Compiling tests passed on ubuntu 20.04 with ros installed. 

install:
```
sudo apt-get install ros-noetic-geodesy
git clone -b traj_follow https://github.com/RM-Huang/Moving_landing
cd Moving_landing/src/utils
unzip mavlink_msg.zip
cd ../..
catkin_make
source devel/setup.bash
```

install odometry data transport module on car computer:
```
unzip car_data_trans.zip  -d /home
cd car_data_trans
catkin_make
```

## Simulation
You need to install PX4-Autopilot, gazebo and rviz with correct ros version. And run the following scripts:
```
sudo apt install ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers
```
Run the following sctipt to start simulation.
```
./sim_traj_follow.sh
./takeoff.sh
```
After vehicle stablized, run the following script to start planning:
```
./pub_triger.sh
```

## Realfight run
You have to read the _README.md_ file in the px4ctrl package before you run the script.
Execute the following commands to take off your vehical after you connecting _Autopilot_ to _flight computer_. 
```
cd Moving_landing/sh_utils
./realflight_traj_follow
./takeoff.sh
```
Than run the following sctipt to start planning.
```
./pub_triger.sh
```

Traj analyse data would be generated into Moving_landing/data