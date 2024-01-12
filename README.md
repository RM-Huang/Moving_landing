# Moving_landing
This is traj follow experiment branch from HRM for quardrotor landing on a moving platform project.

## Getting start
Compiling tests passed on ubuntu 20.04 with ros installed. 

install:
```
sudo apt-get install ros-noetic-geodesy ros-noetic-mavros ros-noetic-apriltag-ros
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
You need to install gazebo and rviz with correct ros version.
Run the following script to start simulation.
```
./sim_traj_follow.sh
```
You can use ***WSAD*** in the second terminator to adjust velocity and attitude of the car.
![car-control terminator]([URL](https://github.com/RM-Huang/Moving_landing/blob/tmp/pic/car-control-terminator.png) "car-control terminator")
And nodelet status would be publised on the following terminator.
![nodelet status]([URL](https://github.com/RM-Huang/Moving_landing/blob/tmp/pic/nodelet%20status.png) "nodelet status")
Then use the following script to takeoff UAV.
```
./takeoff.sh
```
After vehicle stablized, run the following script to start planning:
```
./pub_triger.sh
```
Landing differ would be published as follow.
![landing differ]([URL](https://github.com/RM-Huang/Moving_landing/blob/tmp/pic/landing%20differ.png) "landing differ")

## Realfight run
You have to read the _README.md_ file in the px4ctrl package before you run the script.
Execute the following commands to take off your vehical after you connecting _Autopilot_ to _flight computer_. 
```
cd Moving_landing/sh_utils
./realflight_traj_follow.sh
./takeoff.sh
```
Than run the following sctipt to start planning.
```
./pub_triger.sh
```

Traj analyse data would be generated into Moving_landing/data