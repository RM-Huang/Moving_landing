# Moving_landing
This is traj follow experiment branch from HRM for quardrotor landing on a moving platform project.

## Getting start
Compiling tests passed on ubuntu 20.04 with ros installed. 

install:
```
git clone -b traj_follow https://github.com/RM-Huang/Moving_landing
cd Moving_landing
catkin_make
source devel/setup.bash
```

## Realfight run
You have to read the _README.md_ file in the px4ctrl package before you run the scripe.
Execute the following commands after you connecting _Autopilot_ to _flight computer_. 
```
cd Moving_landing/sh_utils
./realflight_traj_follow
./takeoff.sh
```

Traj analyse data would be generated into Moving_landing/data