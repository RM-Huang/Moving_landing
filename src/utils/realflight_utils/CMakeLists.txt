cmake_minimum_required(VERSION 3.0.2)
project(realflight_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  message_generation
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  quadrotor_msgs
  nodelet
  car_odom_server
  geodesy
  target_prediction
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plumbing_control
# CATKIN_DEPENDS geometry_msgs mavros mavros_msgs message_generation roscpp std_msgs std_srvs 
CATKIN_DEPENDS geometry_msgs mavros mavros_msgs  roscpp std_msgs message_runtime std_srvs nodelet
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rcin_remap src/rcin_remap.cpp)
add_library(traj_analyse src/traj_analyse.cpp)
add_library(odom_remap src/odom_remap.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(rcin_remap ${catkin_EXPORTED_TARGETS})
 add_dependencies(traj_analyse ${catkin_EXPORTED_TARGETS})
 add_dependencies(odom_remap ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(rcin_remap
  ${catkin_LIBRARIES}
)
target_link_libraries(traj_analyse
  ${catkin_LIBRARIES}
  bezier_predict
)
target_link_libraries(odom_remap
  ${catkin_LIBRARIES}
)
