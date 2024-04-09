cmake_minimum_required(VERSION 3.0.2)
project(realflight_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  nodelet
  car_odom_server
  geodesy
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp nodelet
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(rcin_remap src/rcin_remap.cpp)
add_library(odom_remap src/odom_remap.cpp)
add_library(traj_analyse src/traj_analyse.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(rcin_remap ${catkin_EXPORTED_TARGETS})
 add_dependencies(odom_remap ${catkin_EXPORTED_TARGETS})
 add_dependencies(traj_analyse ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(rcin_remap
  ${catkin_LIBRARIES}
)

target_link_libraries(odom_remap
  ${catkin_LIBRARIES}
)

target_link_libraries(traj_analyse
  ${catkin_LIBRARIES}
)
