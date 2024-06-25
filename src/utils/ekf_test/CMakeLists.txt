cmake_minimum_required(VERSION 3.0.2)
project(ekf_test)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  car_odom_server
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
   include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ekf_test 
  src/ekf_test.cpp
)

# add_dependencies(ekf_test car_odom_server)

target_link_libraries(ekf_test
  ${catkin_LIBRARIES}
)
