cmake_minimum_required(VERSION 3.0.2)
project(simulation_utils)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  quadrotor_msgs
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

add_library(odomSim src/simulation_odom.cpp)
add_library(add_bias src/car_bias_pub.cpp)

target_link_libraries(odomSim ${catkin_LIBRARIES})
target_link_libraries(add_bias ${catkin_LIBRARIES})

add_dependencies(odomSim ${catkin_EXPORTED_TARGETS})
add_dependencies(add_bias ${catkin_EXPORTED_TARGETS})