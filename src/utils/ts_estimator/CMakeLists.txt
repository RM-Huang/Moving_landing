cmake_minimum_required(VERSION 3.0.2)
project(estimator)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  traj_opt
  roscpp
  nodelet
  vis_utils
  quadrotor_msgs
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp nodelet traj_opt
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
   include
  "$ENV{COPT_HOME}/include/"
  "$ENV{COPT_HOME}/include/coptcpp_inc/"
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories("$ENV{COPT_HOME}/lib")

# add_library(${PROJECT_NAME}
#   src/estimator.cpp
# )
# planning node
add_library(sim_odom
  src/estimator_odom_handler.cpp
)

target_link_libraries(sim_odom
  ${catkin_LIBRARIES}
)

add_dependencies(sim_odom
  ${catkin_EXPORTED_TARGETS}
)

add_executable(estimator 
  src/estimator.cpp
  src/estimator_test.cpp
)

target_link_libraries(estimator
  ${catkin_LIBRARIES}
  copt_cpp
)