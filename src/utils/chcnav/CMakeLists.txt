
cmake_minimum_required(VERSION 3.1.6)

project(chcnav)

message(STATUS "---------------------------")
message(STATUS "-- USE CHCNAV ROS DRIVER --")
message(STATUS "---------------------------")

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=gnu++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    sensor_msgs
    message_generation
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
add_message_files(
    FILES
    int8_array.msg
    string.msg
    hcinspvatzcb.msg
    hc_sentence.msg
    hcrawimub.msg
)

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# Service1.srv
# Service2.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/DynReconf1.cfg
# cfg/DynReconf2.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include

    # LIBRARIES chcnav_ros
    # CATKIN_DEPENDS other_catkin_pkg
    DEPENDS message_runtime
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# debug flags
option(NTRIP_SERVER_INCLUDE "option for ntrip_server" OFF)


# # Declare a C++ library

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide

# hc_msg_parser
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/hc_msg_parser)

# ------------------------------------------------------------
# serial lib

find_package(catkin REQUIRED)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

if(UNIX AND NOT APPLE)
    # If Linux, add rt and pthread
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        DEPENDS rt pthread
    )
else()
    # Otherwise normal call
    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
    )
endif()

set(SERIAL_LIB_SRCS
    ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/serial.cc
    ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/include/serial/serial.h
    ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/include/serial/v8stdint.h
)
if(APPLE)
    # If OSX
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/unix.cc)
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/unix.cc)
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/win.cc)
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/list_ports/list_ports_win.cc)
endif()

include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/include)

include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/udp_common)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/tcp_common)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/file_common)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common)

set(CHCNAV_MSG_PARSER_SOURCES
        ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/hc_msg_parser/hc_msg_parser.c
        ${SERIAL_LIB_SRCS}
)

# hc_msg_processer
include_directories(${PROJECT_SOURCE_DIR}/src/hc_cgi_protocol_process_node)
set(CHCNAV_MSG_PROCESSER_SOURCES ${PROJECT_SOURCE_DIR}/src/hc_cgi_protocol_process_node/hc_cgi_protocol.c)

# a swas server class
if (NTRIP_SERVER_INCLUDE)
    include_directories(${PROJECT_SOURCE_DIR}/src/ntrip_server_node)
    set(CHCNAV_NTRIP_SOURCES ${PROJECT_SOURCE_DIR}/src/ntrip_server_node/swas_server.cpp)
    set(CHCNAV_ROS_LIBS swas_sdk)

    message(STATUS "architecture ${CMAKE_HOST_SYSTEM_PROCESSOR}")

    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
        link_directories(${PROJECT_SOURCE_DIR}/lib/x86_64)
    elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
        link_directories(${PROJECT_SOURCE_DIR}/lib/aarch64)
    endif()

    set(CHCNAV_NODE_SOURCES
        ${PROJECT_SOURCE_DIR}/src/ntrip_server_node/ntrip_server_launch_node.cpp
        ${CHCNAV_NODE_SOURCES}
    )

endif()

# 设置依赖库
set(CHCNAV_ROS_LIBS ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBA_INIT} ${CHCNAV_ROS_LIBS} pthread dl)

# lib file 此处这样添加是为了输出日志
foreach(SINGLE_SOURCE_FILE ${CHCNAV_SERIAL_SOURCES} ${CHCNAV_NTRIP_SOURCES} ${CHCNAV_MSG_PARSER_SOURCES} ${CHCNAV_MSG_PROCESSER_SOURCES})
    message(STATUS "add ${SINGLE_SOURCE_FILE}")
    set(CHCNAV_ROS_SOURCES ${SINGLE_SOURCE_FILE} ${CHCNAV_ROS_SOURCES})
endforeach(SINGLE_SOURCE_FILE)

# node
set(CHCNAV_NODE_SOURCES
    ${PROJECT_SOURCE_DIR}/src/hc_cgi_protocol_process_node/hc_cgi_protocol_process_node.cpp
    ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/hc_msg_parser_launch_node.cpp
    ${PROJECT_SOURCE_DIR}/src/demo/chcnav_fix_demo.cpp
    ${PROJECT_SOURCE_DIR}/src/demo/time_uniformity_node.cpp
    ${PROJECT_SOURCE_DIR}/src/demo/record_msg_to_file.cpp
    ${CHCNAV_NODE_SOURCES}
)

foreach(SINGLE_NODE_FILE ${CHCNAV_NODE_SOURCES} )
    message(STATUS "node file ${SINGLE_NODE_FILE} detected")
    STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" NODE_NAME ${SINGLE_NODE_FILE})
    message(STATUS "node ${NODE_NAME}")

    # node name is made of filename and _node
    add_executable(${NODE_NAME} ${SINGLE_NODE_FILE} ${CHCNAV_ROS_SOURCES})
    add_dependencies(${NODE_NAME} ${PROJECT_NAME}_generate_messages_cpp)

    target_link_libraries(${NODE_NAME} ${CHCNAV_ROS_LIBS})
endforeach(SINGLE_NODE_FILE)

message(STATUS "---------------------------")
message(STATUS "-- END CHCNAV ROS DRIVER --")
message(STATUS "---------------------------")

# # Rename C++ executable without prefix
# # The above recommended prefix causes long target names, the following renames the
# # target back to the shorter version for ease of user use
# # e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
# scripts/my_python_script
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark executables for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
# )

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_chcnav_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
