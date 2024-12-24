#ifndef __INPUT_HPP
#define __INPUT_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <quadrotor_msgs/PositionCommand.h>

namespace ctrl_node{

    class Odom_Data_t{
        public:
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Quaterniond q;
        Eigen::Vector3d w;
        nav_msgs::Odometry msg;

        int odom_source = 0; //0:pvq in world frame, 1:v in body frame

        ros::Time rcv_stamp;
        bool recv_new_msg;

        Odom_Data_t();
        void feed(nav_msgs::OdometryConstPtr pMsg);
    };

    class Imu_Data_t{
        public:
        Eigen::Quaterniond q;
        Eigen::Vector3d w;
        Eigen::Vector3d a;

        sensor_msgs::Imu msg;
        ros::Time rcv_stamp;

        Imu_Data_t();
        void feed(sensor_msgs::ImuConstPtr pMsg);
    };

    class State_Data_t{
        public:
        mavros_msgs::State current_state;
        mavros_msgs::State previous_state;

        State_Data_t();
        void feed(mavros_msgs::StateConstPtr pMsg);
    };

    class ExtendedState_Data_t{
        public:
        mavros_msgs::ExtendedState current_extended_state;

        ExtendedState_Data_t();
        void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
    };

    class Command_Data_t
    {
        public:
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Vector3d a;
        Eigen::Vector3d j;
        Eigen::Vector3d omg;
        double yaw;
        double yaw_rate;

        quadrotor_msgs::PositionCommand msg;
        ros::Time rcv_stamp;

        Command_Data_t();
        void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
    };
}
#endif