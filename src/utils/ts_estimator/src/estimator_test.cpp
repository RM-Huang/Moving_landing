#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <quadrotor_msgs/EstimatorOdom.h>
#include <quadrotor_msgs/EstimatorDebug.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include "estimator.hpp"

estimate::Solver solver;
nav_msgs::Odometry car_truth;
quadrotor_msgs::EstimatorOdom odom_data;

ros::Publisher debug_pub;
ros::Publisher car_pub;

int idx = 0;

bool odom_sub_tri = false;

void odom_Callback(const quadrotor_msgs::EstimatorOdom::ConstPtr& msg)
{
    odom_data = *msg;
    if(!odom_sub_tri){
        odom_sub_tri = true;
        ROS_INFO("\033[32m[estimator]:car odom received!\033[32m");
    }
}

void car_truth_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    car_truth = *msg;
}

void handler()
{
    /* 在头文件中写好函数后在此调用 */
    if(odom_sub_tri)
    {
        // idx += 1;
        std::cout << "________________entering solver______________" <<std::endl;
        nav_msgs::Odometry uav_odom = odom_data.uav_odom;
        nav_msgs::Odometry car_odom = odom_data.car_odom;
        Eigen::Vector3d p_uu(uav_odom.pose.pose.position.x, uav_odom.pose.pose.position.y, uav_odom.pose.pose.position.z);
        Eigen::Vector3d p_cc(car_odom.pose.pose.position.x, car_odom.pose.pose.position.y, car_odom.pose.pose.position.z);
        Eigen::Vector3d v_cc(car_odom.twist.twist.linear.x, car_odom.twist.twist.linear.y, car_odom.twist.twist.linear.z);
        Eigen::Quaterniond q_uu(uav_odom.pose.pose.orientation.w, uav_odom.pose.pose.orientation.x, uav_odom.pose.pose.orientation.y, uav_odom.pose.pose.orientation.z);
        Eigen::Quaterniond q_cc(car_odom.pose.pose.orientation.w, car_odom.pose.pose.orientation.x, car_odom.pose.pose.orientation.y, car_odom.pose.pose.orientation.z);

        Eigen::Vector3d b(odom_data.dir_uc.x, odom_data.dir_uc.y, odom_data.dir_uc.z);

        Eigen::VectorXd res;
        int ret = solver.optimize(p_uu, p_cc, q_uu, v_cc, b, res);
        if(ret == -1){
            ROS_ERROR("[estimator]:solving time out!");
        }else if(ret == 1){
            quadrotor_msgs::EstimatorDebug debug_msg;
            nav_msgs::Odometry re_car_msg;
            Eigen::Quaterniond q_b(res(0), res(1), res(2), res(3));
            Eigen::Vector3d pos_b(res(4), res(5), res(6));
            double t_b = res(7);

            debug_msg.pose_bias.pose.position.x = pos_b(0);
            debug_msg.pose_bias.pose.position.y = pos_b(1);
            debug_msg.pose_bias.pose.position.z = pos_b(2);
            debug_msg.pose_bias.pose.orientation.w = q_b.w();
            debug_msg.pose_bias.pose.orientation.x = q_b.x();
            debug_msg.pose_bias.pose.orientation.y = q_b.y();
            debug_msg.pose_bias.pose.orientation.z = q_b.z();
            debug_msg.time_bias = t_b;
            debug_msg.pose_bias.header.stamp = car_odom.header.stamp;
            debug_pub.publish(debug_msg);

            p_cc = q_b * (p_cc + v_cc * t_b) + pos_b;
            v_cc = q_b * v_cc;
            q_cc = q_b * q_cc;

            re_car_msg.pose.pose.position.x = p_cc(0);
            re_car_msg.pose.pose.position.y = p_cc(1);
            re_car_msg.pose.pose.position.z = p_cc(2);
            re_car_msg.pose.pose.orientation.w = q_cc.w();
            re_car_msg.pose.pose.orientation.x = q_cc.x();
            re_car_msg.pose.pose.orientation.y = q_cc.y();
            re_car_msg.pose.pose.orientation.z = q_cc.z();
            re_car_msg.twist.twist.linear.x = v_cc(0);
            re_car_msg.twist.twist.linear.y = v_cc(1);
            re_car_msg.twist.twist.linear.z = v_cc(2);
            re_car_msg.header.stamp = car_odom.header.stamp;
            car_pub.publish(re_car_msg);
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "estimator");
    ros::NodeHandle nh("~");

    debug_pub = nh.advertise<quadrotor_msgs::EstimatorDebug>("/estimator_debug", 10);
    car_pub = nh.advertise<nav_msgs::Odometry>("/car_recovery", 10);
    ros::Subscriber odom_sub = nh.subscribe("odom_topic", 10, odom_Callback);
    ros::Subscriber car_truth_sub = nh.subscribe("car_truth_topic", 10, car_truth_Callback);

    bool time_iter;
    int sample_num;
    double weight_decrese_rate;
    nh.param("time_iter", time_iter, false);
    nh.param("sample_num", sample_num, 100);
    nh.param("weight_decrese_rate", weight_decrese_rate, 0.8);

    int init_flag = solver.init(time_iter, sample_num, weight_decrese_rate);
    if(init_flag == 1) 
        ROS_INFO("\033[32m[estimator]:solver initiated!\033[32m");
    else
        ROS_INFO("\033[32m[estimator]:solver initiation failed! flag = %d\033[32m",init_flag);

    while (ros::ok())
    {
        ros::spinOnce();
        handler();
        ros::Duration(0.01).sleep();
    }

    return 0;
}