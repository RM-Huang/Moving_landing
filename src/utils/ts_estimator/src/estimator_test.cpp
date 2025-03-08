#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <quadrotor_msgs/EstimatorOdom.h>
#include <quadrotor_msgs/EstimatorDebug.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <ros/callback_queue.h>
#include <mutex>
#include "estimator.hpp"

estimate::Solver solver;
nav_msgs::Odometry car_truth;
quadrotor_msgs::EstimatorOdom odom_data;

ros::Publisher debug_pub;
ros::Publisher car_pub;

Eigen::Quaterniond q_b_l;
Eigen::Vector3d pos_b_l;
double t_b_l;

std::mutex data_mutex;

int idx = 0;

bool odom_sub_tri = false;

void odom_Callback(const quadrotor_msgs::EstimatorOdom::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
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

void read_odom(Eigen::Vector3d& p_uu, Eigen::Vector3d& p_cc, Eigen::Vector3d& v_cc, Eigen::Quaterniond& q_uu, 
                Eigen::Quaterniond& q_cc, Eigen::Vector3d& b, ros::Time& stamp){
    std::lock_guard<std::mutex> lock(data_mutex);
    nav_msgs::Odometry uav_odom = odom_data.uav_odom;
    nav_msgs::Odometry car_odom = odom_data.car_odom;
    b << odom_data.dir_uc.x, odom_data.dir_uc.y, odom_data.dir_uc.z;
    p_uu << uav_odom.pose.pose.position.x, uav_odom.pose.pose.position.y, uav_odom.pose.pose.position.z;
    p_cc << car_odom.pose.pose.position.x, car_odom.pose.pose.position.y, car_odom.pose.pose.position.z;
    v_cc << car_odom.twist.twist.linear.x, car_odom.twist.twist.linear.y, car_odom.twist.twist.linear.z;
    q_uu.coeffs() << uav_odom.pose.pose.orientation.x, uav_odom.pose.pose.orientation.y, uav_odom.pose.pose.orientation.z, uav_odom.pose.pose.orientation.w;
    q_cc.coeffs() << car_odom.pose.pose.orientation.x, car_odom.pose.pose.orientation.y, car_odom.pose.pose.orientation.z, car_odom.pose.pose.orientation.w;
    stamp = car_odom.header.stamp;
}

void handler()
{
    /* 在头文件中写好函数后在此调用 */
    if(odom_sub_tri)
    {
        // idx += 1;
        std::cout << "________________entering solver______________" <<std::endl;
        Eigen::Vector3d p_uu, p_cc, v_cc, b;
        Eigen::Quaterniond q_uu, q_cc;
        ros::Time stamp;
        read_odom(p_uu, p_cc, v_cc, q_uu, q_cc, b, stamp);

        Eigen::VectorXd res;

        auto tic = std::chrono::steady_clock::now();
        int ret = solver.optimize(p_uu, p_cc, q_uu, v_cc, b, res);
        auto toc = std::chrono::steady_clock::now();

        if(ret == -1){
            ROS_ERROR("[estimator]:solving time out!");
        }else if(ret == 1){
            quadrotor_msgs::EstimatorDebug debug_msg;
            nav_msgs::Odometry re_car_msg;
            Eigen::Quaterniond q_b(res(0), res(1), res(2), res(3));
            Eigen::Vector3d pos_b(res(4), res(5), res(6));
            double t_b = res(7), t_b_total = res(9);
            
            Eigen::Quaterniond q_diff = (q_b * q_b_l.inverse());
            Eigen::AngleAxisd angle_axis(q_diff);
            // std::cout << "err = " << (pos_b + pos_b_l).norm() << ", " << angle_axis.angle() << ", " << std::abs(t_b - t_b_l) << std::endl;
            // if(((pos_b + pos_b_l).norm() < 0.2) && (angle_axis.angle() < 0.06) && (std::abs(t_b - t_b_l) < 0.08))
            // {
                read_odom(p_uu, p_cc, v_cc, q_uu, q_cc, b, stamp);

                debug_msg.pose_bias.pose.position.x = pos_b(0);
                debug_msg.pose_bias.pose.position.y = pos_b(1);
                debug_msg.pose_bias.pose.position.z = pos_b(2);
                debug_msg.pose_bias.pose.orientation.w = q_b.w();
                debug_msg.pose_bias.pose.orientation.x = q_b.x();
                debug_msg.pose_bias.pose.orientation.y = q_b.y();
                debug_msg.pose_bias.pose.orientation.z = q_b.z();
                debug_msg.time_bias = t_b;
                debug_msg.time_bias_total = t_b_total;
                debug_msg.rank = res(8);
                debug_msg.solving_t = (toc - tic).count() * 1e-6;
                debug_msg.pose_bias.header.stamp = stamp;
                debug_pub.publish(debug_msg);

                Eigen::Vector3d p_truth = p_cc; //debug

                p_cc = q_b.inverse() * (p_cc + v_cc * t_b) + pos_b;
                v_cc = q_b.inverse() * v_cc;
                q_cc = q_b.inverse() * q_cc;

                re_car_msg.pose.pose.position.x = p_cc(0);
                re_car_msg.pose.pose.position.y = p_cc(1);
                re_car_msg.pose.pose.position.z = p_truth(2);
                re_car_msg.pose.pose.orientation.w = q_cc.w();
                re_car_msg.pose.pose.orientation.x = q_cc.x();
                re_car_msg.pose.pose.orientation.y = q_cc.y();
                re_car_msg.pose.pose.orientation.z = q_cc.z();
                re_car_msg.twist.twist.linear.x = v_cc(0);
                re_car_msg.twist.twist.linear.y = v_cc(1);
                re_car_msg.twist.twist.linear.z = v_cc(2);
                re_car_msg.header.stamp = stamp;
                car_pub.publish(re_car_msg);
            // }
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "estimator");
    ros::NodeHandle nh("~");

    debug_pub = nh.advertise<quadrotor_msgs::EstimatorDebug>("/estimator_debug", 10);
    car_pub = nh.advertise<nav_msgs::Odometry>("/car_recovery", 10);
    ros::Subscriber odom_sub = nh.subscribe("odom_topic", 1, odom_Callback);
    ros::Subscriber car_truth_sub = nh.subscribe("car_truth_topic", 1, car_truth_Callback);

    bool time_iter;
    int sample_num;
    double weight_decrese_rate;
    nh.param("time_iter", time_iter, false);
    nh.param("sample_num", sample_num, 100);
    nh.param("weight_decrese_rate", weight_decrese_rate, 0.8);

    double trans_x, trans_y, trans_z, rotat_roll, rotat_pitch, rotat_yaw;
    nh.param("time_delay", t_b_l, 0.0);
    nh.param("trans_x", trans_x, 0.0);
    nh.param("trans_y", trans_y, 0.0);
    nh.param("trans_z", trans_z, 0.0);
    nh.param("rotat_roll", rotat_roll, 0.0);
    nh.param("rotat_pitch", rotat_pitch, 0.0);
    nh.param("rotat_yaw", rotat_yaw, 0.0);
    pos_b_l << trans_x, trans_y, trans_z;
    Eigen::AngleAxisd rollAngle(rotat_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rotat_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rotat_yaw, Eigen::Vector3d::UnitZ());
    q_b_l = yawAngle * pitchAngle * rollAngle;

    int init_flag = solver.init(time_iter, sample_num, weight_decrese_rate);
    if(init_flag == 1) 
        ROS_INFO("\033[32m[estimator]:solver initiated!\033[32m");
    else
        ROS_INFO("\033[32m[estimator]:solver initiation failed! flag = %d\033[32m",init_flag);

    ros::AsyncSpinner spinner(1); // 开启1个额外线程处理回调
    spinner.start();
    while (ros::ok())
    {
        handler();
        ros::Duration(0.01).sleep();
    }
    spinner.stop();

    return 0;
}