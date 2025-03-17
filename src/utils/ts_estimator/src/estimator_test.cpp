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

// struct bias_data
// {
//     Eigen::Vector3d position;
//     Eigen::Vector3d euler;
//     double time;

//     bias_data operator +(const bias_data& x) const {
//         bias_data b;
//         b.position = position + x.position;
//         b.euler = euler + x.euler;
//         b.time = time + x.time;
//         return b;
//     }
// };

Eigen::Quaterniond q_b_l(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3d pos_b_l(0.0, 0.0, 0.0);
double t_b_l = 0.0;

int valid_rank;
std::vector<Eigen::VectorXd> bias_valid;

Eigen::Quaterniond q_b;
Eigen::Vector3d pos_b(0.0, 0.0, 0.0);
double t_b = 0, t_b_total = 0;

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

Eigen::VectorXd check_valid(const Eigen::VectorXd& res){
    // b_cur.position = Eigen::Vector3d pos_b_tmp(res(4), res(5), res(6));
    Eigen::Quaterniond q_b_cur(res(0), res(1), res(2), res(3));
    Eigen::Vector3d euler = q_b_cur.matrix().eulerAngles(0,1,2);
    int rank = res(8);

    Eigen::VectorXd b_cur(res(4), res(5), res(6), euler(0), euler(1), euler(2), res(7)); // pos(3), euler(3), time(1)

    if(rank <= valid_rank){
        bias_valid.emplace_back(b_cur);
        if(bias_valid.size() > 200){
            bias_valid.erase(bias_valid.begin());
        }
    }

    // averaging
    Eigen::VectorXd b_mean = std::accumulate(bias_valid.begin(), bias_valid.end(), Eigen::VectorXd::Zero(7), 
                                            [](const Eigen::VectorXd& a, const Eigen::VectorXd& b){return a + b;});
    b_mean = b_mean / bias_valid.size();

    Eigen::Quaterniond q_valid = Eigen::AngleAxisd(b_mean(5),Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(b_mean(4),Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(b_mean(3),Eigen::Vector3d::UnitX());
    
    Eigen::VectorXd b_valid(q_valid.w(), q_valid.x(), q_valid.y(), q_valid.z(),
                            b_mean(0), b_mean(1), b_mean(2), b_mean(6));

    return b_valid;
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
        quadrotor_msgs::EstimatorDebug debug_msg;
        nav_msgs::Odometry re_car_msg;

        read_odom(p_uu, p_cc, v_cc, q_uu, q_cc, b, stamp);
        std::cout << "read odom" << std::endl;

        Eigen::VectorXd res = Eigen::VectorXd::Zero(10);
        auto tic = std::chrono::steady_clock::now();
        int ret = solver.optimize(p_uu, p_cc, q_uu, v_cc, b, res);
        auto toc = std::chrono::steady_clock::now();

        std::cout << "solve comp" << std::endl;

        if(ret == -1){
            ROS_ERROR("[estimator]:solving time out!");
        }else if(ret == 1){
            // Eigen::Quaterniond q_b_tmp(res(0), res(1), res(2), res(3));
            // Eigen::Vector3d pos_b_tmp(res(4), res(5), res(6));
            // double t_b_tmp = res(7), t_b_total_tmp = res(9);
            
            // Eigen::Quaterniond q_diff = (q_b_tmp * q_b.inverse());
            // Eigen::AngleAxisd angle_axis(q_diff);
            // std::cout << "err = " << (pos_b - pos_b_l).norm() << ", " << angle_axis.angle() << ", " << std::abs(t_b - t_b_l) << std::endl;
            // if((angle_axis.angle() < 0.12))
            // {
            //     q_b_l = q_b_tmp;
            // }

            // if((pos_b - pos_b_tmp).norm() < 0.2){
            //     pos_b_l = pos_b_tmp;
            // }

            // if(std::abs(t_b - t_b_tmp) < 0.03){
            //     t_b_l = t_b_tmp;
            // }

            Eigen::VectorXd bias = check_valid(res);
            q_b_l = Eigen::Quaterniond(bias(0), bias(1), bias(2), bias(3));
            pos_b_l = Eigen::Vector3d(bias(4), bias(5), bias(6));
            t_b_l = bias(7);
        }
        read_odom(p_uu, p_cc, v_cc, q_uu, q_cc, b, stamp);

        debug_msg.pose_bias_cur.pose.position.x = res(4);
        debug_msg.pose_bias_cur.pose.position.y = res(5);
        debug_msg.pose_bias_cur.pose.position.z = res(6);
        debug_msg.pose_bias_cur.pose.orientation.w = res(0);
        debug_msg.pose_bias_cur.pose.orientation.x = res(1);
        debug_msg.pose_bias_cur.pose.orientation.y = res(2);
        debug_msg.pose_bias_cur.pose.orientation.z = res(3);
        debug_msg.time_bias_cur = res(7);
        debug_msg.pose_bias.pose.position.x = pos_b_l(0);
        debug_msg.pose_bias.pose.position.y = pos_b_l(1);
        debug_msg.pose_bias.pose.position.z = pos_b_l(2);
        debug_msg.pose_bias.pose.orientation.w = q_b_l.w();
        debug_msg.pose_bias.pose.orientation.x = q_b_l.x();
        debug_msg.pose_bias.pose.orientation.y = q_b_l.y();
        debug_msg.pose_bias.pose.orientation.z = q_b_l.z();
        debug_msg.time_bias = t_b_l;
        debug_msg.time_bias_total = t_b_total;
        debug_msg.rank = res(8);
        debug_msg.solving_t = (toc - tic).count() * 1e-6;
        debug_msg.pose_bias.header.stamp = stamp;
        debug_pub.publish(debug_msg);

        // Eigen::Vector3d p_truth = q_b.inverse() * (p_cc + v_cc * t_b) + pos_b; //debug

        // p_cc = q_b_l.inverse() * (p_cc + v_cc * t_b_l) + pos_b_l;
        // v_cc = q_b_l.inverse() * v_cc;
        // q_cc = q_b_l.inverse() * q_cc;

        // re_car_msg.pose.pose.position.x = p_cc(0);
        // re_car_msg.pose.pose.position.y = p_cc(1);
        // re_car_msg.pose.pose.position.z = p_truth(2);
        // re_car_msg.pose.pose.orientation.w = q_cc.w();
        // re_car_msg.pose.pose.orientation.x = q_cc.x();
        // re_car_msg.pose.pose.orientation.y = q_cc.y();
        // re_car_msg.pose.pose.orientation.z = q_cc.z();
        // re_car_msg.twist.twist.linear.x = v_cc(0);
        // re_car_msg.twist.twist.linear.y = v_cc(1);
        // re_car_msg.twist.twist.linear.z = v_cc(2);
        // re_car_msg.header.stamp = stamp;
        // car_pub.publish(re_car_msg);
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
    nh.param("valid_rank", valid_rank, 9);
    nh.param("weight_decrese_rate", weight_decrese_rate, 0.8);

    double trans_x, trans_y, trans_z, rotat_roll, rotat_pitch, rotat_yaw;
    nh.param("time_delay", t_b, 0.0);
    nh.param("trans_x", trans_x, 0.0);
    nh.param("trans_y", trans_y, 0.0);
    nh.param("trans_z", trans_z, 0.0);
    nh.param("rotat_roll", rotat_roll, 0.0);
    nh.param("rotat_pitch", rotat_pitch, 0.0);
    nh.param("rotat_yaw", rotat_yaw, 0.0);
    pos_b << - trans_x, - trans_y, - trans_z;
    Eigen::AngleAxisd rollAngle(rotat_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rotat_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rotat_yaw, Eigen::Vector3d::UnitZ());
    q_b = yawAngle * pitchAngle * rollAngle;

    // // debug
    // q_b_l = q_b;
    // pos_b_l = pos_b;
    // t_b_l = t_b;

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