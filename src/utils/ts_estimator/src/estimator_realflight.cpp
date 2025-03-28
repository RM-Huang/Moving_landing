#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <quadrotor_msgs/EstimatorOdom.h>
#include <quadrotor_msgs/EstimatorDebug.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <random>
#include <ros/callback_queue.h>
#include <mutex>
#include "estimator.hpp"
#include "filter.hpp"

estimate::Solver solver;
quadrotor_msgs::EstimatorOdom odom_data;

ros::Publisher debug_pub;
ros::Publisher car_pub;
ros::Publisher vis_pub;

Eigen::Quaterniond q_b_l(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3d pos_b_l(0.0, 0.0, 0.0);
double t_b_l = 0.0;

int valid_rank;
int flit_win;
// std::vector<bias_data> bias_valid;
Filter::MedianFilterBias median_filter;
Filter::MedianFilterBias median_filter_vis;

Eigen::Quaterniond q_b;
Eigen::Vector3d pos_b(0.0, 0.0, 0.0);
double t_b = 0, t_b_total = 0;

std::mutex data_mutex;
std::mutex vis_mutex;

int idx = 0;

bool odom_sub_tri = false;
bool vision_sub_tri = false;
double vision_time_last;
int vis_solved = 0;

Eigen::Vector3d q2rpy(const Eigen::Quaterniond &ori){
    Eigen::Vector3d rpy;
    rpy(0) = atan2(2.0 * (ori.w() * ori.x() + ori.y() * ori.z()), 1.0 - 2.0 * (ori.x() * ori.x() + ori.y() * ori.y()));
    rpy(1) = asin(2.0 * (ori.w() * ori.y() - ori.x() * ori.z()));
    rpy(2) = atan2(2.0 * (ori.x() * ori.y() + ori.w() * ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z()));
    return rpy;
}

void odom_Callback(const quadrotor_msgs::EstimatorOdom::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(data_mutex);
    odom_data = *msg;
    if(!odom_sub_tri){
        odom_sub_tri = true;
        ROS_INFO("\033[32m[estimator]:car odom received!\033[32m");
    }
}

// void vision_Callback(const nav_msgs::Odometry::ConstPtr& msg){
//     // std::lock_guard<std::mutex> lock(vis_mutex);
//     vision_odom = *msg;
// }

void vis_tri_Callback(const std_msgs::Float64::ConstPtr& msg){
    if(msg->data){
        vision_sub_tri = true;
        vision_time_last = ros::Time::now().toSec();
    }
}

void read_odom(Eigen::Vector3d& p_uu, Eigen::Vector3d& p_uv, Eigen::Vector3d& p_cc, Eigen::Vector3d& v_cc, Eigen::Quaterniond& q_uu, 
                Eigen::Quaterniond& q_cc, Eigen::Vector3d& b, ros::Time& stamp){
    std::lock_guard<std::mutex> lock(data_mutex);
    nav_msgs::Odometry uav_odom = odom_data.uav_odom;
    nav_msgs::Odometry car_odom = odom_data.car_odom;
    nav_msgs::Odometry vis_odom = odom_data.vis_odom;
    b << odom_data.dir_uc.x, odom_data.dir_uc.y, odom_data.dir_uc.z;
    p_uu << uav_odom.pose.pose.position.x, uav_odom.pose.pose.position.y, uav_odom.pose.pose.position.z;
    p_uv << vis_odom.pose.pose.position.x, vis_odom.pose.pose.position.y, vis_odom.pose.pose.position.z;
    p_cc << car_odom.pose.pose.position.x, car_odom.pose.pose.position.y, car_odom.pose.pose.position.z;
    v_cc << car_odom.twist.twist.linear.x, car_odom.twist.twist.linear.y, car_odom.twist.twist.linear.z;
    q_uu.coeffs() << uav_odom.pose.pose.orientation.x, uav_odom.pose.pose.orientation.y, uav_odom.pose.pose.orientation.z, uav_odom.pose.pose.orientation.w;
    q_cc.coeffs() << car_odom.pose.pose.orientation.x, car_odom.pose.pose.orientation.y, car_odom.pose.pose.orientation.z, car_odom.pose.pose.orientation.w;
    stamp = car_odom.header.stamp;
}

// void read_vis_odom(Eigen::Vector3d& p_uv){
//     // std::lock_guard<std::mutex> lock(vis_mutex);
//     p_uv << vision_odom.pose.pose.position.x, vision_odom.pose.pose.position.y, vision_odom.pose.pose.position.z;
// }

int check_valid(const Eigen::VectorXd& res, Eigen::VectorXd& b_valid){
    Filter::bias_data b_cur;
    b_cur.position = Eigen::Vector3d(res(4), res(5), res(6));
    Eigen::Quaterniond q_b_cur;
    q_b_cur.w() = res(0);
    q_b_cur.x() = res(1);
    q_b_cur.y() = res(2);
    q_b_cur.z() = res(3);
    b_cur.euler = q2rpy(q_b_cur);
    b_cur.time = res(7);
    int rank = res(8);

    if(rank <= valid_rank){
        // median filter
        Filter::bias_data b_filt = median_filter.update(b_cur);

        // Eigen::Quaterniond q_valid = Eigen::AngleAxisd(b_filt.euler(2),Eigen::Vector3d::UnitZ())
        //                             * Eigen::AngleAxisd(b_filt.euler(1),Eigen::Vector3d::UnitY())
        //                             * Eigen::AngleAxisd(b_filt.euler(0),Eigen::Vector3d::UnitX());
        Eigen::Quaterniond q_valid = Eigen::AngleAxisd(b_filt.euler(2),Eigen::Vector3d::UnitZ())
                                    * Eigen::AngleAxisd(0.0 ,Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(0.0 ,Eigen::Vector3d::UnitX()); // debug

        b_valid << q_valid.w(), q_valid.x(), q_valid.y(), q_valid.z(), b_filt.position(0), b_filt.position(1), b_filt.position(2), b_filt.time;

        return 1;
    }
    return 0;
}

int check_valid_vis(const Eigen::VectorXd& res, Eigen::VectorXd& b_valid){
    Filter::bias_data b_cur;
    b_cur.position = Eigen::Vector3d(res(4), res(5), res(6));
    Eigen::Quaterniond q_b_cur;
    q_b_cur.w() = res(0);
    q_b_cur.x() = res(1);
    q_b_cur.y() = res(2);
    q_b_cur.z() = res(3);
    b_cur.euler = q2rpy(q_b_cur);
    b_cur.time = res(7);
    int rank = res(8);

    if(rank <= valid_rank){
        // median filter
        Filter::bias_data b_filt = median_filter_vis.update(b_cur);

        Eigen::Quaterniond q_valid = Eigen::AngleAxisd(b_filt.euler(2),Eigen::Vector3d::UnitZ())
                                    * Eigen::AngleAxisd(b_filt.euler(1),Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(b_filt.euler(0),Eigen::Vector3d::UnitX());
        // Eigen::Quaterniond q_valid = Eigen::AngleAxisd(b_filt.euler(2),Eigen::Vector3d::UnitZ())
        //                             * Eigen::AngleAxisd(0.0 ,Eigen::Vector3d::UnitY())
        //                             * Eigen::AngleAxisd(0.0 ,Eigen::Vector3d::UnitX()); // debug

        b_valid << q_valid.w(), q_valid.x(), q_valid.y(), q_valid.z(), b_filt.position(0), b_filt.position(1), b_filt.position(2), b_filt.time;

        return 1;
    }
    return 0;
}

void handler()
{
    if(odom_sub_tri)
    {
        // idx += 1;
        Eigen::Vector3d p_uu, p_uv, p_cc, v_cc, b;
        Eigen::Quaterniond q_uu, q_cc;
        ros::Time stamp;
        quadrotor_msgs::EstimatorDebug debug_msg;
        nav_msgs::Odometry re_car_msg;
        Eigen::VectorXd res = Eigen::VectorXd::Zero(10);

        read_odom(p_uu, p_uv, p_cc, v_cc, q_uu, q_cc, b, stamp);
        int ret = -1;
        auto tic = std::chrono::steady_clock::now();
        if(!vision_sub_tri){
            std::cout << "________________entering bearing solver______________" <<std::endl;
            ret = solver.optimize(p_uu, p_cc, q_uu, v_cc, b, res);

            if(ret == -1){
                ROS_ERROR("[estimator]:Solving time out!");
            }else if(ret == 1){
                ROS_INFO("\033[32m[estimator]:Solving succeed!\033[32m");
                Eigen::VectorXd bias = Eigen::VectorXd::Zero(8);
                if(check_valid(res, bias)){
                    q_b_l.w() = bias(0);
                    q_b_l.x() = bias(1);
                    q_b_l.y() = bias(2);
                    q_b_l.z() = bias(3);
                    pos_b_l = Eigen::Vector3d(bias(4), bias(5), bias(6));
                    t_b_l = bias(7);
                }  
            } 
        }else{
            // TODO:需要增加逻辑防止首次飞离视觉域后偏差不再更新，当前逻辑中未进入降落流程时车辆不能出现在视觉域内
            if(ros::Time::now().toSec() - vision_time_last > 0.01){ // 如果视觉信息中断则返回
                return;
            }
            std::cout << "________________entering vision solver______________" <<std::endl;
            ret = solver.optimize_vision(p_uv, p_cc, v_cc, res);

            if(ret == -1){
                ROS_ERROR("[estimator]:Solving time out!");
            }else if(ret == 1){
                ROS_INFO("\033[32m[estimator]:Solving succeed!\033[32m");
                Eigen::VectorXd bias = Eigen::VectorXd::Zero(8);
                if(check_valid_vis(res, bias)){
                    q_b_l.w() = bias(0);
                    q_b_l.x() = bias(1);
                    q_b_l.y() = bias(2);
                    q_b_l.z() = bias(3);
                    pos_b_l = Eigen::Vector3d(bias(4), bias(5), bias(6));
                    t_b_l = bias(7);
                    vis_solved = 1;
                }  
            } 
        }
        auto toc = std::chrono::steady_clock::now();

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
        debug_msg.rank = res(8);
        debug_msg.vis_solved = vis_solved;
        debug_msg.solving_t = (toc - tic).count() * 1e-6;
        debug_msg.pose_bias.header.stamp = stamp;
        debug_pub.publish(debug_msg);

        read_odom(p_uu, p_uv, p_cc, v_cc, q_uu, q_cc, b, stamp);
        Eigen::Vector3d p_rec, v_rec;
        Eigen::Quaterniond q_rec;
        p_rec = q_b_l * (p_cc + v_cc * t_b_l) + pos_b_l;
        v_rec = q_b_l * v_cc;
        q_rec = q_b_l * q_cc;
        re_car_msg.pose.pose.position.x = p_rec(0);
        re_car_msg.pose.pose.position.y = p_rec(1);
        re_car_msg.pose.pose.position.z = p_rec(2);
        re_car_msg.pose.pose.orientation.w = q_rec.w();
        re_car_msg.pose.pose.orientation.x = q_rec.x();
        re_car_msg.pose.pose.orientation.y = q_rec.y();
        re_car_msg.pose.pose.orientation.z = q_rec.z();
        re_car_msg.twist.twist.linear.x = v_rec(0);
        re_car_msg.twist.twist.linear.y = v_rec(1);
        re_car_msg.twist.twist.linear.z = v_rec(2);
        re_car_msg.header.stamp = stamp;
        car_pub.publish(re_car_msg);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "estimator_realflight");
    ros::NodeHandle nh("~");

    debug_pub = nh.advertise<quadrotor_msgs::EstimatorDebug>("/estimator_debug", 10);
    car_pub = nh.advertise<nav_msgs::Odometry>("/car_recovery", 1);

    ros::Subscriber odom_sub = nh.subscribe("odom_topic", 1, odom_Callback);
    // ros::Subscriber vision_sub = nh.subscribe("vision_topic", 1, vision_Callback);
    ros::Subscriber vision_tri_sub = nh.subscribe("/vision_received", 1, vis_tri_Callback);

    bool time_iter;
    int sample_num;
    double weight;
    nh.param("time_iter", time_iter, false);
    nh.param("sample_num", sample_num, 100);
    nh.param("valid_rank", valid_rank, 9);
    nh.param("fliter_window", flit_win, 21);
    nh.param("weight", weight, 0.8);

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

    median_filter.init(flit_win);
    median_filter_vis.init(21);

    // // debug
    // q_b_l = q_b;
    // pos_b_l = pos_b;
    // t_b_l = t_b;
    // q_b_v = q_b;
    // pos_b_v = pos_b;
    // t_b_v = t_b;

    int init_flag = solver.init(time_iter, sample_num, weight);
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