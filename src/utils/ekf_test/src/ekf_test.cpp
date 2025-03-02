#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <car_odom_server/car_status.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ekf_test/ekf.hpp>
#include <random>

Ekf::CTRV ekf_ctrv;
nav_msgs::Odometry car_odom;
geometry_msgs::Pose vision_msg;
nav_msgs::Odometry gps_msg;

ros::Publisher ekf_pub;

bool odom_sub_tri = false;
double tmp = -3.14159265;

void car_odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    car_odom = *msg;
    odom_sub_tri = true;
}

void vision_Callback(const apriltag_ros::AprilTagDetectionArray &transform)
{ 
    vision_msg.position.x = transform.detections[0].pose.pose.pose.position.x;
    vision_msg.position.y = transform.detections[0].pose.pose.pose.position.y;
    vision_msg.position.z = -transform.detections[0].pose.pose.pose.position.z;

    vision_msg.orientation.w = transform.detections[0].pose.pose.pose.orientation.w;
    vision_msg.orientation.x = -transform.detections[0].pose.pose.pose.orientation.x;
    vision_msg.orientation.y = -transform.detections[0].pose.pose.pose.orientation.y;
    vision_msg.orientation.z = transform.detections[0].pose.pose.pose.orientation.z;   
}

void car_gps_Callback(const car_odom_server::car_status::ConstPtr &msg)
{
    gps_msg.pose.pose.position.x = msg->px;
    gps_msg.pose.pose.position.y = msg->py;
    gps_msg.pose.pose.position.z = msg->pz;

    gps_msg.twist.twist.linear.x = msg->vx;
    gps_msg.twist.twist.linear.y = msg->vy;
    gps_msg.twist.twist.linear.z = msg->vz;

    Eigen::AngleAxisd roll(Eigen::AngleAxisd(msg->roll,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitch(Eigen::AngleAxisd(msg->pitch,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yaw(Eigen::AngleAxisd(msg->yaw,Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond car_orientation = roll * pitch * yaw;

    gps_msg.pose.pose.orientation.w = car_orientation.w();
    gps_msg.pose.pose.orientation.x = car_orientation.x();
    gps_msg.pose.pose.orientation.y = car_orientation.y();
    gps_msg.pose.pose.orientation.z = car_orientation.z();
}

void handler()
{
    /* 在头文件中写好函数后在此调用 */
    if(odom_sub_tri == true)
    {
        Eigen::Vector3d pos, vel;
        Eigen::Quaterniond ori;
        double theta_raw;

        std::default_random_engine e;
        std::normal_distribution<double> pos_r(0.02,0.1); // 均值，标准差
        std::normal_distribution<double> vel_r(0.1,0.06); // 均值，标准差
        std::normal_distribution<double> yaw_r(0.02,0.05); // 均值，标准差
        e.seed(ros::Time::now().toSec());

        pos << car_odom.pose.pose.position.x, car_odom.pose.pose.position.y, car_odom.pose.pose.position.z;
        vel << car_odom.twist.twist.linear.x, car_odom.twist.twist.linear.y, car_odom.twist.twist.linear.z;
        ori.w() = car_odom.pose.pose.orientation.w;
        ori.x() = car_odom.pose.pose.orientation.x;
        ori.y() = car_odom.pose.pose.orientation.y;
        ori.z() = car_odom.pose.pose.orientation.z;

        // TODO add roll, pitch
        theta_raw = atan2(2.0*(ori.x()*ori.y() + ori.w()*ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z())); // quat=[w,x,y,z]
        // if(theta < 0){
        //     theta = 2 * 3.1415926 + theta;
        // }

        // pos += pos_r(e) * Eigen::Vector3d::Ones();
        // vel += vel_r(e) * Eigen::Vector3d::Ones();
        // theta_raw += yaw_r(e);

        ekf_ctrv.update(pos, vel, theta_raw);
        Eigen::Quaterniond q = Eigen::AngleAxisd(ekf_ctrv.theta,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX());
        
        
        tmp += 0.005;
        if(tmp >= 3.14159265){
            tmp -= 2 * 3.14159265;
        }

        nav_msgs::Odometry ekf_odom;
        ekf_odom.pose.covariance[0] = pos(0);
        ekf_odom.pose.covariance[1] = pos(1);
        ekf_odom.pose.covariance[2] = pos(2);
        ekf_odom.pose.covariance[3] = theta_raw;
        ekf_odom.pose.pose.position.x = ekf_ctrv.p_x;
        ekf_odom.pose.pose.position.y = ekf_ctrv.p_y;
        ekf_odom.pose.pose.position.z = ekf_ctrv.p_z; // 此处暂时将输出值赋为未处理值
        ekf_odom.pose.pose.orientation.w = q.w();
        ekf_odom.pose.pose.orientation.x = q.x();
        ekf_odom.pose.pose.orientation.y = q.y();
        ekf_odom.pose.pose.orientation.z = q.z();
        ekf_odom.twist.covariance[0] = std::sqrt(vel(0) * vel(0) + vel(1) * vel(1));
        ekf_odom.twist.covariance[1] = vel(2);
        ekf_odom.twist.covariance[2] = ekf_ctrv.acc(0);
        ekf_odom.twist.covariance[3] = ekf_ctrv.acc(1);
        ekf_odom.twist.twist.linear.x = ekf_ctrv.v_hor * cos(ekf_ctrv.theta);
        ekf_odom.twist.twist.linear.y = ekf_ctrv.v_hor * sin(ekf_ctrv.theta);
        ekf_odom.twist.twist.linear.z = ekf_ctrv.v_ver;
        ekf_odom.twist.twist.angular.x = car_odom.twist.twist.angular.x;
        ekf_odom.twist.twist.angular.y = car_odom.twist.twist.angular.y;
        ekf_odom.twist.twist.angular.z = ekf_ctrv.delta_the;

        ekf_pub.publish(ekf_odom);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ekf_test");
    ros::NodeHandle nh("~");

    ekf_pub = nh.advertise<nav_msgs::Odometry>("/pose_ekf", 10);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/remap/car", 1, &car_odom_Callback, ros::TransportHints().tcpNoDelay()); // 小车里程计话题，local坐标系
    // ros::Subscriber vision_sub = nh.subscribe("/tag_detections", 1, &vision_Callback, ros::TransportHints().tcpNoDelay()); // 二维码话题，相机坐标系
    // ros::Subscriber gps_sub = nh.subscribe("/odom/remap/car/raw", 1, &car_gps_Callback, ros::TransportHints().tcpNoDelay()); // 小车px4话题，东北天坐标系

    double error_ah_, error_av_, error_ddtheta_;
    int max_seg_;
    Eigen::VectorXd e_measure_(6);
    nh.param("error_ah", error_ah_, 0.1);
    nh.param("error_av", error_av_, 0.1);
    nh.param("error_ddtheta", error_ddtheta_, 0.1);
    nh.param("error_mpx", e_measure_(0), 0.1);
    nh.param("error_mpy", e_measure_(1), 0.1);
    nh.param("error_mpz", e_measure_(2), 0.1);
    nh.param("error_mvh", e_measure_(3), 0.1);
    nh.param("error_mvv", e_measure_(4), 0.1);
    nh.param("error_theta", e_measure_(5), 0.1);
    nh.param("max_seg", max_seg_, 100);

    int init_flag = ekf_ctrv.init(max_seg_, 0.005, error_ah_, error_av_, error_ddtheta_, e_measure_);
    if(!init_flag) 
        ROS_INFO("\033[32m[Ekf_perching]:Ekf for CTRV model initiated!\033[32m");
    else
        ROS_INFO("\033[32m[Ekf_perching]:Ekf for CTRV model initiation failed! flag = %d\033[32m",init_flag);

    while (ros::ok())
    {
        ros::spinOnce();
        handler();
        ros::Duration(0.005).sleep();
    }

    return 0;
}