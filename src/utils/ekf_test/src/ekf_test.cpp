#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <car_odom_server/car_status.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ekf_test/ekf.hpp>

nav_msgs::Odometry car_odom;
geometry_msgs::Pose vision_msg;
nav_msgs::Odometry gps_msg;

ros::Publisher ekf_pub;

bool odom_sub_tri = false;

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
    if(odom_sub_tri = true)
    {
        geometry_msgs::Point ekf_pos;
    
        ekf_pos.x = car_odom.pose.pose.position.x;
        ekf_pos.y = car_odom.pose.pose.position.y;
        ekf_pos.z = car_odom.pose.pose.position.z; // 此处暂时将输出值赋为未处理值

        ekf_pub.publish(ekf_pos);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ekf_test");
    ros::NodeHandle nh("~");

    ekf_pub = nh.advertise<geometry_msgs::Point>("/pose_ekf", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom/remap/car", 1, &car_odom_Callback, ros::TransportHints().tcpNoDelay()); // 小车里程计话题，local坐标系
    ros::Subscriber vision_sub = nh.subscribe("/tag_detections", 1, &vision_Callback, ros::TransportHints().tcpNoDelay()); // 二维码话题，相机坐标系
    ros::Subscriber gps_sub = nh.subscribe("/odom/remap/car/raw", 1, &car_gps_Callback, ros::TransportHints().tcpNoDelay()); // 小车px4话题，东北天坐标系

    while (ros::ok())
    {
        ros::spinOnce();
        handler();
        ros::Duration(0.005).sleep();
    }

    return 0;
}