#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

ros::Publisher cmd_pub;
nav_msgs::Odometry cmd;
double body_vel = 0;
double yaw = 0;

void init()
{
    cmd.header.frame_id = "world";
    cmd.header.stamp = ros::Time::now();
    cmd.pose.pose.orientation.w = 1.0;
    cmd.pose.pose.orientation.x = 0.0;
    cmd.pose.pose.orientation.y = 0.0;
    cmd.pose.pose.orientation.z = 0.0;
    cmd.pose.pose.position.x = 0.0;
    cmd.pose.pose.position.y = 0.0;
    cmd.pose.pose.position.z = 0.0;
    cmd.twist.twist.linear.x = 0.0;
    cmd.twist.twist.linear.y = 0.0;
    cmd.twist.twist.linear.z = 0.0;
    cmd.twist.twist.angular.x = 0.0;
    cmd.twist.twist.angular.y = 0.0;
    cmd.twist.twist.angular.z = 0.0;
}

char getkey()
{
    
}

void handler()
{

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "smart_cmd");
    ros::NodeHandle nh("~");

    cmd_pub = nh.advertise<nav_msgs::Odometry>("/smart/odom", 10);

    init();

    while (ros::ok())
    {
        ros::spinOnce();
        handler();
        ros::Duration(0.005).sleep();
    }

    return 0;
}