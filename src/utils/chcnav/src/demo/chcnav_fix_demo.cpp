#include "chcnav/hc_sentence.h"
#include "chcnav/hcinspvatzcb.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <map>
#include <string>

using namespace std;

#define M_PI 3.14159265358979323846

/**
 * @brief hcinspvatzcb msg callback
 *
 * @param msg
 * */
static void pvt_callback(const chcnav::hcinspvatzcb::ConstPtr &msg);

static ros::Publisher gs_imu_pub;
static ros::Publisher gs_fix_pub;

/**
 * @brief broadcaste static tf.
 * */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "chcnav_fix_demo");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    tf2_ros::StaticTransformBroadcaster stf_pub;
    tf2::Quaternion qtn;

    geometry_msgs::TransformStamped ts;

    ts.header.seq = 1;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "map";
    ts.child_frame_id = "chcnav";

    ts.transform.translation.x = 0;
    ts.transform.translation.y = 0;
    ts.transform.translation.z = 0;

    qtn.setRPY(0, 0, 0);

    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    stf_pub.sendTransform(ts);

    ts.header.frame_id = "chcnav";
    ts.child_frame_id = "c_rs232";

    stf_pub.sendTransform(ts);

    gs_imu_pub = private_nh.advertise<sensor_msgs::Imu>("imu", 1000);
    gs_fix_pub = private_nh.advertise<sensor_msgs::NavSatFix>("fix", 1000);

    ros::Subscriber pvt_source = private_nh.subscribe("/chcnav/devpvt", 1000, pvt_callback);

    ros::spin();

    return 0;
}

static void pvt_callback(const chcnav::hcinspvatzcb::ConstPtr &msg)
{
    sensor_msgs::Imu imu;
    sensor_msgs::NavSatFix fix;

    // publish NavSatFix msg
    fix.header = msg->header;
    fix.altitude = msg->altitude;
    fix.longitude = msg->longitude;
    fix.latitude = msg->latitude;
    fix.status.service = fix.status.SERVICE_COMPASS;

    if (msg->stat[1] == 4 || msg->stat[1] == 8)
        fix.status.status = fix.status.STATUS_FIX;
    else
        fix.status.status = fix.status.STATUS_NO_FIX;

    gs_fix_pub.publish(fix);

    // publish imu msg
    imu.header = msg->header;
    imu.header.stamp = ros::Time::now();

    imu.angular_velocity.x = msg->vehicle_angular_velocity.x / 180 * M_PI;
    imu.angular_velocity.y = msg->vehicle_angular_velocity.y / 180 * M_PI;
    imu.angular_velocity.z = msg->vehicle_angular_velocity.z / 180 * M_PI;

    imu.linear_acceleration.x = msg->vehicle_linear_acceleration.x;
    imu.linear_acceleration.y = msg->vehicle_linear_acceleration.y;
    imu.linear_acceleration.z = msg->vehicle_linear_acceleration.z;

    // yaw: 车体坐标系下双天线航向角，取值范围 [-180, +180]，遵循右手定则，逆时针为正。
    // heading: 为车体坐标系下速度航向角，也称航迹角。取值范围 [0, 360] ，顺时针为正。
    // heading2: 为车体坐标系下双天线航向角，取值范围 [0, 360] ，顺时针为正。
    float yaw = 0.0;
    if (msg->yaw <= 180)
        yaw = -1 * msg->heading2;
    else
        yaw = 360 - msg->heading2;

    imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(msg->roll / 180 * M_PI, -msg->pitch / 180 * M_PI, yaw / 180 * M_PI);

    gs_imu_pub.publish(imu);
}
