#include "chcnav/hc_sentence.h"
#include "chcnav/hcinspvatzcb.h"
#include "chcnav/hcrawimub.h"
#include "chcnav/string.h"
#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <libgen.h>
#include <map>
#include <signal.h>
#include <stdio.h>
#include <string>

using namespace std;

static FILE *gs_fp__nmea_time_record;
static FILE *gs_fp__devpvt_time_record;
static FILE *gs_fp__devimu_time_record;

static void nmea_sentence_callback(const chcnav::string::ConstPtr &msg);
static void devpvt_sentence_callback(const chcnav::hcinspvatzcb::ConstPtr &msg);
static void devimu_sentence_callback(const chcnav::hcrawimub::ConstPtr &msg);

static void signal_exit(int sigo)
{
    fclose(gs_fp__nmea_time_record);
    fclose(gs_fp__devpvt_time_record);
    fclose(gs_fp__devimu_time_record);

    return;
}

/**
 * @brief broadcaste static tf.
 * */
int main(int argc, char **argv)
{
    signal(SIGTERM, signal_exit); // signal to eixt
    signal(SIGINT, signal_exit);  // signal to eixt
    signal(SIGKILL, signal_exit); // signal to eixt

    ros::init(argc, argv, "time_uniformity_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber nmea_sentence_sub = private_nh.subscribe("/chcnav/nmea_sentence", 1000, nmea_sentence_callback);
    ros::Subscriber devpvt_sentence_sub = private_nh.subscribe("/chcnav/devpvt", 1000, devpvt_sentence_callback);
    ros::Subscriber devimu_sentence_sub = private_nh.subscribe("/chcnav/devimu", 1000, devimu_sentence_callback);

    char fp_path[1024];
    char *dir_path = dirname(argv[0]);
    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "nmea_sentence_record");
    gs_fp__nmea_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "devpvt_sentence_record");
    gs_fp__devpvt_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "devimu_sentence_record");
    gs_fp__devimu_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    ros::spin();

    return 0;
}

static void nmea_sentence_callback(const chcnav::string::ConstPtr &msg)
{
    fprintf(gs_fp__nmea_time_record, "%d--", msg->header.seq);
    fprintf(gs_fp__nmea_time_record, "%s\n", msg->sentence.c_str());
}

static void devpvt_sentence_callback(const chcnav::hcinspvatzcb::ConstPtr &msg)
{
    fprintf(gs_fp__devpvt_time_record, "%lf,", msg->header.stamp.toSec());
    fprintf(gs_fp__devpvt_time_record, "%lf,", msg->latitude);
    fprintf(gs_fp__devpvt_time_record, "%lf,", msg->longitude);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->altitude);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->undulation);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->roll);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->pitch);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->yaw);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity_stdev[0]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity_stdev[1]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity_stdev[2]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->euler_stdev[0]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->euler_stdev[1]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->euler_stdev[2]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_angular_velocity.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_angular_velocity.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_angular_velocity.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration_without_g.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration_without_g.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration_without_g.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->speed);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->heading);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->heading2);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->stat[1] << 4 + msg->stat[0]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->age);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->ns);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->ns2);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->leaps);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->hdop);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->warning);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->sensor_used);

    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_angular_velocity.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_angular_velocity.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_angular_velocity.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_acceleration.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_acceleration.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_acceleration.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_velocity.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_velocity.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_velocity.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->pdop);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vdop);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->tdop);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->gdop);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2gnss_vector.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2gnss_vector.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2gnss_vector.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2body_angle.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2body_angle.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2body_angle.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->gnss2body_vector.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->gnss2body_vector.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->gnss2body_vector.z);
    fprintf(gs_fp__devpvt_time_record, "%f", msg->gnss2body_angle_z);

    fprintf(gs_fp__devpvt_time_record, "\n");
}

static void devimu_sentence_callback(const chcnav::hcrawimub::ConstPtr &msg)
{
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->header.stamp.toSec());
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_velocity.x);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_velocity.y);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_velocity.z);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_acceleration.x);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_acceleration.y);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_acceleration.z);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->temp);
    fprintf(gs_fp__devimu_time_record, "%d,", msg->err_status);
    fprintf(gs_fp__devimu_time_record, "%d", msg->yaw);

    fprintf(gs_fp__devimu_time_record, "\n");
}