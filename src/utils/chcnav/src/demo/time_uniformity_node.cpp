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

static FILE *gs_fp__hc_time_record;
static FILE *gs_fp__nmea_time_record;
static FILE *gs_fp__devpvt_time_record;
static FILE *gs_fp__devimu_time_record;

static void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg);
static void nmea_sentence_callback(const chcnav::string::ConstPtr &msg);
static void devpvt_sentence_callback(const chcnav::hcinspvatzcb::ConstPtr &msg);
static void devimu_sentence_callback(const chcnav::hcrawimub::ConstPtr &msg);

static void signal_exit(int sigo)
{
    fclose(gs_fp__hc_time_record);
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

    ros::Subscriber hc_sentence_sub = private_nh.subscribe("/chcnav/hc_sentence", 1000, hc_sentence_callback);
    ros::Subscriber nmea_sentence_sub = private_nh.subscribe("/chcnav/nmea_sentence", 1000, nmea_sentence_callback);
    ros::Subscriber devpvt_sentence_sub = private_nh.subscribe("/chcnav/devpvt", 1000, devpvt_sentence_callback);
    ros::Subscriber devimu_sentence_sub = private_nh.subscribe("/chcnav/devimu", 1000, devimu_sentence_callback);

    char fp_path[1024];
    char *dir_path = dirname(argv[0]);
    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "hc_time_record");
    gs_fp__hc_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "nmea_time_record");
    gs_fp__nmea_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "devpvt_time_record");
    gs_fp__devpvt_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "devimu_time_record");
    gs_fp__devimu_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    ros::spin();

    return 0;
}

static void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg)
{
    fprintf(gs_fp__hc_time_record, "%lf\n", msg->header.stamp.toSec());
}

static void nmea_sentence_callback(const chcnav::string::ConstPtr &msg)
{
    fprintf(gs_fp__nmea_time_record, "%lf\n", msg->header.stamp.toSec());
}

static void devpvt_sentence_callback(const chcnav::hcinspvatzcb::ConstPtr &msg)
{
    fprintf(gs_fp__devpvt_time_record, "%lf\n", msg->header.stamp.toSec());
}

static void devimu_sentence_callback(const chcnav::hcrawimub::ConstPtr &msg)
{
    fprintf(gs_fp__devimu_time_record, "%lf\n", msg->header.stamp.toSec());
}