#include "ros/ros.h"

#include "chcnav/hc_sentence.h"
#include "chcnav/hcinspvatzcb.h"
#include "chcnav/hcrawimub.h"
#include "hc_cgi_protocol.h"

static ros::Publisher gs_devpvt_pub;
static ros::Publisher gs_devimu_pub;

static unsigned int g_leaps = 18;

/**
 * @brief 处理华测协议的回调函数
 *
 * @param msg 接收到的数据
 * */
static void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hc_cgi_protocol_process_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber serial_suber = nh.subscribe("hc_sentence", 1000, hc_sentence_callback);

    gs_devpvt_pub = nh.advertise<chcnav::hcinspvatzcb>("devpvt", 1000);
    gs_devimu_pub = nh.advertise<chcnav::hcrawimub>("devimu", 1000);

    ros::spin();

    return 0;
}

// 处理各个协议的函数
static void msg_deal__hcrawgnsspvatb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawimuib(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawodob(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawrtcmpb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawrtcmsb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawrtcmb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawimub(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawimuvb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawgsvb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcrawnmeab(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcinspvatb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcinspvatzcb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcpinfoltsb(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcpinfonzb(const chcnav::hc_sentence::ConstPtr &msg);

/**
 * @brief 处理华测协议的回调函数
 *
 * @param msg 接收到的数据
 * */
static void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg)
{
    if (hc__cgi_check_crc32((uint8_t *)(&msg->data[0]), msg->data.size()) != 0)
    {
        fprintf(stderr, "crc32 check failed!\n");
        return;
    }

    switch (msg->msg_id)
    {
        case INSPVATZCB:
            msg_deal__hcinspvatzcb(msg);
            break;
        case RAWIMUIB:
            msg_deal__hcrawimuib(msg);
            break;
        default:
            break;
    }

    return;
}

static void msg_deal__hcinspvatzcb(const chcnav::hc_sentence::ConstPtr &msg)
{
    chcnav::hcinspvatzcb devpvt;

    // devpvt的header使用原msg的header
    devpvt.header = msg->header;

    // 如果长度不对，不解析发布
    if (msg->data.size() == 296)
    {
        // 润秒
        devpvt.leaps = *((unsigned short *)(&msg->data[162]));
        g_leaps = devpvt.leaps;

        // gps 周 周内秒
        devpvt.week = *((unsigned short *)(&msg->data[22]));
        devpvt.second = *((double *)(&msg->data[24]));
        devpvt.header.stamp = ros::Time(devpvt.week * 7.0 * 24.0 * 3600.0 + devpvt.second + 315964800.0 - g_leaps);
        
        // 经纬高
        devpvt.latitude = *((double *)(&msg->data[32]));
        devpvt.longitude = *((double *)(&msg->data[40]));
        devpvt.altitude = *((float *)(&msg->data[48]));

        devpvt.position_stdev[0] = *((float *)(&msg->data[80]));
        devpvt.position_stdev[1] = *((float *)(&msg->data[84]));
        devpvt.position_stdev[2] = *((float *)(&msg->data[88]));

        devpvt.undulation = *((float *)(&msg->data[52]));

        // 姿态角
        devpvt.roll = *((float *)(&msg->data[72]));
        devpvt.pitch = *((float *)(&msg->data[68]));
        devpvt.yaw = *((float *)(&msg->data[76]));

        devpvt.euler_stdev[0] = *((float *)(&msg->data[108])); // std_roll
        devpvt.euler_stdev[1] = *((float *)(&msg->data[104])); // std_pitch
        devpvt.euler_stdev[2] = *((float *)(&msg->data[112])); // std_yaw

        devpvt.speed = *((float *)(&msg->data[140]));
        devpvt.heading = *((float *)(&msg->data[144]));
        devpvt.heading2 = *((float *)(&msg->data[148]));

        // vehicle velocity and acceleration
        devpvt.vehicle_angular_velocity.x = *((float *)(&msg->data[116]));
        devpvt.vehicle_angular_velocity.y = *((float *)(&msg->data[120]));
        devpvt.vehicle_angular_velocity.z = *((float *)(&msg->data[124]));

        devpvt.vehicle_linear_velocity.x = *((float *)(&msg->data[208]));
        devpvt.vehicle_linear_velocity.y = *((float *)(&msg->data[212]));
        devpvt.vehicle_linear_velocity.z = *((float *)(&msg->data[216]));

        devpvt.vehicle_linear_acceleration.x = *((float *)(&msg->data[196]));
        devpvt.vehicle_linear_acceleration.y = *((float *)(&msg->data[200]));
        devpvt.vehicle_linear_acceleration.z = *((float *)(&msg->data[204]));

        devpvt.vehicle_linear_acceleration_without_g.x = *((float *)(&msg->data[128]));
        devpvt.vehicle_linear_acceleration_without_g.y = *((float *)(&msg->data[132]));
        devpvt.vehicle_linear_acceleration_without_g.z = *((float *)(&msg->data[136]));

        devpvt.enu_velocity.x = *((float *)(&msg->data[56]));
        devpvt.enu_velocity.y = *((float *)(&msg->data[60]));
        devpvt.enu_velocity.z = *((float *)(&msg->data[64]));

        devpvt.enu_velocity_stdev[0] = *((float *)(&msg->data[92]));
        devpvt.enu_velocity_stdev[1] = *((float *)(&msg->data[96]));
        devpvt.enu_velocity_stdev[2] = *((float *)(&msg->data[100]));

        // 原始imu数据
        devpvt.raw_angular_velocity.x = *((float *)(&msg->data[172]));
        devpvt.raw_angular_velocity.y = *((float *)(&msg->data[176]));
        devpvt.raw_angular_velocity.z = *((float *)(&msg->data[180]));

        devpvt.raw_acceleration.x = *((float *)(&msg->data[184]));
        devpvt.raw_acceleration.y = *((float *)(&msg->data[188]));
        devpvt.raw_acceleration.z = *((float *)(&msg->data[192]));

        // stat, warning and flags
        devpvt.stat[0] = msg->data[152] & 0x0f;
        devpvt.stat[1] = (msg->data[152] >> 4) & 0x0f;

        devpvt.age = *((float *)(&msg->data[154]));

        devpvt.ns = *((unsigned short *)(&msg->data[158]));
        devpvt.ns2 = *((unsigned short *)(&msg->data[160]));

        // dop
        devpvt.hdop = *((float *)(&msg->data[164]));
        devpvt.pdop = *((float *)(&msg->data[220]));
        devpvt.vdop = *((float *)(&msg->data[224]));
        devpvt.tdop = *((float *)(&msg->data[228]));
        devpvt.gdop = *((float *)(&msg->data[232]));

        // warning
        devpvt.warning = *((unsigned short *)(&msg->data[168]));
        devpvt.sensor_used = *((unsigned short *)(&msg->data[170]));

        // body
        devpvt.ins2gnss_vector.x = *((float *)(&msg->data[236]));
        devpvt.ins2gnss_vector.y = *((float *)(&msg->data[240]));
        devpvt.ins2gnss_vector.z = *((float *)(&msg->data[244]));

        devpvt.ins2body_angle.x = *((float *)(&msg->data[248]));
        devpvt.ins2body_angle.y = *((float *)(&msg->data[252]));
        devpvt.ins2body_angle.z = *((float *)(&msg->data[256]));

        devpvt.gnss2body_vector.x = *((float *)(&msg->data[260]));
        devpvt.gnss2body_vector.y = *((float *)(&msg->data[264]));
        devpvt.gnss2body_vector.z = *((float *)(&msg->data[268]));

        devpvt.gnss2body_angle_z = *((float *)(&msg->data[272]));

        for (int index = 0; index < 16; index++)
            devpvt.receiver[index] = *((unsigned char *)(&msg->data[276 + index]));

        gs_devpvt_pub.publish(devpvt);
    }
}

static void msg_deal__hcrawimuib(const chcnav::hc_sentence::ConstPtr &msg)
{
    chcnav::hcrawimub devimu;

    // devpvt的header使用原msg的header
    devimu.header = msg->header;

    // 如果长度不对，不解析发布
    if (msg->data.size() == 68)
    {
        // header的时间设置为gps时间
        devimu.week = *((unsigned short *)(&msg->data[22]));
        devimu.second = *((double *)(&msg->data[24]));
        devimu.header.stamp = ros::Time(devimu.week * 7.0 * 24.0 * 3600.0 + devimu.second + 315964800.0 - g_leaps);

        // xyz角速度
        devimu.angular_velocity.x = *((float *)(&msg->data[32]));
        devimu.angular_velocity.y = *((float *)(&msg->data[36]));
        devimu.angular_velocity.z = *((float *)(&msg->data[40]));

        // xyz角角速度 g
        devimu.angular_acceleration.x = *((float *)(&msg->data[44]));
        devimu.angular_acceleration.y = *((float *)(&msg->data[48]));
        devimu.angular_acceleration.z = *((float *)(&msg->data[52]));

        // 温度
        devimu.temp = *((float *)(&msg->data[56]));

        // 异常表示
        devimu.err_status = *((unsigned char *)(&msg->data[60]));

        // Z轴陀螺积分航向, 180~180 系数 0.01
        devimu.yaw = *((short *)(&msg->data[61]));

        // 预留
        devimu.receiver = *((unsigned char *)(&msg->data[63]));

        gs_devimu_pub.publish(devimu);
    }
    //short 
    if (msg->data.size() == 34)
    {
        // header的时间设置为gps时间
        devimu.week = *((unsigned short *)(&msg->data[8]));
        devimu.second = ((double)*((unsigned int *)(&msg->data[10]))) / 1000.0;
        devimu.header.stamp = ros::Time(devimu.week * 7.0 * 24.0 * 3600.0 + devimu.second + 315964800.0 - g_leaps);

        // xyz角速度
        devimu.angular_velocity.x = ((float)*((short *)(&msg->data[14]))) / 80.0;
        devimu.angular_velocity.y = ((float)*((short *)(&msg->data[16]))) / 80.0;
        devimu.angular_velocity.z = ((float)*((short *)(&msg->data[18]))) / 80.0;

        // xyz角角速度 g
        devimu.angular_acceleration.x = ((float)*((short *)(&msg->data[20]))) / 5000.0;
        devimu.angular_acceleration.y = ((float)*((short *)(&msg->data[22]))) / 5000.0;
        devimu.angular_acceleration.z = ((float)*((short *)(&msg->data[24]))) / 5000.0;

        // 温度
        devimu.temp = ((float)*((short *)(&msg->data[26]))) / 100.0;

        // 异常表示
        devimu.err_status = *((unsigned char *)(&msg->data[28]));

        // Z轴陀螺积分航向, 180~180 系数 0.01
        //devimu.yaw = *((short *)(&msg->data[61]));

        // 预留
        devimu.receiver = *((unsigned char *)(&msg->data[29]));

        gs_devimu_pub.publish(devimu);
    }
}