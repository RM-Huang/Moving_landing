#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "mavros_msgs/RCIn.h"
#include "std_msgs/String.h"
// #include "plumbing_control/anjiang.h"

namespace rcinRemap {

class rcRemap : public nodelet::Nodelet
{
  private:
    std::thread initThread_;

    ros::Publisher rcInPub;
    ros::Subscriber rcInSub;

    double chn_lim = 0.5;

    void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg)
    {   
        mavros_msgs::RCInPtr rc_ref(new mavros_msgs::RCIn());
        //std::cout<<2<<std::endl;
        // 获取遥控器输入通道14的数值
        rc_ref->header = msg->header;
        rc_ref->rssi = msg->rssi;
        rc_ref->channels.resize(msg->channels.size());
        rc_ref->channels = msg->channels;

        // 判断遥控器输入数值是否为2000
        for(int i=14; i<16; i++)
        {
            if ( (msg->channels[i] - 1000 / 1000) > chn_lim) 
            {
                rc_ref->channels[i] = 2000;
                // ROS_INFO("接收的按键名称为:%s",keyname1.name.c_str());
            }
        }
        rcInPub.publish(rc_ref);
    }

    void init(ros::NodeHandle& nh)
    {
        rcInPub = nh.advertise<mavros_msgs::RCIn>("/mavros/rc/in/remap",10);
        rcInSub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, rcInCallback);
    }

  public:

    void onInit(void) 
    {
        ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
        initThread_ = std::thread(std::bind(&rcRemap::init, this, nh)); //在单独的线程中运行Nodelet::init()
    }
};
// int main(int argc, char *argv[])
// {
//     setlocale(LC_ALL,"");
//     ros::init(argc, argv, "remote_control");
//     ros::NodeHandle nh;
//     // 订阅遥控器输入话题
//     // ros::Subscriber rcInSub ;
//     //rcInPub = nh.advertise<plumbing_control::anjiang>("Ctrl_pub",10);

//     // plumbing_control::anjiang controlMsg; 
//     // std::cout<<1<<std::endl;
//     rcInPub = nh.advertise<mavros_msgs::RCIn>("/anjianming",10);
//     ros::Subscriber rcInSub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, rcInCallback);

//     ros::spin();
    
//     return 0;
// }
} //namespace rcinRemap

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcinRemap::rcRemap, nodelet::Nodelet);