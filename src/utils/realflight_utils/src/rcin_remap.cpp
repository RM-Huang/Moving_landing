#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "mavros_msgs/RCIn.h"
#include "std_msgs/String.h"

#include <quadrotor_msgs/RcinRemap.h>
// #include "plumbing_control/anjiang.h"

namespace rcinRemap {

class rcRemap : public nodelet::Nodelet
{
  private:
    std::thread initThread_;

    ros::Publisher rcInPub;
    ros::Subscriber rcInSub;

    std::vector<uint16_t, std::allocator<uint16_t>> chn; //通道中间变量
    std::vector<uint16_t, std::allocator<uint16_t>> chn_last;

    double chn_lim = 0.5;
    bool ifchange = false;

    void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg)
    {   
        quadrotor_msgs::RcinRemapPtr rc_ref(new quadrotor_msgs::RcinRemap);

        rc_ref->header = msg->header;
        rc_ref->rssi = msg->rssi;
        chn.resize(msg->channels.size());
        rc_ref->channels.resize(chn.size());
        rc_ref->channels = msg->channels;

        // 判断遥控器14 15 16通道输入数值，若大于1500则触发：原本为1000则映射为2000，原本为2000则映射为1000。
        for(int i=13; i<16; i++)
        {
            if ( (std::abs(chn_last[i] - rc_ref->channels[i]) / 1000) > chn_lim && ( (rc_ref->channels[i] - 1000) / 1000) > chn_lim) 
            {
                if ( ( (chn[i] - 1000) / 1000) > chn_lim) //若此时chn对应通道为2000
                    chn[i] = 1000;
                
                else if ( ( (chn[i] -1000) / 1000) < chn_lim )
                    chn[i] = 2000;
                
                ifchange = true;
            }
            rc_ref->channels[i] = chn[i];
        }
        if (ifchange)
        {
            std::cout<<"FN1("<<rc_ref->channels[14]<<") ,"<<"FN2("<<rc_ref->channels[13]<<") ,"<<"FN3("<<rc_ref->channels[15]<<")"<<std::endl;
            ifchange = false;
        }
        chn_last = msg->channels;
        rcInPub.publish(rc_ref);
    }

    void init(ros::NodeHandle& nh)
    {
        chn.resize(16);
        for (int i=0; i<16; i++)
        {
            chn[i] = 1000;
        }
        chn_last = chn;
        rcInPub = nh.advertise<mavros_msgs::RCIn>("/mavros/rc/in/remap",10);
        rcInSub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &rcRemap::rcInCallback, this,
                                                     ros::TransportHints().tcpNoDelay());

        ROS_INFO("Waiting for rcIn");
    }

  public:

    void onInit(void) 
    {
        ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
        initThread_ = std::thread(std::bind(&rcRemap::init, this, nh)); //在单独的线程中运行Nodelet::init()
    }
};

} //namespace rcinRemap

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcinRemap::rcRemap, nodelet::Nodelet);