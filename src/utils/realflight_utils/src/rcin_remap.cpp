#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "mavros_msgs/RCIn.h"
#include "std_msgs/String.h"

#include <quadrotor_msgs/RcinRemap.h>
#include <quadrotor_msgs/MotorlockTriger.h>
namespace rcinRemap {

class rcRemap : public nodelet::Nodelet
{
  private:
    std::thread initThread_;

    ros::Publisher rcInPub;
    ros::Subscriber rcInSub;
    ros::Subscriber landtriSub;

    ros::Timer sim_timer;

    std::vector<uint16_t, std::allocator<uint16_t>> chn; //通道中间变量
    std::vector<uint16_t, std::allocator<uint16_t>> chn_last;

    double chn_lim = 0.5;
    bool ifchange = false;
    bool locktriger = false;
    bool takeoff_sim = false;

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

        if(locktriger) // 若接收到motorlock信号，则切回手控模式
        {
            chn[14] = 1000;
            rc_ref->channels[14] = chn[14];
            locktriger = false;
            ifchange = true;
        }

        if (ifchange)
        {
            std::cout<<"FN1("<<rc_ref->channels[14]<<") ,"<<"FN2("<<rc_ref->channels[13]<<") ,"<<"FN3("<<rc_ref->channels[15]<<")"<<std::endl;
            ifchange = false;
        }
        chn_last = msg->channels;
        rcInPub.publish(rc_ref);
    }

    void rcinSim(const ros::TimerEvent& event)
    {   
        quadrotor_msgs::RcinRemapPtr rc_ref(new quadrotor_msgs::RcinRemap);
        rc_ref->channels.resize(chn.size());

        if(locktriger) // 若接收到motorlock信号，则切回悬停
        {
            chn[13] = 1000;
            locktriger = false;
            ifchange = true;
        }

        // if(takeoff_sim)
        // {
        //     if(chn[14] == 1000)
        //     {
        //         chn[14] = 2000;
        //     }
        //     else
        //     {
        //         chn[13] = 2000;
        //         takeoff_sim = false;
        //     }
        // }

        rc_ref->header.stamp = ros::Time::now();
        rc_ref->rssi = 100;
        rc_ref->channels = chn;

        if (ifchange)
        {
            std::cout<<"FN1("<<rc_ref->channels[14]<<") ,"<<"FN2("<<rc_ref->channels[13]<<") ,"<<"FN3("<<rc_ref->channels[15]<<")"<<std::endl;
            ifchange = false;
        }

        rcInPub.publish(rc_ref);
    }

    void lockCallback(const quadrotor_msgs::MotorlockTriger::ConstPtr& msg)
    {
        if(msg->triger)
        {
            locktriger = true;
        }
        else if(!msg->triger)
        {
            takeoff_sim = true;
        } 
    }

    void init(ros::NodeHandle& nh)
    {   
        int flag;
        nh.param("flag", flag, 0); // 0 for real, 1 for sim
        chn.resize(16);
        for (int i=0; i<16; i++)
        {
            chn[i] = 1000;
        }

        rcInPub = nh.advertise<mavros_msgs::RCIn>("/mavros/rc/in/remap",10);
        landtriSub = nh.subscribe<quadrotor_msgs::MotorlockTriger>("/locktriger", 1, &rcRemap::lockCallback, this,
                                                        ros::TransportHints().tcpNoDelay());

        if(flag == 0)
        {
            chn_last = chn;
            rcInSub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &rcRemap::rcInCallback, this,
                                                        ros::TransportHints().tcpNoDelay());

            ROS_INFO("[rcin_remap]:Waiting for rcIn");
        }
        else if(flag == 1)
        {
            ifchange = true;
            chn[13] = 2000;
            chn[14] = 2000;
            chn[0] = 1500;
            chn[1] = 1500;
            chn[2] = 1500;
            chn[3] = 1500;
            sim_timer = nh.createTimer(ros::Duration(0.05), &rcRemap::rcinSim, this);

            ROS_INFO("[rcin_remap]:rcin ready to publish");
        }
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