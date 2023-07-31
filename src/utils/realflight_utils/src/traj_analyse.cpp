#include <thread>
#include <fstream>
#include <iostream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <nav_msgs/Path.h>
#include <quadrotor_msgs/TrajcurDesire.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>

namespace trajAnalyse {

class trajAls : public nodelet::Nodelet
{
  private:
    std::thread initThread_;
    ros::Timer als_timer;
    ros::Time current_t;
    std::ofstream dataWrite;

    quadrotor_msgs::TrajcurDesire des;
    quadrotor_msgs::TrajcurDesire gtruth;
    nav_msgs::Path desMsg;
    nav_msgs::Path truthMsg;

    ros::Subscriber desSub;
    ros::Subscriber gtruthSub;

    ros::Publisher despathPub;
    ros::Publisher truthpathPub;
    ros::Publisher posdifferPub;
    ros::Publisher yawdifferPub;
    ros::Publisher pitchdefferPub;
    ros::Publisher rolldifferPub;

    std::string datafile;
    std::string desTopic;
    std::string gtruthTopic;

    bool dessubTri = false;
    bool gtruthsubTri = false;

    void desCallback(const quadrotor_msgs::TrajcurDesire::ConstPtr &desMsg)
    {
        dessubTri = true;
        des = *desMsg;
    }

    void gtruthCallback(const quadrotor_msgs::TrajcurDesire::ConstPtr &gtruthMsg)
    {
        gtruthsubTri = true;
        gtruth = *gtruthMsg;
    }

    void despathPublish()
    {
        geometry_msgs::PoseStamped despose;
        desMsg.header.stamp = current_t;
        desMsg.header.frame_id = "odom";
        despose.header.stamp = current_t;
        despose.header.frame_id = "odom";
        despose.pose = des.pos;
        desMsg.poses.push_back(despose);

        despathPub.publish(desMsg);
    }

    void truthpathPublish()
    {
        geometry_msgs::PoseStamped truthpose;
        truthMsg.header.stamp = current_t;
        truthMsg.header.frame_id = "odom";
        truthpose.header.stamp = current_t;
        truthpose.header.frame_id = "odom";
        truthpose.pose = gtruth.pos;
        truthMsg.poses.push_back(truthpose);

        truthpathPub.publish(truthMsg);
    }

    void fileWrite(const geometry_msgs::Point truthpoint, const geometry_msgs::Point despoint,
                   const geometry_msgs::Vector3 desrpy, const geometry_msgs::Vector3 truthrpy)
    {
        dataWrite.open(datafile, std::ios::out | std::ios::app);
        dataWrite<<current_t<<' '<<truthpoint.x<<' '<<truthpoint.y<<' '<<truthpoint.z<<' '<<despoint.x<<' '<<despoint.y<<' '<<despoint.z
                  <<' '<<truthrpy.x<<' '<<truthrpy.y<<' '<<truthrpy.z<<' '<<desrpy.x<<' '<<desrpy.y<<' '<<desrpy.z<<std::endl;
        dataWrite.close();
    }   

    void posedifferPublish()
    {
        std_msgs::Float64 posdiffer, rolldiffer, pitchdiffer, yawdiffer;
        tf::Quaternion des_Q2T;
        tf::Quaternion gtruth_Q2T;
        geometry_msgs::Vector3 des_RPY; //(roll,pitch,yaw)
        geometry_msgs::Vector3 gtruth_RPY; //(roll,pitch,yaw)
          
        tf::quaternionMsgToTF(des.pos.orientation, des_Q2T);
        tf::Matrix3x3(des_Q2T).getRPY(des_RPY.x, des_RPY.y, des_RPY.z);

        tf::quaternionMsgToTF(gtruth.pos.orientation, gtruth_Q2T);
        tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_RPY.x, gtruth_RPY.y, gtruth_RPY.z);

        posdiffer.data = sqrt( (gtruth.pos.position.x - des.pos.position.x)*(gtruth.pos.position.x - des.pos.position.x)
                    +(gtruth.pos.position.y - des.pos.position.y)*(gtruth.pos.position.y - des.pos.position.y)
                    +(gtruth.pos.position.z - des.pos.position.z)*(gtruth.pos.position.z - des.pos.position.z) );

        rolldiffer.data = abs(gtruth_RPY.x - des_RPY.x);
        pitchdiffer.data = abs(gtruth_RPY.y - des_RPY.y);
        yawdiffer.data = abs(gtruth_RPY.z - des_RPY.z);

        posdifferPub.publish(posdiffer);
        rolldifferPub.publish(rolldiffer);
        pitchdefferPub.publish(pitchdiffer);
        yawdifferPub.publish(yawdiffer);

        fileWrite(gtruth.pos.position, des.pos.position, des_RPY, gtruth_RPY);
    }

    void analyse(const ros::TimerEvent& time_event)
    {
        if (dessubTri && gtruthsubTri)
        {
          current_t = ros::Time::now();
          double des_t = des.header.stamp.toSec();
          double truth_t = gtruth.header.stamp.toSec();
          if ( des_t - truth_t < 0.01) //时间同步检测
          {
            despathPublish();

            truthpathPublish();

            posedifferPublish();
          }
        }      
    }

    void init(ros::NodeHandle& nh)
    {
        nh.getParam("dataFile", datafile);
        nh.getParam("desTopic", desTopic);
        nh.getParam("gtruthTopic", gtruthTopic);

        desSub = nh.subscribe(desTopic, 10, &trajAls::desCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        
        gtruthSub = nh.subscribe(gtruthTopic, 10, &trajAls::gtruthCallback, this,
                                   ros::TransportHints().tcpNoDelay());

        despathPub = nh.advertise<nav_msgs::Path>("desPath", 10);
        truthpathPub = nh.advertise<nav_msgs::Path>("truthPath", 10);
        posdifferPub = nh.advertise<std_msgs::Float64>("visual/posdiffer", 100);
        yawdifferPub = nh.advertise<std_msgs::Float64>("visual/yawdiffer", 100);
        pitchdefferPub = nh.advertise<std_msgs::Float64>("visual/pitchdiffer", 100);
        rolldifferPub = nh.advertise<std_msgs::Float64>("visual/rolldiffer", 100);

        als_timer = nh.createTimer(ros::Duration(0.01), &trajAls::analyse, this);
    }

  public:
    void onInit(void)
    {
        ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
        initThread_ = std::thread(std::bind(&trajAls::init, this, nh)); //在单独的线程中运行Nodelet::init()      
    }
};

} //namespace tarjAls
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trajAnalyse::trajAls, nodelet::Nodelet);