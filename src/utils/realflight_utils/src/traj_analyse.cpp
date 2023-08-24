#include <thread>
#include <fstream>
#include <iostream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TrajcurDesire.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

namespace trajAnalyse {

class trajAls : public nodelet::Nodelet
{
  private:
    int als_arg;

    std::thread initThread_;
    ros::Timer als_timer;
    ros::Time current_t;
    std::ofstream dataWrite;

    quadrotor_msgs::TrajcurDesire des;
    nav_msgs::Odometry gtruth;
    nav_msgs::Path desMsg;
    nav_msgs::Path truthMsg;
    quadrotor_msgs::Px4ctrlDebug ctrldes;
    // nav_msgs::Odometry ctrltruth;

    ros::Subscriber desSub;
    ros::Subscriber gtruthSub;
    ros::Subscriber imuSub;
    ros::Subscriber ctrlSub;

    ros::Publisher despathPub;
    ros::Publisher truthpathPub;
    ros::Publisher posdifferPub;
    ros::Publisher yawdifferPub;
    ros::Publisher pitchdefferPub;
    ros::Publisher rolldifferPub;

    ros::Publisher imuaccPub_x;
    ros::Publisher imuaccPub_y;
    ros::Publisher imuaccPub_z;

    ros::Publisher rpydesPub;
    ros::Publisher rpytruthPub;

    std::string datafile;
    std::string desTopic;
    std::string gtruthTopic;

    std_msgs::Float64 acc_x, acc_y, acc_z; // sub from imu/data
    // geometry_msgs::Vector3 imuacc_l;

    bool dessubTri = false;
    bool gtruthsubTri = false;
    bool ctrlsubTri = false;
    // bool imusubTri = false;

    void desCallback(const quadrotor_msgs::TrajcurDesire::ConstPtr &Msg)
    {
        dessubTri = true;
        des = *Msg;
    }

    void gtruthCallback(const nav_msgs::Odometry::ConstPtr &gtruthMsg)
    {
        gtruthsubTri = true;
        gtruth = *gtruthMsg;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        // imusubTri = true;
        // std::cout<<"imu_t_size_1 = "<<imu_t.size()<<std::endl;
        // if(imu_t.size() > (fit_size - 1) )
        // {
        //     imu_t.erase(imu_t.begin());
        //     imuvel_x.erase(imuvel_x.begin());
        //     imuvel_y.erase(imuvel_y.begin());
        //     imuvel_z.erase(imuvel_z.begin());
        //     // std::cout<<"imu_t_size_2 = "<<imu_t.size()<<std::endl;
        // }

        // if(imu_t.size() < 1)
        // {
        //     imu_t0 = imuMsg->header.stamp.toSec();
        // }

        // imu_t.emplace_back(imuMsg->header.stamp.toSec() - imu_t0);
        // std::cout<<"imu_t_size = "<<imu_t.size()<<std::endl;

        acc_x.data = imuMsg->linear_acceleration.x;
        acc_y.data = imuMsg->linear_acceleration.y;
        acc_z.data = imuMsg->linear_acceleration.z;

        // imuvel_cul();

        imuaccPub_x.publish(acc_x);
        imuaccPub_y.publish(acc_y);
        imuaccPub_z.publish(acc_z);
    }

    void ctrlCallback(const quadrotor_msgs::Px4ctrlDebug::ConstPtr &ctrlMsg)
    {
        ctrlsubTri = true;
        ctrldes = *ctrlMsg;
    }

    // void ctrltruthCallback(const nav_msgs::Odometry::ConstPtr &ctrltruthMsg)
    // {
    //     ctrltruth = *ctrltruthMsg;
    // }

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
        truthpose.pose = gtruth.pose.pose;
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
        // des_Q2T.normalize();
        tf::Matrix3x3(des_Q2T).getRPY(des_RPY.x, des_RPY.y, des_RPY.z);

        tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
        // gtruth_Q2T.normalize();
        tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_RPY.x, gtruth_RPY.y, gtruth_RPY.z);

        posdiffer.data = sqrt( (gtruth.pose.pose.position.x - des.pos.position.x)*(gtruth.pose.pose.position.x - des.pos.position.x)
                    +(gtruth.pose.pose.position.y - des.pos.position.y)*(gtruth.pose.pose.position.y - des.pos.position.y)
                    +(gtruth.pose.pose.position.z - des.pos.position.z)*(gtruth.pose.pose.position.z - des.pos.position.z) );

        rolldiffer.data = abs(gtruth_RPY.x - des_RPY.x);
        pitchdiffer.data = abs(gtruth_RPY.y - des_RPY.y);
        yawdiffer.data = abs(gtruth_RPY.z - des_RPY.z);

        posdifferPub.publish(posdiffer);
        rolldifferPub.publish(rolldiffer);
        pitchdefferPub.publish(pitchdiffer);
        yawdifferPub.publish(yawdiffer);

        fileWrite(gtruth.pose.pose.position, des.pos.position, des_RPY, gtruth_RPY);
    }

    void traj_analyse(const ros::TimerEvent& time_event)
    {
        // std::cout<<"Trigger = "<<dessubTri<<" "<<gtruthsubTri<<std::endl;
        if (dessubTri && gtruthsubTri)
        {
            current_t = ros::Time::now();
            double des_t = des.header.stamp.toSec();
            double truth_t = gtruth.header.stamp.toSec();
            std::cout<<"als_dur = "<<des_t - truth_t<<std::endl;
            if ( fabs(des_t - truth_t) < 0.01) //时间同步检测
            {
                // ROS_INFO("write file succeed!");

                despathPublish();

                truthpathPublish();

                posedifferPublish();
          }
        }      
    }

    void imu_analyse(const ros::TimerEvent& time_event)
    {
        if (dessubTri && gtruthsubTri)
        {
            // tf::Quaternion des_Qua;
            tf::Quaternion des_Q2T;
            tf::Quaternion gtruth_Q2T;
            geometry_msgs::Vector3 des_RPY; //(roll,pitch,yaw)
            geometry_msgs::Vector3 gtruth_RPY; //(roll,pitch,yaw)
            double des_q_x, des_q_y, des_q_z, des_q_w;

            des_q_w = ctrldes.des_q_w;
            des_q_x = ctrldes.des_q_x;
            des_q_y = ctrldes.des_q_y;
            des_q_z = ctrldes.des_q_z;
            tf::Quaternion des_Qua(des_q_x, des_q_y, des_q_z, des_q_w);
            // des_Qua.x() = ctrldes.des_q_x;
            // des_Qua.y() = ctrldes.des_q_y;
            // des_Qua.z() = ctrldes.des_q_z;

            des_Qua.normalize();

            // tf::quaternionMsgToTF(des_Qua, des_Q2T);
            // des_Q2T.normalize();
            tf::Matrix3x3(des_Q2T).getRPY(des_RPY.x, des_RPY.y, des_RPY.z);

            tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
            // gtruth_Q2T.normalize();
            tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_RPY.x, gtruth_RPY.y, gtruth_RPY.z);

            rpydesPub.publish(des_RPY);
            rpytruthPub.publish(gtruth_RPY);
        }
        // if(imusubTri)
        // {
        //     imuvel_cul();
        //     imusubTri = false;
        // }
    }

    void init(ros::NodeHandle& nh)
    {
        nh.getParam("dataFile", datafile);
        nh.getParam("desTopic", desTopic);
        nh.getParam("gtruthTopic", gtruthTopic);
        nh.param("analyse_arg", als_arg, 0);
        
        gtruthSub = nh.subscribe(gtruthTopic, 10, &trajAls::gtruthCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        
        if (als_arg == 0)
        {
            desSub = nh.subscribe(desTopic, 10, &trajAls::desCallback, this,
                                   ros::TransportHints().tcpNoDelay());

            despathPub = nh.advertise<nav_msgs::Path>("desPath", 10);
            truthpathPub = nh.advertise<nav_msgs::Path>("truthPath", 10);
            posdifferPub = nh.advertise<std_msgs::Float64>("visual/posdiffer", 100);
            yawdifferPub = nh.advertise<std_msgs::Float64>("visual/yawdiffer", 100);
            pitchdefferPub = nh.advertise<std_msgs::Float64>("visual/pitchdiffer", 100);
            rolldifferPub = nh.advertise<std_msgs::Float64>("visual/rolldiffer", 100);

            als_timer = nh.createTimer(ros::Duration(0.01), &trajAls::traj_analyse, this);
        }
        else if (als_arg == 1)
        {
            // nh.param("fit_size", fit_size, 100);

            imuaccPub_x = nh.advertise<std_msgs::Float64>("/visual/imuacc_x",10);
            imuaccPub_y = nh.advertise<std_msgs::Float64>("/visual/imuacc_y",10);
            imuaccPub_z = nh.advertise<std_msgs::Float64>("/visual/imuacc_z",10);

            // imuvelPub = nh.advertise<geometry_msgs::Vector3>("/mavros/imu/data/linear_velocity",10);
            // imuvelrawPub = nh.advertise<geometry_msgs::Vector3>("/mavros/imu/data/linear_velocity_raw",10);//test

            rpydesPub = nh.advertise<geometry_msgs::Vector3>("/analyse/rpy_des",10);
            rpytruthPub = nh.advertise<geometry_msgs::Vector3>("/analyse/rpy_truth",10);

            imuSub = nh.subscribe("/mavros/imu/data", 10, &trajAls::imuCallback, this);
            ctrlSub = nh.subscribe("/debugPx4ctrl", 10, &trajAls::ctrlCallback, this);

            als_timer = nh.createTimer(ros::Duration(0.01), &trajAls::imu_analyse, this);
        }
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