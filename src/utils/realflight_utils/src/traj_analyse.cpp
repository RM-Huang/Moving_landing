#include <thread>
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>
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
#include "target_prediction/bezier_predict.h"

namespace trajAnalyse {

class trajAls : public nodelet::Nodelet
{
  private:
    int als_arg;
    int bezier_flag = 1;

    std::thread initThread_;
    ros::Timer als_timer;
    ros::Time current_t;
    std::ofstream dataWrite;
    Bezierpredict tgpredict;

    quadrotor_msgs::TrajcurDesire des;
    nav_msgs::Odometry gtruth;
    nav_msgs::Path desMsg;
    nav_msgs::Path truthMsg;
    quadrotor_msgs::Px4ctrlDebug ctrldes;
    // nav_msgs::Odometry ctrltruth;
    geometry_msgs::TwistStamped carvel;
    geometry_msgs::PoseStamped carpose;
    std::vector<Eigen::Vector4d> target_detect_list;
    std::vector<Eigen::MatrixXd> bezier_polyc_list;
    std::vector<double> bezierT_list;
    std::vector<double> bezier_init_time_list;

    ros::Subscriber desSub;
    ros::Subscriber gtruthSub;
    ros::Subscriber imuSub;
    ros::Subscriber ctrlSub;

    ros::Subscriber carvelSub;
    ros::Subscriber carposeSub;

    ros::Publisher predictPub;
    ros::Publisher veldifferPub;

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
    ros::Publisher ctrlquaPub;

    std::string datafile;
    std::string desTopic;
    std::string gtruthTopic;

    std_msgs::Float64 acc_x, acc_y, acc_z; // sub from imu/data
    // geometry_msgs::Vector3 imuacc_l;

    bool dessubTri = false;
    bool gtruthsubTri = false;
    bool ctrlsubTri = false;
    // bool imusubTri = false;
    bool carvelsubTri = false;
    bool carposesubTri = false;

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

        // acc_x.data = imuMsg->linear_acceleration.x;
        // acc_y.data = imuMsg->linear_acceleration.y;
        // acc_z.data = imuMsg->linear_acceleration.z;

        // // imuvel_cul();

        // imuaccPub_x.publish(acc_x);
        // imuaccPub_y.publish(acc_y);
        // imuaccPub_z.publish(acc_z);
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
        if (ctrlsubTri && gtruthsubTri)
        {
            // std::cout<<"imu_analyse start"<<std::endl;
            geometry_msgs::QuaternionStamped des_Qua;
            tf::Quaternion des_Q2T;
            tf::Quaternion gtruth_Q2T;
            geometry_msgs::Vector3 des_RPY; //(roll,pitch,yaw)
            geometry_msgs::Vector3 gtruth_RPY; //(roll,pitch,yaw)
            double des_q_x, des_q_y, des_q_z, des_q_w;

            des_q_w = ctrldes.des_q_w;
            des_q_x = ctrldes.des_q_x;
            des_q_y = ctrldes.des_q_y;
            des_q_z = ctrldes.des_q_z;
            // tf::Quaternion des_Qua(des_q_x, des_q_y, des_q_z, des_q_w);

            des_Qua.quaternion.w = ctrldes.des_q_w;
            des_Qua.quaternion.x = ctrldes.des_q_x;
            des_Qua.quaternion.y = ctrldes.des_q_y;
            des_Qua.quaternion.z = ctrldes.des_q_z;
            des_Qua.header.stamp = gtruth.header.stamp;

            ctrlquaPub.publish(des_Qua);

            // des_Qua.normalize();

            tf::quaternionMsgToTF(des_Qua.quaternion, des_Q2T);
            // des_Q2T.normalize();
            tf::Matrix3x3(des_Q2T).getRPY(des_RPY.x, des_RPY.y, des_RPY.z);
            std::cout<<"des_RPY = "<<des_RPY.x<<" "<<des_RPY.y<<" "<<des_RPY.z<<std::endl;

            tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
            // gtruth_Q2T.normalize();
            tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_RPY.x, gtruth_RPY.y, gtruth_RPY.z);

            rpydesPub.publish(des_RPY);
            rpytruthPub.publish(gtruth_RPY);
        }
        else
        {
            ROS_ERROR("[traj_anlyse]:No odom or ctrl data, please check rostopic.");
            while(!ctrlsubTri || !gtruthsubTri)
            {
                // std::cout<<"ctrsubTri = "<<ctrlsubTri<<" gtruthsubTri = "<<gtruthsubTri<<std::endl;
                ros::Duration(0.2).sleep();
            }
        }
        // if(imusubTri)
        // {
        //     imuvel_cul();
        //     imusubTri = false;
        // }
    }

    void carvelCallback(const geometry_msgs::TwistStamped::ConstPtr &velMsg)
    {
        carvel = *velMsg;
        if(!carvelsubTri)
        {
            carvelsubTri = true;
            ROS_INFO("\033[32m[traj_analyse]: car velocity msg received!\033[32m");
        }
    }

    void carposeCallback(const geometry_msgs::PoseStamped::ConstPtr &poseMsg)
    {
        carpose = *poseMsg;
        if(!carposesubTri)
        {
            carposesubTri = true;
            ROS_INFO("\033[32m[traj_analyse]: car pose msg received!\033[32m");
        }
    }

    void bezier_analyse(const ros::TimerEvent& time_event)
    {
        if(carvelsubTri && carposesubTri)
        {
            const double predict_dur = 3.0;
            const double sample_dur = 3.0;
            double carpostime = carpose.header.stamp.toSec();
            Eigen::Vector3d pos_truth;
            pos_truth[0] = carpose.pose.position.x;
            pos_truth[1] = carpose.pose.position.y;
            pos_truth[2] = carpose.pose.position.z;
            Eigen::Vector3d vel_truth;
            vel_truth[0] = carvel.twist.linear.x;
            vel_truth[1] = carvel.twist.linear.y;
            vel_truth[2] = carvel.twist.linear.z;

            if(abs(carvel.header.stamp.toSec() - carpostime) < 0.02)
            {
                target_detect_list.push_back(Eigen::Vector4d(pos_truth[0], pos_truth[1], pos_truth[2], carpostime));
                if(target_detect_list.size() >= sample_dur * 100)
                {
                    bezier_flag = tgpredict.TrackingGeneration(8,1.5,target_detect_list);
                    if(bezier_flag != 0)
                    {
                        ROS_WARN("[planning]:platform predict error");
                        // using velocity stable assumption while bezier failed
                    }
                    else
                    {
                        Eigen::MatrixXd PolyCoeff = tgpredict.getPolyCoeff();
                        bezier_polyc_list.push_back(PolyCoeff);
                        bezierT_list.push_back(tgpredict.getPolyTime()(0));
                        bezier_init_time_list.push_back(carpostime);
                    }
                    target_detect_list.erase(target_detect_list.begin());
                }
            }
            else
            {
                ROS_WARN("data alignment failed");
            }

            if(!bezier_polyc_list.empty())
            {
                std::cout<<"bezier_polyc_list != empty"<<std::endl;
                if(bezier_init_time_list.size() > predict_dur * 100)
                {
                    bezier_polyc_list.erase(bezier_polyc_list.begin());
                    bezier_init_time_list.erase(bezier_init_time_list.begin());
                    bezierT_list.erase(bezierT_list.begin());
                }
                
                double t_from_start = carpostime - bezier_init_time_list[0];
                if(abs(t_from_start) - predict_dur < 0.02 )
                {
                    Eigen::Vector3d pos_predict = tgpredict.getTPosFromBezier(t_from_start, bezierT_list[0], bezier_polyc_list[0]);
                    Eigen::Vector3d vel_predict = tgpredict.getTVelFromBezier(t_from_start, bezierT_list[0], bezier_polyc_list[0]);

                    nav_msgs::Odometry predict_msg;
                    predict_msg.header.stamp.fromSec(carpostime);
                    predict_msg.pose.pose.position.x = pos_predict[0];
                    predict_msg.pose.pose.position.y = pos_predict[1];
                    predict_msg.pose.pose.position.z = pos_predict[2];
                    predict_msg.twist.twist.linear.x = vel_predict[0];
                    predict_msg.twist.twist.linear.y = vel_predict[1];
                    predict_msg.twist.twist.linear.z = vel_predict[2];

                    std_msgs::Float64 posdiffer;
                    posdiffer.data = (pos_truth - pos_predict).norm();

                    std_msgs::Float64 veldiffer;
                    veldiffer.data = (vel_truth - vel_predict).norm();

                    predictPub.publish(predict_msg);
                    posdifferPub.publish(posdiffer);
                    veldifferPub.publish(veldiffer);
                }
            }
        }     
    }

    void init(ros::NodeHandle& nh)
    {
        nh.getParam("dataFile", datafile);
        nh.getParam("desTopic", desTopic);
        nh.getParam("gtruthTopic", gtruthTopic);
        nh.param("analyse_arg", als_arg, 0);
        
        if (als_arg == 0)
        {
            gtruthSub = nh.subscribe(gtruthTopic, 10, &trajAls::gtruthCallback, this,
                                   ros::TransportHints().tcpNoDelay());
            desSub = nh.subscribe(desTopic, 10, &trajAls::desCallback, this,
                                   ros::TransportHints().tcpNoDelay());

            despathPub = nh.advertise<nav_msgs::Path>("/analyse/desPath", 10);
            truthpathPub = nh.advertise<nav_msgs::Path>("/analyse/truthPath", 10);
            posdifferPub = nh.advertise<std_msgs::Float64>("/analyse/posdiffer", 100);
            yawdifferPub = nh.advertise<std_msgs::Float64>("/analyse/yawdiffer", 100);
            pitchdefferPub = nh.advertise<std_msgs::Float64>("/analyse/pitchdiffer", 100);
            rolldifferPub = nh.advertise<std_msgs::Float64>("/analyse/rolldiffer", 100);

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
            ctrlquaPub = nh.advertise<geometry_msgs::QuaternionStamped>("/analyse/qua_ctrl",10);
            rpytruthPub = nh.advertise<geometry_msgs::Vector3>("/analyse/rpy_truth",10);

            gtruthSub = nh.subscribe(gtruthTopic, 10, &trajAls::gtruthCallback, this,
                                   ros::TransportHints().tcpNoDelay());
            imuSub = nh.subscribe("/mavros/imu/data", 10, &trajAls::imuCallback, this);
            ctrlSub = nh.subscribe("/debugPx4ctrl", 10, &trajAls::ctrlCallback, this);

            als_timer = nh.createTimer(ros::Duration(0.01), &trajAls::imu_analyse, this);
        }
        else if (als_arg == 2)
        {
            carvelSub = nh.subscribe("/smart/velocity", 5, &trajAls::carvelCallback, this,
                                     ros::TransportHints().tcpNoDelay());
            carposeSub = nh.subscribe("/smart/center_pose", 5, &trajAls::carposeCallback, this,
                                     ros::TransportHints().tcpNoDelay());

            posdifferPub = nh.advertise<std_msgs::Float64>("/analyse/posdiffer", 100);
            veldifferPub = nh.advertise<std_msgs::Float64>("/analyse/veldiffer", 100);
            predictPub = nh.advertise<nav_msgs::Odometry>("/analyse/bezierpredict", 10);

            als_timer = nh.createTimer(ros::Duration(0.01), &trajAls::bezier_analyse, this);
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