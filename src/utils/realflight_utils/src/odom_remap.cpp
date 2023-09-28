#include <thread>
#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>
#include "polyfit.hpp"

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>

namespace odomRemap{

class odomRemap : public nodelet::Nodelet
{
private:
    polyfit::Fit fit;

    std::thread initThread_;
    ros::Timer odom_timer;

    nav_msgs::Odometry gtruth;
    geometry_msgs::Vector3 gtruth_pos_bias;
    geometry_msgs::Vector3 gtruth_rpy_bias;
    geometry_msgs::Vector3 gtruth_rpy;
    Eigen::Quaterniond gtruth_qua_bias;
    tf::Quaternion gtruth_Q2T;
          
    sensor_msgs::Imu imu;

    ros::Subscriber gtruthSub;
    ros::Subscriber imuSub;
    // ros::Subscriber debugSub;

    std::string gtruthTopic;

    ros::Publisher odomPub;
    // ros::Publisher imuvelrawPub; // publish imu_vel msg with merely acc integral

    std::vector<double> imuvel_x; // imuvel store the unfit data
    std::vector<double> imuvel_y;
    std::vector<double> imuvel_z;
    std::vector<double> imu_t;

    double imu_t0; // coefficient for imu_t return to zero
    double gtruth_time_delay = 0;
    double gtruth_time_l;
    bool imusubTri = false;
    bool gtruthsubTri = false;
    int seq = 0;
    int bias_c = 0;
    int fit_size;

    void mctruthCallback(const geometry_msgs::PoseStamped::ConstPtr &gtruthMsg)
    {
        gtruth.header = gtruthMsg->header;
        gtruth.pose.pose = gtruthMsg->pose;
        gtruthsubTri = true;
    }

    void gpstruthCallback(const nav_msgs::Odometry::ConstPtr &gtruthMsg)
    {
        gtruth = *gtruthMsg;
        gtruthsubTri = true;
    }

    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &poseMsg)
    {
        gtruth.header = poseMsg->header;
        gtruth.pose.pose = poseMsg->pose;
        gtruthsubTri = true;
    }

    void gtruth_const_bias_cal(const double &gtruth_t)
    {
        double current_t = imu.header.stamp.toSec();

        double flash_dur;
        if(bias_c > 0)
        {
            flash_dur = abs( abs(gtruth_t - current_t) - gtruth_time_delay / bias_c );
        }
        else
        {
            flash_dur = abs( abs(gtruth_t - current_t) - gtruth_time_delay);
            gtruth_time_delay = 0; // delete first gtruth_time_delay for total time bias cal
        }

        if( flash_dur < 0.5)
        {
            bias_c += 1;

            gtruth_time_delay = gtruth_time_delay + abs(gtruth_t - current_t);

            gtruth_pos_bias.x = gtruth_pos_bias.x + gtruth.pose.pose.position.x;
            gtruth_pos_bias.y = gtruth_pos_bias.y + gtruth.pose.pose.position.y;
            gtruth_pos_bias.z = gtruth_pos_bias.z + gtruth.pose.pose.position.z;

            tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
            tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_rpy.x, gtruth_rpy.y, gtruth_rpy.z);

            gtruth_rpy_bias.x = gtruth_rpy_bias.x + gtruth_rpy.x;
            gtruth_rpy_bias.y = gtruth_rpy_bias.y + gtruth_rpy.y;
            gtruth_rpy_bias.z = gtruth_rpy_bias.z + gtruth_rpy.z;

            // std::cout<<"bias_c = "<<bias_c<<std::endl;
            // std::cout<<"delta_r_tol = "<<gtruth_rpy_bias.x<<" delta_p_tol = "<<gtruth_rpy_bias.y<<" delta_y_tol = "<<gtruth_rpy_bias.z<<std::endl;
            // std::cout<<"delta_t_tol = "<<gtruth_time_delay<<" delta_x_tol = "<<gtruth_pos_bias.x<<" delta_y_tol = "<<gtruth_pos_bias.y<<" delta_z_tol = "<<gtruth_pos_bias.z<<std::endl;
        }
        else
        {
            gtruth_time_delay = abs(gtruth_t - current_t);
            seq = 0;
            bias_c = 0;
            std::cout<<"time delay haven't stable"<<std::endl;
        }
    }

    void gtruth_const_bias_cal()
    {
        bias_c += 1;

        gtruth_pos_bias.x = gtruth_pos_bias.x + gtruth.pose.pose.position.x;
        gtruth_pos_bias.y = gtruth_pos_bias.y + gtruth.pose.pose.position.y;
        gtruth_pos_bias.z = gtruth_pos_bias.z + gtruth.pose.pose.position.z;

        // tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
        // tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_rpy.x, gtruth_rpy.y, gtruth_rpy.z);

        // gtruth_rpy_bias.x = gtruth_rpy_bias.x + gtruth_rpy.x;
        // gtruth_rpy_bias.y = gtruth_rpy_bias.y + gtruth_rpy.y;
        // gtruth_rpy_bias.z = gtruth_rpy_bias.z + gtruth_rpy.z;

        gtruth_qua_bias.w() += gtruth.pose.pose.orientation.w;
        gtruth_qua_bias.x() += gtruth.pose.pose.orientation.x;
        gtruth_qua_bias.y() += gtruth.pose.pose.orientation.y;
        gtruth_qua_bias.z() += gtruth.pose.pose.orientation.z;

        // std::cout<<"bias_c = "<<bias_c<<std::endl;
        // std::cout<<"delta_r_tol = "<<gtruth_rpy_bias.x<<" delta_p_tol = "<<gtruth_rpy_bias.y<<" delta_y_tol = "<<gtruth_rpy_bias.z<<std::endl;
        // std::cout<<"delta_t_tol = "<<gtruth_time_delay<<" delta_x_tol = "<<gtruth_pos_bias.x<<" delta_y_tol = "<<gtruth_pos_bias.y<<" delta_z_tol = "<<gtruth_pos_bias.z<<std::endl;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        imu = *imuMsg;
        imusubTri = true;
    }

    void imuvel_cal(geometry_msgs::Vector3 &vel)
    {
        int data_c = imu_t.size();
        int order = 2;

        if(imu_t.size() > (fit_size - 1) )
        {
            imu_t.erase(imu_t.begin());
            imuvel_x.erase(imuvel_x.begin());
            imuvel_y.erase(imuvel_y.begin());
            imuvel_z.erase(imuvel_z.begin());
        }

        if(imu_t.size() < 1)
        {
            imu_t0 = imu.header.stamp.toSec();
        }

        imu_t.emplace_back(imu.header.stamp.toSec() - imu_t0);

        if(data_c > order)
        {
            double imu_dur;
            
            imu_dur = imu_t[data_c - 1] - imu_t[data_c - 2];
            vel.x = imuvel_x[data_c - 1] + imu.linear_acceleration.x * imu_dur;
            vel.y = imuvel_y[data_c - 1] + imu.linear_acceleration.y * imu_dur;
            vel.z = imuvel_z[data_c - 1] + imu.linear_acceleration.z * imu_dur;

            // imuvelrawPub.publish(vel); //test
            
            imuvel_x.emplace_back(vel.x);
            imuvel_y.emplace_back(vel.y);
            imuvel_z.emplace_back(vel.z);

            fit.polyfit(imu_t, imuvel_x, order, false);
            double fit_x = fit.getY(imu_t[data_c - 1]);
            vel.x = imuvel_x[data_c - 1] - fit.getY(imu_t[data_c - 1]);

            fit.polyfit(imu_t, imuvel_y, order, false);
            double fit_y = fit.getY(imu_t[data_c - 1]);
            vel.y = imuvel_y[data_c - 1] - fit.getY(imu_t[data_c - 1]);

            fit.polyfit(imu_t, imuvel_z, order, false);
            double fit_z = fit.getY(imu_t[data_c - 1]);
            vel.z = imuvel_z[data_c - 1] - fit.getY(imu_t[data_c - 1]);
        }
        else
        {
            imuvel_x.resize(order+1, 0);
            imuvel_y.resize(order+1, 0);
            imuvel_z.resize(order+1, 0);
        }     
    }

    // void qua_cal_mc(geometry_msgs::Quaternion& gtruth_qua)
    // {
    //     Eigen::Quaterniond gtruth_orientation;

    //     gtruth_orientation.w() = gtruth.pose.pose.orientation.w;
    //     gtruth_orientation.x() = gtruth.pose.pose.orientation.x;
    //     gtruth_orientation.y() = gtruth.pose.pose.orientation.y;
    //     gtruth_orientation.z() = gtruth.pose.pose.orientation.z;

    //     gtruth_qua = gtruth_qua_bias.inverse() * gtruth_orientation;
    //     // tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
    //     // tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_rpy.x, gtruth_rpy.y, gtruth_rpy.z);

    //     // gtruth_rpy.x = gtruth_rpy.x - gtruth_rpy_bias.x;
    //     // gtruth_rpy.y = gtruth_rpy.y - gtruth_rpy_bias.y;
    //     // gtruth_rpy.z = gtruth_rpy.z - gtruth_rpy_bias.z;

    //     // std::cout<<"roll = "<<gtruth_rpy.y<<" pitch = "<<-gtruth_rpy.x<<" yaw = "<<gtruth_rpy.z<<std::endl;

    //     // gtruth_qua = tf::createQuaternionMsgFromRollPitchYaw(gtruth_rpy.y, - gtruth_rpy.x, gtruth_rpy.z); //mocap system has y front and x right 
    // }
    void motion_cal(Eigen::Quaterniond& qua)
    {
        Eigen::Quaterniond gtruth_orientation;
        gtruth_orientation.w() = gtruth.pose.pose.orientation.w;
        gtruth_orientation.x() = gtruth.pose.pose.orientation.x;
        gtruth_orientation.y() = gtruth.pose.pose.orientation.y;
        gtruth_orientation.z() = gtruth.pose.pose.orientation.z;

        qua = gtruth_qua_bias.inverse() * gtruth_orientation;
    }

    void motion_cal(Eigen::Quaterniond& qua, Eigen::Vector3d& pos, Eigen::Vector3d& lin_vel, Eigen::Vector3d& ang_vel)
    {
        Eigen::Quaterniond gtruth_orientation;
        Eigen::Vector3d p(gtruth.pose.pose.position.x, gtruth.pose.pose.position.y, gtruth.pose.pose.position.z);
        Eigen::Vector3d p_bias(gtruth_pos_bias.x, gtruth_pos_bias.y, gtruth_pos_bias.z);
        Eigen::Vector3d l_v(gtruth.twist.twist.linear.x, gtruth.twist.twist.linear.y, gtruth.twist.twist.linear.z);
        Eigen::Vector3d a_v(gtruth.twist.twist.angular.x, gtruth.twist.twist.angular.y, gtruth.twist.twist.angular.z);
        // Eigen::Vector3d l_v(0,0,0);
        // Eigen::Vector3d a_v(0,0,0);

        gtruth_orientation.w() = gtruth.pose.pose.orientation.w;
        gtruth_orientation.x() = gtruth.pose.pose.orientation.x;
        gtruth_orientation.y() = gtruth.pose.pose.orientation.y;
        gtruth_orientation.z() = gtruth.pose.pose.orientation.z;

        qua = gtruth_qua_bias.inverse() * gtruth_orientation;
        pos = gtruth_qua_bias.inverse() * (p - p_bias);
        lin_vel = gtruth_qua_bias.inverse() * l_v;
        ang_vel = gtruth_qua_bias.inverse() * a_v;
    }

    void mc_odom_pub(const ros::TimerEvent& time_event)
    {
        // std::cout<<"---------------"<<std::endl;
        if(imusubTri && gtruthsubTri)
        {
            seq += 1;
            double gtruth_t = gtruth.header.stamp.toSec();

            if(seq < 300)
            {
                gtruth_const_bias_cal(gtruth_t);    
            }
            else if(seq == 300)
            {
                gtruth_time_delay = gtruth_time_delay / bias_c;
                gtruth_pos_bias.x = gtruth_pos_bias.x / bias_c;
                gtruth_pos_bias.y = gtruth_pos_bias.y / bias_c;
                gtruth_pos_bias.z = gtruth_pos_bias.z / bias_c;
                gtruth_rpy_bias.x = gtruth_rpy_bias.x / bias_c;
                gtruth_rpy_bias.y = gtruth_rpy_bias.y / bias_c;
                gtruth_rpy_bias.z = gtruth_rpy_bias.z / bias_c;

                gtruth_qua_bias = Eigen::AngleAxisd(gtruth_rpy_bias.z, Eigen::Vector3d::UnitZ()) * 
                    Eigen::AngleAxisd(gtruth_rpy_bias.y, Eigen::Vector3d::UnitY()) * 
                    Eigen::AngleAxisd(gtruth_rpy_bias.x, Eigen::Vector3d::UnitX());

                ROS_INFO("[odom_remap]:Odom const bias cal succeed, ready to flight!");

                std::cout<<"delta_r = "<<gtruth_rpy_bias.x * 180 / 3.14159265<<" delta_p = "<<gtruth_rpy_bias.y * 180 / 3.14159265<<" delta_y = "<<gtruth_rpy_bias.z * 180 / 3.14159265<<std::endl;
                // std::cout<<"delta_t = "<<gtruth_time_delay<<" delta_x = "<<gtruth_pos_bias.x<<" delta_y = "<<gtruth_pos_bias.y<<" delta_z = "<<gtruth_pos_bias.z<<std::endl;
            }
            else
            {
                //  std::cout<<"truth_data_delta= "<<abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec())<<std::endl;
                // std::cout<<"dur_judge = "<<abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec())<<std::endl;
                if( (abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec()) < 0.05))
                {
                    geometry_msgs::Vector3 vel;
                    nav_msgs::OdometryPtr odomMsg(new nav_msgs::Odometry);
                    Eigen::Quaterniond gtruth_qua;

                    imuvel_cal(vel);
                    motion_cal(gtruth_qua);

                    odomMsg->header.stamp = ros::Time().fromSec(gtruth_t - gtruth_time_delay);
                    odomMsg->pose.pose.position.x = gtruth.pose.pose.position.y - gtruth_pos_bias.y;
                    odomMsg->pose.pose.position.y = -(gtruth.pose.pose.position.x - gtruth_pos_bias.x);
                    odomMsg->pose.pose.position.z = gtruth.pose.pose.position.z - gtruth_pos_bias.z;
                    odomMsg->pose.pose.orientation.w = gtruth_qua.w();
                    odomMsg->pose.pose.orientation.x = gtruth_qua.x();
                    odomMsg->pose.pose.orientation.y = gtruth_qua.y();
                    odomMsg->pose.pose.orientation.z = gtruth_qua.z();
                    odomMsg->twist.twist.linear.x = vel.x;
                    odomMsg->twist.twist.linear.y = vel.y;
                    odomMsg->twist.twist.linear.z = vel.z;
                    odomMsg->twist.twist.angular.x = 0;
                    odomMsg->twist.twist.angular.y = 0;
                    odomMsg->twist.twist.angular.z = 0;

                    odomPub.publish(odomMsg);
                }
                else
                {
                    ROS_WARN("[odom_remap]:Truth data delay too high!");
                    std::cout<<"truth_data_delay= "<<abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec())<<std::endl;
                }
            }
        }
        else
        {
            ROS_ERROR("[odom_remap]:No odom or imu data, please check rostopic.");
            while(!imusubTri || !gtruthsubTri)
            {
                ros::Duration(0.2).sleep();
            }
        }
        // gtruth_time_l = gtruth.header.stamp.toSec();
    }

    void gps_odom_pub(const ros::TimerEvent& time_event)
    {
        if(gtruthsubTri)
        {
            seq += 1;

            if(seq < 300)
            {
                gtruth_const_bias_cal();    
            }
            else if(seq == 300)
            {
                gtruth_pos_bias.x = gtruth_pos_bias.x / bias_c;
                gtruth_pos_bias.y = gtruth_pos_bias.y / bias_c;
                gtruth_pos_bias.z = gtruth_pos_bias.z / bias_c;
                // gtruth_rpy_bias.x = gtruth_rpy_bias.x / bias_c;
                // gtruth_rpy_bias.y = gtruth_rpy_bias.y / bias_c;
                // gtruth_rpy_bias.z = gtruth_rpy_bias.z / bias_c;
                // gtruth_qua_bias = Eigen::AngleAxisd(gtruth_rpy_bias.z, Eigen::Vector3d::UnitZ()) * 
                //                     Eigen::AngleAxisd(gtruth_rpy_bias.y, Eigen::Vector3d::UnitY()) * 
                //                     Eigen::AngleAxisd(gtruth_rpy_bias.x, Eigen::Vector3d::UnitX());
                gtruth_qua_bias.w() = gtruth_qua_bias.w() / bias_c;
                gtruth_qua_bias.x() = gtruth_qua_bias.x() / bias_c;
                gtruth_qua_bias.y() = gtruth_qua_bias.y() / bias_c;
                gtruth_qua_bias.z() = gtruth_qua_bias.z() / bias_c;

                ROS_INFO("[odom_remap]:Odom const bias cal succeed, ready to flight!");

                std::cout<<"delta_x = "<<gtruth_pos_bias.x<<" delta_y = "<<gtruth_pos_bias.y<<" delta_z = "<<gtruth_pos_bias.z<<std::endl;
                Eigen::Matrix3d rx = gtruth_qua_bias.toRotationMatrix();
                Eigen::Vector3d eular_bias = rx.eulerAngles(2,1,0);
                std::cout<<"delta_r = "<<eular_bias[0] * 180 / 3.14159265<<" delta_p = "<<eular_bias[1] * 180 / 3.14159265<<" delta_y = "<<eular_bias[2] * 180 / 3.14159265<<std::endl;
            }
            else
            {
                if( (abs(gtruth.header.stamp.toSec() - gtruth_time_l) < 0.04))
                {
                    nav_msgs::OdometryPtr odomMsg(new nav_msgs::Odometry);
                    Eigen::Quaterniond gtruth_qua;
                    Eigen::Vector3d position;
                    Eigen::Vector3d lin_vel;
                    Eigen::Vector3d ang_vel;

                    motion_cal(gtruth_qua, position, lin_vel, ang_vel);

                    odomMsg->header = gtruth.header;
                    odomMsg->child_frame_id = gtruth.child_frame_id;
                    odomMsg->pose.pose.position.x = position[0];
                    odomMsg->pose.pose.position.y = position[1];
                    odomMsg->pose.pose.position.z = position[2];
                    odomMsg->pose.covariance = gtruth.pose.covariance;
                    odomMsg->pose.pose.orientation.w = gtruth_qua.w();
                    odomMsg->pose.pose.orientation.x = gtruth_qua.x();
                    odomMsg->pose.pose.orientation.y = gtruth_qua.y();
                    odomMsg->pose.pose.orientation.z = gtruth_qua.z();
                    odomMsg->twist.twist.linear.x = lin_vel[0];
                    odomMsg->twist.twist.linear.y = lin_vel[1];
                    odomMsg->twist.twist.linear.z = lin_vel[2];
                    odomMsg->twist.twist.angular.x = ang_vel[0];
                    odomMsg->twist.twist.angular.y = ang_vel[1];
                    odomMsg->twist.twist.angular.z = ang_vel[2];

                    odomPub.publish(odomMsg);

                    // tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, gtruth_Q2T);
                    // tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_rpy.x, gtruth_rpy.y, gtruth_rpy.z);
                    // std::cout<<"roll = "<<gtruth_rpy.x * 180 / 3.14159265<<" pitch = "<<gtruth_rpy.y * 180 / 3.14159265<<" yaw = "<<gtruth_rpy.z * 180 / 3.14159265<<std::endl;
                }
                else
                {
                    ROS_WARN("[odom_remap]:odom data update rate is too low!");
                    std::cout<<"truth_data_delay= "<<abs(gtruth.header.stamp.toSec() - gtruth_time_l)<<std::endl; //test
                }
            }
        }
        else
        {
            ROS_ERROR("[odom_remap]:No odom data, please check rostopic.");
            while(!gtruthsubTri)
            {
                ros::Duration(0.2).sleep();
            }
        }
        gtruth_time_l = gtruth.header.stamp.toSec();
    }

    void init(ros::NodeHandle& nh)
    {
        int odom_source;
        nh.getParam("gtruthTopic", gtruthTopic);
        nh.param("fit_size", fit_size, 100);
        nh.param("odom_source", odom_source, 0); // 0 for mocap, 1 for gps

        gtruth_pos_bias.x = 0;
        gtruth_pos_bias.y = 0;
        gtruth_pos_bias.z = 0;
        gtruth_rpy_bias.x = 0;
        gtruth_rpy_bias.y = 0;
        gtruth_rpy_bias.z = 0;

        imuSub = nh.subscribe("/mavros/imu/data", 10, &odomRemap::imuCallback, this);

        odomPub = nh.advertise<nav_msgs::Odometry>("/odom/remap/test", 10);
        // imuvelrawPub = nh.advertise<geometry_msgs::Vector3>("/mavros/imu/data/linear_velocity_raw",10);//test

        switch (odom_source) {
            case 0:
                gtruthSub = nh.subscribe(gtruthTopic, 10, &odomRemap::mctruthCallback, this,
                                   ros::TransportHints().tcpNoDelay());
                odom_timer = nh.createTimer(ros::Duration(0.005), &odomRemap::mc_odom_pub, this);
                ROS_INFO("[odom_remap]:Using Mocap for odom calculate.");
                break;
            case 1:
                gtruthSub = nh.subscribe(gtruthTopic, 10, &odomRemap::gpstruthCallback, this,
                                    ros::TransportHints().tcpNoDelay());

                // debugSub = nh.subscribe("/mavros/local_position/pose", 10, &odomRemap::localposeCallback, this,
                //                     ros::TransportHints().tcpNoDelay()); // debug
                odom_timer = nh.createTimer(ros::Duration(0.005), &odomRemap::gps_odom_pub, this);
                ROS_INFO("[odom_remap]:Using GPS for odom calculate.");
                break;
        }
    }

public:
    void onInit(void)
    {
        ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
        initThread_ = std::thread(std::bind(&odomRemap::init, this, nh)); //在单独的线程中运行Nodelet::init()      
    }
};
} // namespace odomRemap
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(odomRemap::odomRemap, nodelet::Nodelet);