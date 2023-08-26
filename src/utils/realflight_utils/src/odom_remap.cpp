#include <thread>
#include <ros/ros.h>
#include <tf/tf.h>
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

    geometry_msgs::PoseStamped gtruth;
    geometry_msgs::Vector3 gtruth_pos_bias;
    geometry_msgs::Vector3 gtruth_rpy_bias;
    geometry_msgs::Vector3 gtruth_rpy;
    tf::Quaternion gtruth_Q2T;
          
    sensor_msgs::Imu imu;

    ros::Subscriber gtruthSub;
    ros::Subscriber imuSub;

    std::string gtruthTopic;

    ros::Publisher odomPub;
    ros::Publisher imuvelrawPub; // publish imu_vel msg with merely acc integral

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

    void gtruthCallback(const geometry_msgs::PoseStamped::ConstPtr &gtruthMsg)
    {
        gtruth = *gtruthMsg;
        gtruthsubTri = true;
    }

    void gtruth_const_bias_cal(const double &gtruth_t)
    {
        // double current_t = ros::Time::now().toSec();
        double current_t = imu.header.stamp.toSec(); // for bag replay

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
 
        // std::cout<<"gtruth_t =  = "<<gtruth_t<<" current_t = "<<current_t<<std::endl;
        // std::cout<<"gtruth_time_delay = "<<gtruth_time_delay<<std::endl;
        // std::cout<<"flash dur = "<<flash_dur<<std::endl;

        if( flash_dur < 0.5)
        {
            bias_c += 1;

            gtruth_time_delay = gtruth_time_delay + abs(gtruth_t - current_t);

            gtruth_pos_bias.x = gtruth_pos_bias.x + gtruth.pose.position.x;
            gtruth_pos_bias.y = gtruth_pos_bias.y + gtruth.pose.position.y;
            gtruth_pos_bias.z = gtruth_pos_bias.z + gtruth.pose.position.z;

            tf::quaternionMsgToTF(gtruth.pose.orientation, gtruth_Q2T);
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
            // std::cout<<"imu_t = "<<imu_t[data_c - 1]<<std::endl;
            vel.x = imuvel_x[data_c - 1] + imu.linear_acceleration.x * imu_dur;
            vel.y = imuvel_y[data_c - 1] + imu.linear_acceleration.y * imu_dur;
            vel.z = imuvel_z[data_c - 1] + imu.linear_acceleration.z * imu_dur;
            // std::cout<<"imuvel = "<<vel.x<<" "<<vel.y<<" "<<vel.z<<std::endl;
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

            // std::cout<<"fit = "<<fit_x<<" "<<fit_y<<" "<<fit_z<<std::endl;
        }
        else
        {
            imuvel_x.resize(order+1, 0);
            imuvel_y.resize(order+1, 0);
            imuvel_z.resize(order+1, 0);
        }     
    }

    void qua_cal(geometry_msgs::Quaternion& gtruth_qua)
    {
        tf::quaternionMsgToTF(gtruth.pose.orientation, gtruth_Q2T);
        tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_rpy.x, gtruth_rpy.y, gtruth_rpy.z);

        gtruth_rpy.x = gtruth_rpy.x - gtruth_rpy_bias.x;
        gtruth_rpy.y = gtruth_rpy.y - gtruth_rpy_bias.y;
        gtruth_rpy.z = gtruth_rpy.z - gtruth_rpy_bias.z;

        std::cout<<"roll = "<<gtruth_rpy.y<<" pitch = "<<-gtruth_rpy.x<<" yaw = "<<gtruth_rpy.z<<std::endl;

        gtruth_qua = tf::createQuaternionMsgFromRollPitchYaw(gtruth_rpy.y, - gtruth_rpy.x, gtruth_rpy.z); //mocap system has y front and x right 
    }

    void odom_pub(const ros::TimerEvent& time_event)
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

                ROS_INFO("[odom_remap]:Odom const bias cal succeed, ready to flight!");

                std::cout<<"delta_r = "<<gtruth_rpy_bias.x<<" delta_p = "<<gtruth_rpy_bias.y<<" delta_y = "<<gtruth_rpy_bias.z<<std::endl;
                // std::cout<<"delta_t = "<<gtruth_time_delay<<" delta_x = "<<gtruth_pos_bias.x<<" delta_y = "<<gtruth_pos_bias.y<<" delta_z = "<<gtruth_pos_bias.z<<std::endl;
            }
            else
            {
                //  std::cout<<"truth_data_delta= "<<abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec())<<std::endl;
                // std::cout<<"dur_judge = "<<abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec())<<std::endl;
                if( (abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec()) < 0.05))
                {
                    // std::cout<<"---calculate---"<<std::endl;
                    geometry_msgs::Vector3 vel;
                    nav_msgs::OdometryPtr odomMsg(new nav_msgs::Odometry);
                    geometry_msgs::Quaternion gtruth_qua;

                    imuvel_cal(vel);
                    qua_cal(gtruth_qua);

                    odomMsg->header.stamp = ros::Time().fromSec(gtruth_t - gtruth_time_delay);
                    odomMsg->pose.pose.position.x = gtruth.pose.position.y - gtruth_pos_bias.y;
                    odomMsg->pose.pose.position.y = -(gtruth.pose.position.x - gtruth_pos_bias.x);
                    odomMsg->pose.pose.position.z = gtruth.pose.position.z - gtruth_pos_bias.z;
                    odomMsg->pose.pose.orientation = gtruth_qua;
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
        gtruth_time_l = gtruth.header.stamp.toSec();
    }

    void init(ros::NodeHandle& nh)
    {
        nh.getParam("gtruthTopic", gtruthTopic);
        nh.param("fit_size", fit_size, 100);

        gtruth_pos_bias.x = 0;
        gtruth_pos_bias.y = 0;
        gtruth_pos_bias.z = 0;
        gtruth_rpy_bias.x = 0;
        gtruth_rpy_bias.y = 0;
        gtruth_rpy_bias.z = 0;

        gtruthSub = nh.subscribe(gtruthTopic, 10, &odomRemap::gtruthCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        imuSub = nh.subscribe("/mavros/imu/data", 10, &odomRemap::imuCallback, this);

        odomPub = nh.advertise<nav_msgs::Odometry>("/odom/remap", 10);
        // imuvelrawPub = nh.advertise<geometry_msgs::Vector3>("/mavros/imu/data/linear_velocity_raw",10);//test

        odom_timer = nh.createTimer(ros::Duration(0.005), &odomRemap::odom_pub, this);
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