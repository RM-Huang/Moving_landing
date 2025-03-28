#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/EstimatorOdom.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>
#include <mutex>

namespace add_bias {

class AddBias : public nodelet::Nodelet {
private:
    std::thread initThread_;
    ros::Publisher odom_pub;
    ros::Publisher car_truth_pub;
    ros::Subscriber uav_sub;
    ros::Subscriber car_sub;
    ros::Timer timer_;
    std::mutex uav_odom_mutex;
    std::mutex car_odom_mutex;

    nav_msgs::Odometry uav_sub_msg;
    nav_msgs::Odometry car_sub_msg;
    std::vector<nav_msgs::Odometry> car_odom_list;
    bool car_sub_tri = false;
    bool uav_sub_tri = false;

    // bias param
    double time_delay = 0;
    Eigen::Vector3d pos_bias;
    Eigen::Quaterniond q_bias;

    // time syn setup
    // message_filters::Subscriber<nav_msgs::Odometry> uav_sub;
    // message_filters::Subscriber<nav_msgs::Odometry> car_sub;
    // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> SyncPolicy;
    // std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    void uav_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
      uav_sub_msg = *msg;
      if(!uav_sub_tri){
        uav_sub_tri = true;
        ROS_INFO("\033[32m[estimator_odom_handler]:uav odom received!\033[32m");
      }
    }
    // void uav_odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //   uav_sub_msg.header = msg->header;
    //   uav_sub_msg.pose.pose = msg->pose;
    //   if(!uav_sub_tri){
    //     uav_sub_tri = true;
    //     ROS_INFO("\033[32m[estimator_odom_handler]:uav odom received!\033[32m");
    //   }
    // }

    // void uav_odom_callback(const gazebo_msgs::ModelStates::ConstPtr &modelMsg){
    //   for(int i = 0; i < modelMsg->name.size(); i++){
    //     if(modelMsg->name[i] == "iris_0"){
    //       // std::lock_guard<std::mutex> lock(uav_odom_mutex);
    //       uav_sub_msg.header.stamp = ros::Time::now();
    //       uav_sub_msg.pose.pose = modelMsg->pose[i];
    //       uav_sub_msg.twist.twist = modelMsg->twist[i];
    //       uav_sub_tri = true;
    //       return;
    //     }
    //   }
    // }

    void car_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
      // std::lock_guard<std::mutex> lock(car_odom_mutex);
      car_sub_msg = *msg;
      car_sub_tri = true;
    }

    void syncCallback(const nav_msgs::Odometry::ConstPtr& uav_msg, const nav_msgs::OdometryConstPtr& car_msg) {
      uav_sub_msg = *uav_msg;
      car_sub_msg = *car_msg;
      uav_sub_tri = true;
      car_sub_tri = true;
    }

    void get_car_odom(const nav_msgs::Odometry& car_cur, nav_msgs::Odometry& car_odom){
      Eigen::Vector3d p_tmp, v_tmp;
      Eigen::Quaterniond q_tmp;

      p_tmp << car_odom_list[0].pose.pose.position.x, car_odom_list[0].pose.pose.position.y, car_odom_list[0].pose.pose.position.z;
      v_tmp << car_odom_list[0].twist.twist.linear.x, car_odom_list[0].twist.twist.linear.y, car_odom_list[0].twist.twist.linear.z;
      q_tmp.coeffs() << car_odom_list[0].pose.pose.orientation.x, car_odom_list[0].pose.pose.orientation.y, 
                      car_odom_list[0].pose.pose.orientation.z, car_odom_list[0].pose.pose.orientation.w;

      p_tmp = q_bias * p_tmp + pos_bias;
      v_tmp = q_bias * v_tmp;
      q_tmp = q_bias * q_tmp;

      car_odom.pose.pose.position.x = p_tmp(0);
      car_odom.pose.pose.position.y = p_tmp(1);
      car_odom.pose.pose.position.z = p_tmp(2);
      car_odom.twist.twist.linear.x = v_tmp(0);
      car_odom.twist.twist.linear.y = v_tmp(1);
      car_odom.twist.twist.linear.z = v_tmp(2);
      car_odom.pose.pose.orientation.w = q_tmp.w();
      car_odom.pose.pose.orientation.x = q_tmp.x();
      car_odom.pose.pose.orientation.y = q_tmp.y();
      car_odom.pose.pose.orientation.z = q_tmp.z();
      car_odom.header.frame_id = "world";
      car_odom.header.stamp = ros::Time::now();
    }

    bool read_odom(nav_msgs::Odometry& uav_cur, nav_msgs::Odometry& car_cur){
      // std::lock_guard<std::mutex> lock_uav(uav_odom_mutex);
      // std::lock_guard<std::mutex> lock_car(car_odom_mutex);
      // double t_uav = uav_sub_msg.header.stamp.toSec();
      // double t_car = car_sub_msg.header.stamp.toSec();
      // if(abs(t_uav - t_car) > 0.005){
      //   return false;
      // }
      uav_cur = uav_sub_msg;
      car_cur = car_sub_msg;
      car_odom_list.push_back(car_cur);

      double car_t_cur = car_cur.header.stamp.toSec();
      while(car_odom_list.size() > 1 && car_t_cur - car_odom_list[0].header.stamp.toSec() > time_delay){
        car_odom_list.erase(car_odom_list.begin());
      }
      return true;
    }

    void timer_callback(const ros::TimerEvent& event){
      if(car_sub_tri && uav_sub_tri){
        quadrotor_msgs::EstimatorOdomPtr odom_msg(new quadrotor_msgs::EstimatorOdom);
        // Eigen::Quaterniond uav_q;
        nav_msgs::Odometry uav_cur, car_cur;
        if(!read_odom(uav_cur, car_cur)){
          return;
        }

        odom_msg->vis_odom = car_cur; // test
        Eigen::Vector3d uav_p(car_cur.pose.pose.position.x, car_cur.pose.pose.position.y, car_cur.pose.pose.position.z);
        Eigen::Vector3d car_p(uav_cur.pose.pose.position.x, uav_cur.pose.pose.position.y, uav_cur.pose.pose.position.z);
        get_car_odom(car_cur, odom_msg->car_odom);
        odom_msg->uav_odom = uav_cur;
        odom_msg->uav_odom.header.frame_id = "world";
        // Eigen::Vector3d dir = uav_q.inverse() * (car_p - uav_p);
        Eigen::Vector3d dir = (car_p - uav_p);
        dir.normalize();
        odom_msg->dir_uc.x = dir(0);
        odom_msg->dir_uc.y = dir(1);
        odom_msg->dir_uc.z = dir(2);
        // std::cout << "err = " << (dir * (car_p - uav_p).norm() + uav_p - car_p).transpose() << std::endl;
        odom_pub.publish(odom_msg);
      }else{
        while(!car_sub_tri || !uav_sub_tri)
        {
            if(!uav_sub_tri)
            {
                ROS_ERROR("[odom_remap]:No uav odom data, please check rostopic.");
            }

            if(!car_sub_tri)
            {
                ROS_ERROR("[odom_remap]:No car odom data, please check rostopic.");
            }
            ros::Duration(1.0).sleep();
        }   
      }
    }

    void init(ros::NodeHandle& nh) {
        int pub_hz_;
        double trans_x, trans_y, trans_z, rotat_roll, rotat_pitch, rotat_yaw;

        nh.getParam("pub_hz_", pub_hz_);
        nh.param("time_delay", time_delay, 0.0);
        nh.param("trans_x", trans_x, 0.0);
        nh.param("trans_y", trans_y, 0.0);
        nh.param("trans_z", trans_z, 0.0);
        nh.param("rotat_roll", rotat_roll, 0.0);
        nh.param("rotat_pitch", rotat_pitch, 0.0);
        nh.param("rotat_yaw", rotat_yaw, 0.0);

        pos_bias << trans_x, trans_y, trans_z;
        Eigen::AngleAxisd rollAngle(rotat_roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(rotat_pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rotat_yaw, Eigen::Vector3d::UnitZ());
        q_bias = yawAngle * pitchAngle * rollAngle;

        uav_sub = nh.subscribe("uav_odom_topic", 1, &AddBias::uav_odom_callback, this, ros::TransportHints().tcpNoDelay());
        car_sub = nh.subscribe("car_odom_topic", 1, &AddBias::car_odom_callback, this, ros::TransportHints().tcpNoDelay());

        // uav_sub.subscribe(nh, "uav_odom_topic", 1);
        // car_sub.subscribe(nh, "car_odom_topic", 1);
        // sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), uav_sub, car_sub);
        // sync->registerCallback(boost::bind(&AddBias::syncCallback, this, _1, _2));
        
        odom_pub = nh.advertise<quadrotor_msgs::EstimatorOdom>("/estimator/sim_odom", 1);
        // car_truth_pub = nh.advertise<nav_msgs::Odometry>("/estimator/car_truth", 1);

        timer_ = nh.createTimer(ros::Duration(1.0 / pub_hz_), &AddBias::timer_callback, this);

        ROS_INFO("\033[32m[add_bias]: Car odom publisher initialized!\033[32m");
    }

public:
    void onInit(void) {
      ros::NodeHandle nh(getMTPrivateNodeHandle());
      initThread_ = std::thread(std::bind(&AddBias::init, this, nh));
    }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(add_bias::AddBias, nodelet::Nodelet);