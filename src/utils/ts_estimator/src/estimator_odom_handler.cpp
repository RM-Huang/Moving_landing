#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/EstimatorOdom.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <traj_opt/minco.hpp>

#include <Eigen/Core>
#include <atomic>
#include <thread>
#include <vis_utils/vis_utils.hpp>

namespace sim_odom {

static Eigen::Vector3d g(0, 0, -9.8);

class SimOdom : public nodelet::Nodelet {
private:
    std::thread initThread_;
    ros::Publisher odom_pub;
    ros::Publisher car_truth_pub;
    ros::Subscriber uav_sub;
    ros::Subscriber car_sub;
    ros::Timer timer_;

    std::shared_ptr<vis_utils::VisUtils> visPtr_;
    minco::MINCO_S4_Uniform mincoOpt_;
    // minco::MINCO_S4_Uniform mincoOpt_car;
    Trajectory traj_u;
    Trajectory traj_c;

    bool use_sim_uav = true;
    bool use_sim_car = true;
    nav_msgs::Odometry uav_sub_msg;
    nav_msgs::Odometry car_sub_msg;
    std::vector<nav_msgs::Odometry> car_odom_list;
    bool car_sub_tri = false;
    bool uav_sub_tri = false;

    // traj param
    Eigen::MatrixXd P_u;
    double T_u;
    Eigen::MatrixXd P_c;
    double T_c;
    double R_c;
    double v_c;
    double omega_c;
    double t_0;

    // bias param
    double time_delay = 0;
    Eigen::Vector3d pos_bias;
    Eigen::Quaterniond q_bias;

    void vec_2_Matrix(const std::vector<std::vector<double>>& vec, Eigen::MatrixXd& mat){
      int rows = vec.size();
      int cols = vec[0].size();
      mat = Eigen::MatrixXd::Zero(rows, cols);

      for (int i = 0; i < rows; ++i) {
          for (int j = 0; j < cols; ++j) {
              mat(i, j) = vec[i][j];
          }
      }
    }

    bool v2q(const Eigen::Vector3d& v, Eigen::Quaterniond& q){
      double a = v.x();
      double b = v.y();
      double c = v.z();
      if (c == -1) {
        return false;
      }
      double d = 1.0 / sqrt(2.0 * (1 + c));
      q.w() = (1 + c) * d;
      q.x() = -b * d;
      q.y() = a * d;
      q.z() = 0;
      return true;
    }

    // void uav_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    //   uav_sub_msg = *msg;
    //   if(!uav_sub_tri){
    //     uav_sub_tri = true;
    //     ROS_INFO("\033[32m[estimator_odom_handler]:uav odom received!\033[32m");
    //   }
    // }
    void uav_odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
      uav_sub_msg.header = msg->header;
      uav_sub_msg.pose.pose = msg->pose;
      if(!uav_sub_tri){
        uav_sub_tri = true;
        ROS_INFO("\033[32m[estimator_odom_handler]:uav odom received!\033[32m");
      }
    }

    void car_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
      car_sub_msg = *msg;
      car_odom_list.push_back(car_sub_msg);
      if(!car_sub_tri){
        car_sub_tri = true;
        ROS_INFO("\033[32m[estimator_odom_handler]:car odom received!\033[32m");
      }
    }

    void generate_uav_traj(){
      mincoOpt_.reset(P_u.cols() - 1);
      Eigen::MatrixXd initS = Eigen::MatrixXd::Zero(3, 4);
      initS.col(0) = P_u.col(0);
      Eigen::MatrixXd tailS = initS;
      Eigen::MatrixXd p_u = P_u.block(0, 1, P_u.rows(), P_u.cols() - 1);
      mincoOpt_.generate(initS, tailS, p_u, T_u);
      traj_u = mincoOpt_.getTraj();
      visPtr_->visualize_traj(traj_u, "traj");
    }

    void generate_car_traj(){
      mincoOpt_.reset(P_c.cols() - 1);
      Eigen::MatrixXd initS = Eigen::MatrixXd::Zero(3, 4);
      initS.col(0) = P_c.col(0);
      Eigen::MatrixXd tailS = initS;
      Eigen::MatrixXd p_c = P_c.block(0, 1, P_c.rows(), P_c.cols() - 1);
      mincoOpt_.generate(initS, tailS, p_c, T_c);
      traj_c = mincoOpt_.getTraj();
      // std::cout << " car_traj_dur : " << traj_c.getTotalDuration() << std::endl;
      visPtr_->visualize_traj(traj_c, "car_traj");
    }

    void get_car_odom(const double t_cur, bool set_bias, nav_msgs::Odometry& car_odom){
      Eigen::Vector3d p_tmp, v_tmp;
      Eigen::Quaterniond q_tmp;

      if(use_sim_car){
        // double theta = omega_c * t;       // 当前旋转角度
        // double cos_theta = cos(theta);
        // double sin_theta = sin(theta);

        // p_tmp << R_c * cos_theta, R_c * sin_theta, 0;
        // v_tmp << -v_c * sin_theta, v_c * cos_theta, 0;

        // double yaw = theta + M_PI_2;
        // // 标准化角度到[0, 2π)范围
        // yaw = fmod(yaw, 2*M_PI);
        // if (yaw < 0) {
        //     yaw += 2*M_PI;
        // }
        // Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
        // Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
        // Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        // q_tmp = yawAngle * pitchAngle * rollAngle;

        double t = t_cur - std::floor(t_cur / traj_c.getTotalDuration()) * traj_c.getTotalDuration();
        // car_odom.header.stamp = ros::Time().fromSec(t_cur);
        p_tmp = traj_c.getPos(t);
        v_tmp = traj_c.getVel(t);

        double yaw = std::atan2(v_tmp(1), v_tmp(0));
        if (yaw < 0) {
            yaw += 2 * M_PI;
        }
        Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        q_tmp = yawAngle * pitchAngle * rollAngle;
        q_tmp.normalize();
        
        if(set_bias){
          p_tmp = q_bias * p_tmp + pos_bias;
          v_tmp = q_bias * v_tmp;
          q_tmp = q_bias * q_tmp;
        }
      }else{
        if(set_bias){
          double t_cur = car_sub_msg.header.stamp.toSec();
          while(t_cur - car_odom_list[0].header.stamp.toSec() > time_delay){
            car_odom_list.erase(car_odom_list.begin());
          }
          p_tmp << car_odom_list[0].pose.pose.position.x, car_odom_list[0].pose.pose.position.y, car_odom_list[0].pose.pose.position.z;
          v_tmp << car_odom_list[0].twist.twist.linear.x, car_odom_list[0].twist.twist.linear.y, car_odom_list[0].twist.twist.linear.z;
          q_tmp.coeffs() << car_odom_list[0].pose.pose.orientation.x, car_odom_list[0].pose.pose.orientation.y, 
                            car_odom_list[0].pose.pose.orientation.z, car_odom_list[0].pose.pose.orientation.w;

          p_tmp = q_bias * p_tmp + pos_bias;
          v_tmp = q_bias * v_tmp;
          q_tmp = q_bias * q_tmp;
        }else{
          p_tmp << car_sub_msg.pose.pose.position.x, car_sub_msg.pose.pose.position.y, car_sub_msg.pose.pose.position.z;
          v_tmp << car_sub_msg.twist.twist.linear.x, car_sub_msg.twist.twist.linear.y, car_sub_msg.twist.twist.linear.z;
          q_tmp.coeffs() << car_sub_msg.pose.pose.orientation.x, car_sub_msg.pose.pose.orientation.y, 
                            car_sub_msg.pose.pose.orientation.z, car_sub_msg.pose.pose.orientation.w;
        }
        // car_odom.header.stamp = car_sub_msg.header.stamp;
      }
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
      visPtr_->visualize_traj(traj_c, "car_traj");
    }

    void get_uav_odom(const double t_cur, Eigen::Vector3d& p, Eigen::Quaterniond& q, nav_msgs::Odometry& uav_odom){
      //TODO unsim odom handle
      if(use_sim_uav){
        double t = t_cur - std::floor(t_cur / traj_u.getTotalDuration()) * traj_u.getTotalDuration();
        p = traj_u.getPos(t);
        Eigen::Vector3d v = traj_u.getVel(t);
        Eigen::Vector3d a = traj_u.getAcc(t);
        Eigen::Vector3d j = traj_u.getJer(t);
        Eigen::Vector3d thrust = a - g;
        Eigen::Vector3d zb = thrust.normalized();
        bool no_singlarity = v2q(zb, q);
        if (no_singlarity) {
          uav_odom.pose.pose.position.x = p.x();
          uav_odom.pose.pose.position.y = p.y();
          uav_odom.pose.pose.position.z = p.z();
          uav_odom.pose.pose.orientation.w = q.w();
          uav_odom.pose.pose.orientation.x = q.x();
          uav_odom.pose.pose.orientation.y = q.y();
          uav_odom.pose.pose.orientation.z = q.z();
          uav_odom.twist.twist.linear.x = v.x();
          uav_odom.twist.twist.linear.y = v.y();
          uav_odom.twist.twist.linear.z = v.z();
          uav_odom.header.stamp = ros::Time::now();
          uav_odom.header.frame_id = "world";
          visPtr_->visualize_traj(traj_u, "traj");
        }
      }else{
        uav_odom = uav_sub_msg;
      }
      visPtr_->pub_msg(uav_odom, "odom");
    }

    void timer_callback(const ros::TimerEvent& event){
      if(car_sub_tri && uav_sub_tri){
        double t = ros::Time::now().toSec() - t_0 + time_delay;
        quadrotor_msgs::EstimatorOdom odom_msg;
        nav_msgs::Odometry car_truth;
        Eigen::Quaterniond uav_q;
        Eigen::Vector3d uav_p, car_p;

        // odom_msg.car_odom.header.frame_id = "world";
        // odom_msg.uav_odom.header.frame_id = "world";

        get_car_odom(t - time_delay, true, odom_msg.car_odom);
        visPtr_->pub_msg(odom_msg.car_odom, "fake_target");

        get_uav_odom(t, uav_p, uav_q, odom_msg.uav_odom);

        get_car_odom(t, false, car_truth);
        visPtr_->pub_msg(car_truth, "target");

        car_p << car_truth.pose.pose.position.x, car_truth.pose.pose.position.y, car_truth.pose.pose.position.z;
        // Eigen::Vector3d dir = uav_q.inverse() * (car_p - uav_p);
        Eigen::Vector3d dir = (car_p - uav_p);
        dir.normalize();
        odom_msg.dir_uc.x = dir(0);
        odom_msg.dir_uc.y = dir(1);
        odom_msg.dir_uc.z = dir(2);
        // std::cout << "err = " << (dir * (car_p - uav_p).norm() + uav_p - car_p).transpose() << std::endl;

        odom_pub.publish(odom_msg);
        car_truth_pub.publish(car_truth);
      }
    }

    void init(ros::NodeHandle& nh) {
      std::vector<std::vector<double>> p_u_vec(3);
      std::vector<std::vector<double>> p_c_vec(3);
      int pub_hz_;
      double trans_x, trans_y, trans_z, rotat_roll, rotat_pitch, rotat_yaw;
      nh.param("use_sim_uav", use_sim_uav, true);
      nh.param("use_sim_car", use_sim_car, true);

      nh.getParam("pub_hz_", pub_hz_);
      nh.getParam("t_uav_per", T_u);
      nh.getParam("t_car_per", T_c);
      nh.getParam("car_R", R_c);
      nh.getParam("car_V", v_c);
      nh.getParam("uav_P_x", p_u_vec[0]);
      nh.getParam("uav_P_y", p_u_vec[1]);
      nh.getParam("uav_P_z", p_u_vec[2]);
      nh.getParam("car_P_x", p_c_vec[0]);
      nh.getParam("car_P_y", p_c_vec[1]);
      nh.getParam("car_P_z", p_c_vec[2]);

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

      visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
      vec_2_Matrix(p_u_vec, P_u);
      vec_2_Matrix(p_c_vec, P_c);
      omega_c = v_c / R_c;
      t_0 = ros::Time::now().toSec();

      // mincoOpt_uav.reset(P_u.cols() - 1);
      // mincoOpt_car.reset(P_c.cols() - 1);

      if(use_sim_uav){
        uav_sub_tri = true;
        generate_uav_traj();
      }else{
        uav_sub = nh.subscribe("uav_odom_topic", 5, &SimOdom::uav_odom_callback, this, ros::TransportHints().tcpNoDelay());
      }

      if(use_sim_car){
        car_sub_tri = true;
        generate_car_traj();
      }else{
        car_sub = nh.subscribe("car_odom_topic", 5, &SimOdom::car_odom_callback, this, ros::TransportHints().tcpNoDelay());
      }
      
      odom_pub = nh.advertise<quadrotor_msgs::EstimatorOdom>("/estimator/sim_odom", 1);
      car_truth_pub = nh.advertise<nav_msgs::Odometry>("/estimator/car_truth", 1);

      timer_ = nh.createTimer(ros::Duration(1.0 / pub_hz_), &SimOdom::timer_callback, this);

      ROS_INFO("\033[32mSimulation odom publisher initialized!\033[32m");
    }

public:
    void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&SimOdom::init, this, nh));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sim_odom::SimOdom, nodelet::Nodelet);