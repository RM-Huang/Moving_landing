#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TrajctrlTrigger.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <atomic>
#include <thread>
#include <vis_utils/vis_utils.hpp>

namespace planning {

std::string sep = "\n-----------------------";
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
Eigen::IOFormat CommaInitFmt2(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber triger_sub_;
  ros::Subscriber target_odom_sub_;
  ros::Subscriber uav_odom_sub_;
  ros::Subscriber ctrl_ready_tri_sub_;
  ros::Subscriber ctrl_start_tri_sub_;
  ros::Timer plan_timer_;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // NOTE planning or fake target
  bool target_ = false;
  Eigen::Vector3d goal_;

  // Using for planning timer
  Eigen::MatrixXd iniState;
  bool generate_new_traj_success = false;
  Trajectory traj;
  Eigen::Vector3d target_p, target_v, uav_p, uav_v;
  Eigen::Quaterniond target_q;
  Eigen::Quaterniond land_q;

  // NOTE just for debug
  bool debug_ = false;
  bool once_ = false;
  bool debug_replan_ = false;

  double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_; // for simulation
  double perching_theta_;

  Trajectory traj_poly_;
  double trajStamp;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  int plan_hz_ = 10;

  bool ctrl_ready_triger = false;
  quadrotor_msgs::TrajctrlTrigger ctrl_start_triger;
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);

  //--------------------- func ---------------------------
  static Eigen::VectorXd f_DN(const Eigen::Vector3d& x) 
  {
    double x_norm_2 = x.squaredNorm();
    return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
  }; //返回x的单位法向量

  // NOTE run vis
  // hopf fiberation
  bool v2q(const Eigen::Vector3d& v, Eigen::Quaterniond& q)
  {
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
  };

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) 
  {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 1.0;
    triger_received_ = true;
  }

  void ctrl_ready_tri_callback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    ctrl_ready_triger = true;
  }

  void ctrl_start_tri_callback(const quadrotor_msgs::TrajctrlTriggerConstPtr &msg)
  {
    if (!ctrl_start_triger.trigger && msg->trigger)
    {
        ctrl_start_triger = *msg;
        ROS_WARN("Traj_follow: ctrl trigger recive! Traj start stamp reset.");
    }
  }

  void uav_odom_callback(const nav_msgs::OdometryConstPtr& msg)
  {
    uav_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    uav_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
  }

  void target_odom_callback(const nav_msgs::OdometryConstPtr& msg)
  {
    target_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    target_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    target_q.w() = msg->pose.pose.orientation.w;
    target_q.x() = msg->pose.pose.orientation.x;
    target_q.y() = msg->pose.pose.orientation.y;
    target_q.z() = msg->pose.pose.orientation.z;
  }

  void realflight_plan(const bool visualize_sig)
  {
    iniState.setZero(3, 4);
    generate_new_traj_success = false;

    iniState.col(0) = uav_p;
    iniState.col(1) = uav_v;

    land_q = target_q; // 后续加入姿态预测内容

    /* 轨迹生成器traj_opt::TrajOpt::generate_traj
      input：初始状态iniState、目标位置target_p、目标速度target_v、降落点四元数land_q、段数N
      output：轨迹tarj
    */
    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);

    if (generate_new_traj_success) 
    {
      trajStamp = ros::Time::now().toSec();
      ROS_WARN("Traj plan succeed");

      if(visualize_sig)
      {
        visPtr_->visualize_traj(traj, "traj");

        Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
        Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
        visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");

        nav_msgs::Odometry msg;
        msg.header.frame_id = "world";
        double dt = 0.001;
        Eigen::Quaterniond q_last;
        double max_omega = 0;
        for (double t = 0; t <= traj.getTotalDuration(); t += dt) { // get cmd at t
          ros::Duration(dt).sleep();
          // drone
          Eigen::Vector3d p = traj.getPos(t);
          Eigen::Vector3d a = traj.getAcc(t);
          Eigen::Vector3d j = traj.getJer(t);
          Eigen::Vector3d g(0, 0, -9.8);
          Eigen::Vector3d thrust = a - g;

          // std::cout << p.x() << " , " << p.z() << " , ";

          Eigen::Vector3d zb = thrust.normalized();
          {
            // double a = zb.x();
            // double b = zb.y();
            // double c = zb.z();
            Eigen::Vector3d zb_dot = f_DN(thrust) * j; //取推力方向jerk的分量
            double omega12 = zb_dot.norm();
            // if (omega12 > 3.1) {
            //   std::cout << "omega: " << omega12 << "rad/s  t: " << t << std::endl;
            // }
            if (omega12 > max_omega) {
              max_omega = omega12;
            }
            // double a_dot = zb_dot.x();
            // double b_dot = zb_dot.y();
            // double omega3 = (b * a_dot - a * b_dot) / (1 + c);
            // std::cout << "jer: " << j.transpose() << std::endl;
            // std::cout << "omega12: " << zb_dot.norm() << std::endl;
            // std::cout << "omega3: " << omega3 << std::endl;
            // std::cout << thrust.x() << " , " << thrust.z() << " , ";
            // double omega2 = zb_dot.x() - zb.x() * zb_dot.z() / (zb.z() + 1);
            // std::cout << omega2 << std::endl;
            // std::cout << zb_dot.norm() << std::endl;
          }

          Eigen::Quaterniond q;
          bool no_singlarity = v2q(zb, q); //将推力方向向量转换为四元数
          Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt; //旋转矩阵表示的推力角速度
          Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
          // std::cout << "omega_M: \n" << omega_M << std::endl;
          Eigen::Vector3d omega_real;
          omega_real.x() = -omega_M(1, 2);
          omega_real.y() = omega_M(0, 2);
          omega_real.z() = -omega_M(0, 1); //此部分从加速度取得角速度omega_real各分量
          // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
          q_last = q;
          if (no_singlarity) { //推力转四元数成功发布
            msg.pose.pose.position.x = p.x();
            msg.pose.pose.position.y = p.y();
            msg.pose.pose.position.z = p.z();
            msg.pose.pose.orientation.w = q.w();
            msg.pose.pose.orientation.x = q.x();
            msg.pose.pose.orientation.y = q.y();
            msg.pose.pose.orientation.z = q.z();
            msg.header.stamp = ros::Time::now();
            visPtr_->visualize_traj(traj, "traj");
            visPtr_->pub_msg(msg, "odom"); //此处的odom是无人机的odom，默认话题名/drone/planning/odom
          }
          // target
          // Eigen::Vector3d fake_target_v = target_v * (1.0 + 0.5 * sin(1e6 * t));
          // target_p = target_p + fake_target_v * dt;
          // target_v *= 1.0001;
          target_p = target_p + target_v * dt;
          msg.pose.pose.position.x = target_p.x();
          msg.pose.pose.position.y = target_p.y();
          msg.pose.pose.position.z = target_p.z();
          msg.pose.pose.orientation.w = land_q.w();
          msg.pose.pose.orientation.x = land_q.x();
          msg.pose.pose.orientation.y = land_q.y();
          msg.pose.pose.orientation.z = land_q.z();
          msg.header.stamp = ros::Time::now();
          visPtr_->pub_msg(msg, "target_odom");
          if (trajOptPtr_->check_collilsion(p, a, target_p)) {
            std::cout << "collide!  t: " << t << std::endl;
          }
          // TODO replan
          if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
            // ros::Duration(3.0).sleep();

            iniState.col(0) = traj.getPos(t);
            iniState.col(1) = traj.getVel(t);
            iniState.col(2) = traj.getAcc(t);
            iniState.col(3) = traj.getJer(t);
            std::cout << "iniState: \n"
                      << iniState << std::endl;
            trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj, t);
            visPtr_->visualize_traj(traj, "traj");
            t = 0;
            std::cout << "max omega: " << max_omega << std::endl;
          }
        }
        std::cout << "tailV: " << traj.getVel(traj.getTotalDuration()).transpose() << std::endl;
        std::cout << "max thrust: " << traj.getMaxThrust() << std::endl;
        std::cout << "max omega: " << max_omega << std::endl;
      }
    } //若轨迹生成则可视化
    else if(!generate_new_traj_success)
    {
      ROS_ERROR("Traj generate fail!");
    }
  }

  void debug_timer_callback(const ros::TimerEvent& event) {
    if (!triger_received_) {
      return;
    }
    iniState.setZero(3, 4);
    generate_new_traj_success = false;
    // land_q = [1, 0, 0, 0];

    // iniState为无人机初始状态：第一列p、二v、三a、四jerk，target_q为平台姿态四元数
    iniState.setZero();
    iniState.col(0).x() = 0.0;
    iniState.col(0).y() = 0.0;
    iniState.col(0).z() = 2.0;
    iniState.col(1) = perching_v_;
    target_p = perching_p_;
    target_v = perching_v_;
    target_q.x() = 0.0;
    target_q.y() = 0.0;
    target_q.z() = 0.0;
    target_q.w() = 1.0; // target_q表示平台的预设姿态

    Eigen::Vector3d axis = perching_axis_.normalized(); //将perching_axis_向量化为单位向量
    double theta = perching_theta_ * 0.5; // 四元数乘法中除以2以保证旋转角为theta

    /* 定义降落姿态四元数为target_q绕axis旋转theta角 */
    land_q.w() = cos(theta);
    land_q.x() = axis.x() * sin(theta);
    land_q.y() = axis.y() * sin(theta);
    land_q.z() = axis.z() * sin(theta);
    land_q = target_q * land_q;

    std::cout << "iniState: \n"
              << iniState << std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl; //输出转置
    std::cout << "target_v: " << target_v.transpose() << std::endl;
    std::cout << "land_q: "
              << land_q.w() << ","
              << land_q.x() << ","
              << land_q.y() << ","
              << land_q.z() << "," << std::endl;

    /* 轨迹生成器traj_opt::TrajOpt::generate_traj
       input：初始状态iniState、目标位置target_p、目标速度target_v、降落点四元数land_q、段数N
       output：轨迹tarj
    */
    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    if (generate_new_traj_success) {
      visPtr_->visualize_traj(traj, "traj");

      Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
      Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
      visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");

    } //若轨迹生成则可视化
    if (!generate_new_traj_success) {
      triger_received_ = false; //若轨迹未生成则返回触发器为false
      return;
      // assert(false);
    }

    // auto f_DN = [](const Eigen::Vector3d& x) {
    //   double x_norm_2 = x.squaredNorm();
    //   return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
    // }; //返回x的单位法向量
    
    // auto f_D2N = [](const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
    //   double x_norm_2 = x.squaredNorm();
    //   double x_norm_3 = x_norm_2 * x.norm();
    //   Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
    //   return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
    // };
    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    double dt = 0.001;
    Eigen::Quaterniond q_last;
    double max_omega = 0;
    msg.header.frame_id = "world";
    double dt = 0.001;
    Eigen::Quaterniond q_last;
    double max_omega = 0;
    for (double t = 0; t <= traj.getTotalDuration(); t += dt) { // get cmd at t
      ros::Duration(dt).sleep();
      // drone
      Eigen::Vector3d p = traj.getPos(t);
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);
      Eigen::Vector3d g(0, 0, -9.8);
      Eigen::Vector3d thrust = a - g;

      // std::cout << p.x() << " , " << p.z() << " , ";

      Eigen::Vector3d zb = thrust.normalized();
      {
        // double a = zb.x();
        // double b = zb.y();
        // double c = zb.z();
        Eigen::Vector3d zb_dot = f_DN(thrust) * j; //取推力方向jerk的分量
        double omega12 = zb_dot.norm();
        // if (omega12 > 3.1) {
        //   std::cout << "omega: " << omega12 << "rad/s  t: " << t << std::endl;
        // }
        if (omega12 > max_omega) {
          max_omega = omega12;
        }
        // double a_dot = zb_dot.x();
        // double b_dot = zb_dot.y();
        // double omega3 = (b * a_dot - a * b_dot) / (1 + c);
        // std::cout << "jer: " << j.transpose() << std::endl;
        // std::cout << "omega12: " << zb_dot.norm() << std::endl;
        // std::cout << "omega3: " << omega3 << std::endl;
        // std::cout << thrust.x() << " , " << thrust.z() << " , ";
        // double omega2 = zb_dot.x() - zb.x() * zb_dot.z() / (zb.z() + 1);
        // std::cout << omega2 << std::endl;
        // std::cout << zb_dot.norm() << std::endl;
      }

      Eigen::Quaterniond q;
      bool no_singlarity = v2q(zb, q); //将推力方向向量转换为四元数
      Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt; //旋转矩阵表示的推力角速度
      Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
      // std::cout << "omega_M: \n" << omega_M << std::endl;
      Eigen::Vector3d omega_real;
      omega_real.x() = -omega_M(1, 2);
      omega_real.y() = omega_M(0, 2);
      omega_real.z() = -omega_M(0, 1); //此部分从加速度取得角速度omega_real各分量
      // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
      q_last = q;
      if (no_singlarity) { //推力转四元数成功发布
        msg.pose.pose.position.x = p.x();
        msg.pose.pose.position.y = p.y();
        msg.pose.pose.position.z = p.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.header.stamp = ros::Time::now();
        visPtr_->visualize_traj(traj, "traj");
        visPtr_->pub_msg(msg, "odom"); //此处的odom是无人机的odom，默认话题名/drone/planning/odom
      }
      // target
      // Eigen::Vector3d fake_target_v = target_v * (1.0 + 0.5 * sin(1e6 * t));
      // target_p = target_p + fake_target_v * dt;
      // target_v *= 1.0001;
      target_p = target_p + target_v * dt;
      msg.pose.pose.position.x = target_p.x();
      msg.pose.pose.position.y = target_p.y();
      msg.pose.pose.position.z = target_p.z();
      msg.pose.pose.orientation.w = land_q.w();
      msg.pose.pose.orientation.x = land_q.x();
      msg.pose.pose.orientation.y = land_q.y();
      msg.pose.pose.orientation.z = land_q.z();
      msg.header.stamp = ros::Time::now();
      visPtr_->pub_msg(msg, "target_odom");
      if (trajOptPtr_->check_collilsion(p, a, target_p)) {
        std::cout << "collide!  t: " << t << std::endl;
      }
      // TODO replan
      if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
        // ros::Duration(3.0).sleep();

        iniState.col(0) = traj.getPos(t);
        iniState.col(1) = traj.getVel(t);
        iniState.col(2) = traj.getAcc(t);
        iniState.col(3) = traj.getJer(t);
        std::cout << "iniState: \n"
                  << iniState << std::endl;
        trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj, t);
        visPtr_->visualize_traj(traj, "traj");
        t = 0;
        std::cout << "max omega: " << max_omega << std::endl;
      }
    }

    triger_received_ = false;
  }

  void cmd_pub(const ros::TimerEvent& event)
  {

    if(!generate_new_traj_success)
    {
      return;
    }
    nav_msgs::Odometry msg;
    double dt = 0.001;
    Eigen::Quaterniond q_last;
    double max_omega = 0;
    for (double t = 0; t <= traj.getTotalDuration(); t += dt) { // get cmd at t
      ros::Duration(dt).sleep();
      // drone
      Eigen::Vector3d p = traj.getPos(t);
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);
      Eigen::Vector3d g(0, 0, -9.8);
      Eigen::Vector3d thrust = a - g;

      // std::cout << p.x() << " , " << p.z() << " , ";

      Eigen::Vector3d zb = thrust.normalized();
      {
        // double a = zb.x();
        // double b = zb.y();
        // double c = zb.z();
        Eigen::Vector3d zb_dot = f_DN(thrust) * j; //取推力方向jerk的分量
        double omega12 = zb_dot.norm();
        // if (omega12 > 3.1) {
        //   std::cout << "omega: " << omega12 << "rad/s  t: " << t << std::endl;
        // }
        if (omega12 > max_omega) {
          max_omega = omega12;
        }
        // double a_dot = zb_dot.x();
        // double b_dot = zb_dot.y();
        // double omega3 = (b * a_dot - a * b_dot) / (1 + c);
        // std::cout << "jer: " << j.transpose() << std::endl;
        // std::cout << "omega12: " << zb_dot.norm() << std::endl;
        // std::cout << "omega3: " << omega3 << std::endl;
        // std::cout << thrust.x() << " , " << thrust.z() << " , ";
        // double omega2 = zb_dot.x() - zb.x() * zb_dot.z() / (zb.z() + 1);
        // std::cout << omega2 << std::endl;
        // std::cout << zb_dot.norm() << std::endl;
      }

      Eigen::Quaterniond q;
      bool no_singlarity = v2q(zb, q); //将推力方向向量转换为四元数
      Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt; //旋转矩阵表示的推力角速度
      Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
      // std::cout << "omega_M: \n" << omega_M << std::endl;
      Eigen::Vector3d omega_real;
      omega_real.x() = -omega_M(1, 2);
      omega_real.y() = omega_M(0, 2);
      omega_real.z() = -omega_M(0, 1); //此部分从加速度取得角速度omega_real各分量
      // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
      q_last = q;
      if (no_singlarity) { //推力转四元数成功发布
        msg.pose.pose.position.x = p.x();
        msg.pose.pose.position.y = p.y();
        msg.pose.pose.position.z = p.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.header.stamp = ros::Time::now();
        visPtr_->visualize_traj(traj, "traj");
        visPtr_->pub_msg(msg, "odom"); //此处的odom是无人机的odom，默认话题名/drone/planning/odom
      }
      // target
      // Eigen::Vector3d fake_target_v = target_v * (1.0 + 0.5 * sin(1e6 * t));
      // target_p = target_p + fake_target_v * dt;
      // target_v *= 1.0001;
      target_p = target_p + target_v * dt;
      msg.pose.pose.position.x = target_p.x();
      msg.pose.pose.position.y = target_p.y();
      msg.pose.pose.position.z = target_p.z();
      msg.pose.pose.orientation.w = land_q.w();
      msg.pose.pose.orientation.x = land_q.x();
      msg.pose.pose.orientation.y = land_q.y();
      msg.pose.pose.orientation.z = land_q.z();
      msg.header.stamp = ros::Time::now();
      visPtr_->pub_msg(msg, "target_odom");
      if (trajOptPtr_->check_collilsion(p, a, target_p)) {
        std::cout << "collide!  t: " << t << std::endl;
      }
      // TODO replan
      if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
        // ros::Duration(3.0).sleep();

        iniState.col(0) = traj.getPos(t);
        iniState.col(1) = traj.getVel(t);
        iniState.col(2) = traj.getAcc(t);
        iniState.col(3) = traj.getJer(t);
        std::cout << "iniState: \n"
                  << iniState << std::endl;
        trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj, t);
        visPtr_->visualize_traj(traj, "traj");
        t = 0;
        std::cout << "max omega: " << max_omega << std::endl;
      }
    }
  }

  void realflight_set(ros::NodeHandle& nh)
  {
    bool visualize_sig;
    ctrl_start_triger.trigger = false;
    nh.param("visualize_sig", visualize_sig, true);
    
    target_odom_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("target_odom", 10, &triger_callback, this, ros::TransportHints().tcpNoDelay());
    uav_odom_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("uav_odom", 10, &triger_callback, this, ros::TransportHints().tcpNoDelay());
    ctrl_ready_tri_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger", 10, &ctrl_ready_tri_callback, this, ros::TransportHints().tcpNoDelay());
    ctrl_start_tri_sub_ = nh.subscribe<quadrotor_msgs::TrajctrlTrigger>("/traj_follow_start_trigger", 10, &ctrl_start_tri_callback, this, ros::TransportHints().tcpNoDelay());
    
    while( !triger_received_ )
    {
      ROS_WARN("waiting for ctrl triger to start planning");
      ros::Duration(2.0).sleep();
    }
    realflight_plan(visualize_sig);
  }

  void init(ros::NodeHandle& nh) {
    int plan_type;
    
    // set parameters of planning
    nh.getParam("replan", debug_replan_);

    // NOTE once
    nh.param("plan_type", plan_type, 1); // 0 for simulation, 1 for realflight
    nh.getParam("perching_px", perching_p_.x());
    nh.getParam("perching_py", perching_p_.y());
    nh.getParam("perching_pz", perching_p_.z());
    nh.getParam("perching_vx", perching_v_.x());
    nh.getParam("perching_vy", perching_v_.y());
    nh.getParam("perching_vz", perching_v_.z());
    nh.getParam("perching_axis_x", perching_axis_.x());
    nh.getParam("perching_axis_y", perching_axis_.y());
    nh.getParam("perching_axis_z", perching_axis_.z());
    nh.getParam("perching_theta", perching_theta_);

    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);

    if(!plan_type)
    {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &debug_timer_callback, this);
    }
    else
    {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &cmd_pub, this);
      realflight_set(nh);
    }

    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &triger_callback, this, ros::TransportHints().tcpNoDelay());
    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh)); //在单独的线程中运行Nodelet::init()
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);