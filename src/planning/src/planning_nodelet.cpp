#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TrajcurDesire.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <atomic>
#include <thread>
#include <vis_utils/vis_utils.hpp>
#include <traj_opt/flatness.hpp>

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
  // ros::Subscriber ctrl_start_tri_sub_;

  ros::Publisher cmd_pub_;
  ros::Publisher des_pub_;
  ros::Publisher land_pub_;

  ros::Timer plan_timer_;
  ros::Timer cmd_timer_;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // NOTE planning or fake target
  // bool target_ = false;
  // Eigen::Vector3d goal_;

  // Using for planning timer
  Eigen::MatrixXd iniState;
  bool generate_new_traj_success = false;
  bool visualize_sig;
  Trajectory traj;
  Eigen::Vector3d target_p, target_v, uav_p, uav_v;
  Eigen::Quaterniond target_q;
  Eigen::Quaterniond land_q;

  // Using for flatness
  double vehicleMass;
  double gravAcc;
  double horizDrag;
  double vertDrag;
  double parasDrag;
  double speedEps;
  double robot_l_;

  // NOTE just for debug
  // bool debug_ = false;
  // bool once_ = false;
  bool debug_replan_ = false;
  bool ifanalyse =false;

  // double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_; // for simulation
  double perching_theta_;

  Trajectory traj_poly_;
  double trajStamp;
  double trigerStamp = 0; // time stamp for current plan start triger from other program
  // int traj_id_ = 0;
  // bool wait_hover_ = true;
  // bool force_hover_ = true;

  int plan_hz_;

  bool ctrl_ready_triger = false;
  bool publishing_cmd = false;
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);

  //--------------------- func ---------------------------
  static Eigen::MatrixXd f_DN(const Eigen::Vector3d& x) 
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
    // goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 1.0;
    // if(msgPtr->header.stamp.toSec() - trigerStamp > 0)
    // {
    //   triger_received_ = true;
    //   trigerStamp = msgPtr->header.stamp.toSec();
    // }
    triger_received_ = true; // for static platfrom landing
    // std::cout<<"triger_received_ = "<<triger_received_<<std::endl;
  }

  void ctrl_ready_tri_callback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    ctrl_ready_triger = true;
    ROS_WARN("[planning]:accept ctrl triger!");
  }

  // void ctrl_start_tri_callback(const quadrotor_msgs::TrajctrlTriggerConstPtr &msg)
  // {
  //   if (!ctrl_start_triger.trigger && msg->trigger)
  //   {
  //       ctrl_start_triger = *msg;
  //       ROS_WARN("Traj_follow: ctrl trigger recive! Traj start stamp reset.");
  //   }
  // }

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

  void realflight_plan(const ros::TimerEvent& event)
  {
    if(!ctrl_ready_triger || !triger_received_ || publishing_cmd)
    {
      ros::Duration(1.0).sleep();
      return;
    }
    ROS_WARN("[planning]:plan triger received! start planning.");
    // if(!triger_received_ || publishing_cmd) // for testing, change to above sentence while finish
    // {
    //   ros::Duration(1.0).sleep();
    //   return;
    // }
    // TODO replan require
    generate_new_traj_success = false;
    
    iniState.setZero(3, 4);

    /* for test */
    // uav_p << 0.0, 0.0, 1.5;
    // uav_v << 0.0,0.0,0.0; 
    target_p = perching_p_;
    target_v = perching_v_;
    target_q.x() = 0.0;
    target_q.y() = 0.0;
    target_q.z() = 0.0;
    target_q.w() = 1.0; // target_q表示平台的预设姿态
    // Eigen::Vector3d axis = perching_axis_.normalized(); //将perching_axis_向量化为单位向量
    // double theta = perching_theta_ * 0.5; // 四元数乘法中除以2以保证旋转角为theta
    // /* 定义降落姿态四元数为target_q绕axis旋转theta角 */
    // land_q.w() = cos(theta);
    // land_q.x() = axis.x() * sin(theta);
    // land_q.y() = axis.y() * sin(theta);
    // land_q.z() = axis.z() * sin(theta);
    // land_q = target_q * land_q;


    iniState.col(0) = uav_p;
    iniState.col(1) = uav_v;
    land_q = target_q; // 后续加入姿态预测内容

    std::cout<<"uav_p = "<<uav_p.transpose()<<" uav_v = "<<uav_v.transpose()<<std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl;
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
    bool generate_new_traj; 
    generate_new_traj = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);

    if (generate_new_traj) 
    {
      trajStamp = ros::Time::now().toSec();
      generate_new_traj_success = true;
      ROS_WARN("[planning]:Traj generate succeed");
      std::cout<<"traj_duration = "<<traj.getTotalDuration()<<std::endl;
    }
    else if(!generate_new_traj)
    {
      generate_new_traj_success = false;
      ROS_ERROR("[planning]:Traj generate fail!");
    }
    triger_received_ = false;
  }

  void debug_timer_callback(const ros::TimerEvent& event) {
    if (!triger_received_) {
      return;
    }
    iniState.setZero(3, 4);
    generate_new_traj_success = false;
    // land_q = [1, 0, 0, 0];

    // iniState为无人机初始状态：p、v、a、jerk，target_q为平台姿态四元数
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
        visPtr_->pub_msg(msg, "odom"); //此处的odom是无人机的odom，默认话题名/drone0/planning/odom
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
      visPtr_->pub_msg(msg, "target");
      if (trajOptPtr_->check_collilsion(p, a, target_p)) {
        std::cout << "collide!  t: " << t << std::endl;
      }
      // TODO debug pulse
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
      std::cout << "tailV: " << traj.getVel(traj.getTotalDuration()).transpose() << std::endl;
      std::cout << "max thrust: " << traj.getMaxThrust() << std::endl;
      std::cout << "max omega: " << max_omega << std::endl;
    }

    triger_received_ = false;
  }

  void cmd_pub(const ros::TimerEvent& event)
  {
    if(generate_new_traj_success && ctrl_ready_triger)
    {
      publishing_cmd = true;
      ros::Time current_time = ros::Time::now();
      double delta_from_start = current_time.toSec() - trajStamp;
      if (delta_from_start > 0.0 && delta_from_start < traj.getTotalDuration())
      {
        Eigen::VectorXd physicalParams(6); //物理参数
        physicalParams(0) = vehicleMass;//质量
        physicalParams(1) = gravAcc;//重力加速度
        physicalParams(2) = horizDrag;//水平阻力系数
        physicalParams(3) = vertDrag;//垂直阻力系数
        physicalParams(4) = parasDrag;//附加阻力系数
        physicalParams(5) = speedEps;//速度平滑因子

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                        physicalParams(3), physicalParams(4), physicalParams(5));//将物理参数赋值给微分平坦的私有变量

        double thr;//总推力
        Eigen::Vector4d quat;//四元数
        Eigen::Vector3d omg;//角速率
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector3d acc;
        Eigen::Vector3d jer;

        pos = traj.getPos(delta_from_start);
        vel = traj.getVel(delta_from_start);
        acc = traj.getAcc(delta_from_start);
        jer = traj.getJer(delta_from_start);

        flatmap.forward(vel,
                        acc,
                        jer,
                        0.0, 0.0,
                        thr, quat, omg); //利用微分平坦特性计算出总推力，姿态四元数，机体角速率
        
        quadrotor_msgs::PositionCommandPtr cmdMsg(new quadrotor_msgs::PositionCommand());
        cmdMsg->position.x = pos(0);
        cmdMsg->position.y = pos(1);
        cmdMsg->position.z = pos(2);
        cmdMsg->velocity.x = vel(0);
        cmdMsg->velocity.y = vel(1);
        cmdMsg->velocity.z = vel(2);
        cmdMsg->acceleration.x = acc(0);
        cmdMsg->acceleration.y = acc(1);
        cmdMsg->acceleration.z = acc(2);
        cmdMsg->jerk.x = jer(0);
        cmdMsg->jerk.y = jer(1);
        cmdMsg->jerk.z = jer(2);
        cmdMsg->yaw = atan2(2.0*(quat(1)*quat(2) + quat(0)*quat(3)), 1.0 - 2.0 * (quat(2) * quat(2) + quat(3) * quat(3))); // quat=[w,x,y,z]
        cmdMsg->yaw_dot = omg[2];

        cmd_pub_.publish(cmdMsg);

        if(ifanalyse)
        {
          /* for traj analyse */
          quadrotor_msgs::TrajcurDesirePtr desMsg(new quadrotor_msgs::TrajcurDesire());
          desMsg->header.stamp = current_time;
          desMsg->pos.orientation.w = quat(0);
          desMsg->pos.orientation.x = quat(1);
          desMsg->pos.orientation.y = quat(2);
          desMsg->pos.orientation.z = quat(3);
          desMsg->pos.position.x = pos(0);
          desMsg->pos.position.y = pos(1);
          desMsg->pos.position.z = pos(2);
          
          des_pub_.publish(desMsg);
        }

        if(visualize_sig)
        {
          visPtr_->visualize_traj(traj, "traj");
          Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
          Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
          visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");

          nav_msgs::Odometry msg;
          msg.header.frame_id = "world";
          
          msg.pose.pose.position.x = pos.x();
          msg.pose.pose.position.y = pos.y();
          msg.pose.pose.position.z = pos.z();
          msg.pose.pose.orientation.w = quat(0);
          msg.pose.pose.orientation.x = quat(1);
          msg.pose.pose.orientation.y = quat(2);
          msg.pose.pose.orientation.z = quat(3);
          msg.header.stamp = ros::Time::now();
          visPtr_->visualize_traj(traj, "traj");
          visPtr_->pub_msg(msg, "odom"); //此处的odom是无人机的odom，默认话题名/drone/planning/odom

          msg.pose.pose.position.x = target_p.x();
          msg.pose.pose.position.y = target_p.y();
          msg.pose.pose.position.z = target_p.z();
          msg.pose.pose.orientation.w = land_q.w();
          msg.pose.pose.orientation.x = land_q.x();
          msg.pose.pose.orientation.y = land_q.y();
          msg.pose.pose.orientation.z = land_q.z();
          msg.header.stamp = ros::Time::now();
          visPtr_->pub_msg(msg, "target");

          if (trajOptPtr_->check_collilsion(pos, acc, target_p)) {
          std::cout << "collide!  t: " << delta_from_start << std::endl;
          }
        }
      }
      else if(delta_from_start > traj.getTotalDuration() && uav_p[2] - target_p[2] - robot_l_ < 0.03)
      {
        quadrotor_msgs::TakeoffLand landMsg;
        landMsg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
        land_pub_.publish(landMsg); // using ctrl autoland for now, consider to swich to rcin_remap lock

        generate_new_traj_success = false;
      }
      publishing_cmd = false;
    }
    // else if(generate_new_traj_success && visualize_sig) // visualize traj
    // {
    //   publishing_cmd = true;
    //   visPtr_->visualize_traj(traj, "traj");

    //   Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
    //   Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
    //   visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");

    //   nav_msgs::Odometry msg;
    //   msg.header.frame_id = "world";
    //   double dt = 0.001;
    //   Eigen::Quaterniond q_last;
    //   double max_omega = 0;
    //   for (double t = 0; t <= traj.getTotalDuration(); t += dt) { // get cmd at t
    //     ros::Duration(dt).sleep();
    //     // drone
    //     Eigen::Vector3d p = traj.getPos(t);
    //     Eigen::Vector3d a = traj.getAcc(t);
    //     Eigen::Vector3d j = traj.getJer(t);
    //     Eigen::Vector3d g(0, 0, -9.8);
    //     Eigen::Vector3d thrust = a - g;

    //     Eigen::Vector3d zb = thrust.normalized();

    //     Eigen::Vector3d zb_dot = f_DN(thrust) * j; //取推力方向jerk的分量
    //     double omega12 = zb_dot.norm();

    //     if (omega12 > max_omega) {
    //       max_omega = omega12;
    //     }

    //     Eigen::Quaterniond q;
    //     bool no_singlarity = v2q(zb, q); //将推力方向向量转换为四元数
    //     Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt; //旋转矩阵表示的推力角速度
    //     Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
    //     // std::cout << "omega_M: \n" << omega_M << std::endl;
    //     Eigen::Vector3d omega_real;
    //     omega_real.x() = -omega_M(1, 2);
    //     omega_real.y() = omega_M(0, 2);
    //     omega_real.z() = -omega_M(0, 1); //此部分从加速度取得角速度omega_real各分量
    //     // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
    //     q_last = q;
    //     if (no_singlarity) { //推力转四元数成功发布
    //       msg.pose.pose.position.x = p.x();
    //       msg.pose.pose.position.y = p.y();
    //       msg.pose.pose.position.z = p.z();
    //       msg.pose.pose.orientation.w = q.w();
    //       msg.pose.pose.orientation.x = q.x();
    //       msg.pose.pose.orientation.y = q.y();
    //       msg.pose.pose.orientation.z = q.z();
    //       msg.header.stamp = ros::Time::now();
    //       visPtr_->visualize_traj(traj, "traj");
    //       visPtr_->pub_msg(msg, "odom"); //此处的odom是无人机的odom，默认话题名/drone/planning/odom
    //     }

    //     target_p = target_p + target_v * dt;
    //     msg.pose.pose.position.x = target_p.x();
    //     msg.pose.pose.position.y = target_p.y();
    //     msg.pose.pose.position.z = target_p.z();
    //     msg.pose.pose.orientation.w = land_q.w();
    //     msg.pose.pose.orientation.x = land_q.x();
    //     msg.pose.pose.orientation.y = land_q.y();
    //     msg.pose.pose.orientation.z = land_q.z();
    //     msg.header.stamp = ros::Time::now();
    //     visPtr_->pub_msg(msg, "target");
    //     if (trajOptPtr_->check_collilsion(p, a, target_p)) {
    //       std::cout << "collide!  t: " << t << std::endl;
    //     }
    //   }
    //   std::cout << "tailV: " << traj.getVel(traj.getTotalDuration()).transpose() << std::endl;
    //   std::cout << "max thrust: " << traj.getMaxThrust() << std::endl;
    //   std::cout << "max omega: " << max_omega << std::endl;
    //   publishing_cmd = false;
    //   generate_new_traj_success = false;
    // }
    else
    {
      //TODO replan cmd set
      return;// 悬停
    }
  }

  void realflight_set(ros::NodeHandle& nh)
  {
    nh.param("visualize_sig", visualize_sig, true); // true for visualize
    nh.getParam("VehicleMass", vehicleMass);
    nh.getParam("GravAcc", gravAcc);
    nh.getParam("HorizDrag", horizDrag);
    nh.getParam("VertDrag", vertDrag);
    nh.getParam("ParasDrag", parasDrag);
    nh.getParam("SpeedEps", speedEps);
    nh.getParam("robot_l", robot_l_);
    
    target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("target_odom", 10, &Nodelet::target_odom_callback, this, ros::TransportHints().tcpNoDelay());
    uav_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("uav_odom", 10, &Nodelet::uav_odom_callback, this, ros::TransportHints().tcpNoDelay());
    ctrl_ready_tri_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("ctrl_triger", 10, &Nodelet::ctrl_ready_tri_callback, this, ros::TransportHints().tcpNoDelay());
    // ctrl_start_tri_sub_ = nh.subscribe<quadrotor_msgs::TrajctrlTrigger>("/traj_follow_start_trigger", 10, &ctrl_start_tri_callback, this, ros::TransportHints().tcpNoDelay());

    if(ifanalyse)
      des_pub_ = nh.advertise<quadrotor_msgs::TrajcurDesire>("/desire_pose_current_traj", 10);
  
    cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 10);
    land_pub_ = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    
    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::realflight_plan, this);
  }

  void init(ros::NodeHandle& nh) {
    int plan_type;
    
    // set parameters of planning
    nh.getParam("replan", debug_replan_);

    // NOTE once
    nh.param("plan_type", plan_type, 1); // 0 for simulation, 1 for realflight
    nh.param("ifanalyse", ifanalyse, false);
    nh.param("plan_hz", plan_hz_, 10);
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
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::debug_timer_callback, this);
    }
    else
    {
      cmd_timer_ = nh.createTimer(ros::Duration(1.0 / (plan_hz_ * 10)), &Nodelet::cmd_pub, this);
      realflight_set(nh);
    }

    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
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