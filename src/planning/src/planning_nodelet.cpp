#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TrajcurDesire.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/MotorlockTriger.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>
#include "target_prediction/bezier_predict.h"

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
  // ros::Publisher land_pub_;
  // ros::Publisher hover_pub_;
  ros::ServiceClient FCU_command_srv;

  ros::Timer plan_timer_;
  ros::Timer cmd_timer_;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // NOTE planning or fake target
  // bool target_ = false;
  // Eigen::Vector3d goal_;

  // Using for prediction
  // int predict_seg;
  bool predict_success;
  Bezierpredict tgpredict;
  std::vector<Eigen::Vector4d> target_detect_list;

  // Using for planning timer
  Eigen::MatrixXd iniState;
  int plan_type; // 0 for sim, 1 for real
  bool generate_new_traj_success = false;
  bool visualize_sig;
  bool target_odom_recrived = false;
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
    ROS_WARN("[planning]:ctrl triger accept!");
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
    if(msg->pose.pose.position.x < 10 && msg->pose.pose.position.y < 5 && msg->pose.pose.position.z < 3)
      target_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    // target_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    // target_q.w() = msg->pose.pose.orientation.w;
    // target_q.x() = msg->pose.pose.orientation.x;
    // target_q.y() = msg->pose.pose.orientation.y;
    // target_q.z() = msg->pose.pose.orientation.z;
    if(!target_odom_recrived)
    {
      target_odom_recrived = true;
    }
  }

  void planning_fsm(const ros::TimerEvent& event)
  {
    /* ________________________________ Program entry condition _________________________________ */
    if(!ctrl_ready_triger || !triger_received_)
    {
      // planning may failed to start if px4ctrl shutdown unexpectly
      ros::Duration(1.0).sleep();
      return;
    }
    ROS_WARN("[planning]:plan triger received!");
    if(plan_type == 1 && !target_odom_recrived)
    {
      ROS_ERROR("[planning]:target odom haven't received!");
      ros::Duration(1.0).sleep();
      return;
    }
    ROS_WARN("[planning]:start planning!");

    iniState.setZero(3, 4);
    target_q.x() = 0.0;
    target_q.y() = 0.0;
    target_q.z() = 0.0;
    target_q.w() = 1.0; // target_q表示平台的预设姿态
    land_q = target_q; 
    predict_success = false;

    //TODO plaform predict
    /* ______________________________________ Prediction _________________________________________ */
    bool prediction_flag = false; // for static landing test, always false
    if(prediction_flag)
    {
      vector<Eigen::Matrix<double, 6, 1>> predict_state_list;
      int bezier_flag = tgpredict.TrackingGeneration(5,5,target_detect_list);
      // if(bezier_flag==0){
      //     predict_state_list = tgpredict.getStateListFromBezier(_PREDICT_SEG);//最终用的预测轨迹数据存放在此 包含位置速度
      //     Sample_list = tgpredict.SamplePoslist_bezier(_PREDICT_SEG);
      // }
      // else{
      //     ROS_WARN("bezier predict error");
      // }
      if(bezier_flag != 0)
      {
        ROS_WARN("[planning]:platform predict error");
      }
      else
        predict_success = true; 
      // else
      // {
      //   vector<Eigen::Matrix<double, 6, 1>> predict_state_list = tgpredict.getStateListFromBezier(_PREDICT_SEG);
      // }
      // if(predict_state_list.size() < 1) 
      // {
      //     ROS_ERROR("[planning]:Bezier predict failed");
      // }
      // else
      //   predict_success = true; 
    }
    // else
    // {
    //   // visualize_pre(Sample_list);
    //   // int flag_pp = 0;
    //   // Eigen::Vector3d begin_point = predict_state_list[0].head(3);
    //   // flag_pp = kinosearch.search(start_pt,start_vel,predict_state_list,_TIME_INTERVAL); 

    //   // for test
    //   // uav_p << 0.0, 0.0, 1.5;
    //   // uav_v << 0.0,0.0,0.0; 
    //   target_q.x() = 0.0;
    //   target_q.y() = 0.0;
    //   target_q.z() = 0.0;
    //   target_q.w() = 1.0; // target_q表示平台的预设姿态
    //   // Eigen::Vector3d axis = perching_axis_.normalized(); //将perching_axis_向量化为单位向量
    //   // double theta = perching_theta_ * 0.5; // 四元数乘法中除以2以保证旋转角为theta
    //   // /* 定义降落姿态四元数为target_q绕axis旋转theta角 */
    //   // land_q.w() = cos(theta);
    //   // land_q.x() = axis.x() * sin(theta);
    //   // land_q.y() = axis.y() * sin(theta);
    //   // land_q.z() = axis.z() * sin(theta);
    //   // land_q = target_q * land_q;
    //   land_q = target_q; 
    // }
    
    // TODO replan require
    /* _____________________________________ Replan condition check ____________________________________ */
    bool replan_from_hover = true; // for test, to be completed: 1.covariance difference too large between two platform predictions, 2.hover state
    if(replan_from_hover)
    {
      // planning from hover state
      generate_new_traj_success = false;
      
      iniState.col(0) = uav_p;
      iniState.col(1) = uav_v;
    }
    else
    {
      // planning from last traj
      double delta_util_next = ros::Time::now().toSec() + 0.001 - trajStamp;
      iniState.col(0) = traj.getPos(delta_util_next);
      iniState.col(1) = traj.getVel(delta_util_next);
      iniState.col(2) = traj.getAcc(delta_util_next);
      iniState.col(3) = traj.getJer(delta_util_next);
    }
    

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
    // if(predict_success == false) // follow state condition
    // {
    //   generate_new_traj = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    // }
    // else // land state condition
    // {
      generate_new_traj = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    // }

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

  bool force_arm_disarm(bool arm)
  {
    // https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
    mavros_msgs::CommandLong force_arm_disarm_srv;
    force_arm_disarm_srv.request.broadcast = false;
    force_arm_disarm_srv.request.command = 400; // MAV_CMD_COMPONENT_ARM_DISARM
    force_arm_disarm_srv.request.param1 = arm;
    force_arm_disarm_srv.request.param2 = 21196.0;	  // force
    force_arm_disarm_srv.request.confirmation = true;

    if (!(FCU_command_srv.call(force_arm_disarm_srv) && force_arm_disarm_srv.response.success))
    {
      if (arm)
        ROS_ERROR("ARM rejected by PX4!");
      else
        ROS_ERROR("DISARM rejected by PX4!");

      return false;
    }
    return true;
  }

  void cmd_pub(const ros::TimerEvent& event)
  {
    if(generate_new_traj_success && ctrl_ready_triger)
    {
      publishing_cmd = true;
      ros::Time current_time = ros::Time::now();
      double delta_from_start = current_time.toSec() - trajStamp;
      // quadrotor_msgs::PositionCommandPtr cmdMsg(new quadrotor_msgs::PositionCommand());
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
      else if(delta_from_start >= traj.getTotalDuration() && abs(uav_p[2] - target_p[2]) < 0.01 + robot_l_)
      {
        force_arm_disarm(false);

        ROS_WARN("[planning]: land triger published");

        generate_new_traj_success = false;
        triger_received_ = false;
      }
      publishing_cmd = false;
    }
    else
    {
      //TODO replan cmd set
      return;// 悬停
    }
  }

  void init(ros::NodeHandle& nh) {
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
    nh.param("visualize_sig", visualize_sig, true); // true for visualize
    nh.getParam("VehicleMass", vehicleMass);
    nh.getParam("GravAcc", gravAcc);
    nh.getParam("HorizDrag", horizDrag);
    nh.getParam("VertDrag", vertDrag);
    nh.getParam("ParasDrag", parasDrag);
    nh.getParam("SpeedEps", speedEps);
    nh.getParam("robot_l", robot_l_);
    // nh.param("Predic_seg", predict_seg, 30); // platform observations used to genetrate prediction curves
    target_p = perching_p_;// set initial state
    target_v = perching_v_;

    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh); // debug
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);

    target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("target_odom", 10, &Nodelet::target_odom_callback, this, ros::TransportHints().tcpNoDelay());
    uav_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("uav_odom", 10, &Nodelet::uav_odom_callback, this, ros::TransportHints().tcpNoDelay());
    ctrl_ready_tri_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("ctrl_triger", 10, &Nodelet::ctrl_ready_tri_callback, this, ros::TransportHints().tcpNoDelay()); // debug
    // ctrl_start_tri_sub_ = nh.subscribe<quadrotor_msgs::TrajctrlTrigger>("/traj_follow_start_trigger", 10, &ctrl_start_tri_callback, this, ros::TransportHints().tcpNoDelay());
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    
    if(ifanalyse)
    {
      des_pub_ = nh.advertise<quadrotor_msgs::TrajcurDesire>("/desire_pose_current_traj", 10); // debug
    }
    cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 10);
    // hover_pub_ = nh.advertise<quadrotor_msgs::MotorlockTriger>("/locktriger", 1);
    // land_pub_ = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    FCU_command_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::planning_fsm, this);
    cmd_timer_ = nh.createTimer(ros::Duration(1.0 / (plan_hz_ * 10)), &Nodelet::cmd_pub, this);
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