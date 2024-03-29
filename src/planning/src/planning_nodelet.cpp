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
#include <std_msgs/Float64.h>
#include <traj_opt/traj_opt.h>
// #include "target_prediction/bezier_predict.h"

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
  ros::Subscriber vision_statu_sub_;
  ros::Subscriber uav_odom_sub_;
  ros::Subscriber ctrl_ready_tri_sub_;

  ros::Publisher cmd_pub_;
  ros::Publisher des_pub_;
  // ros::Publisher land_pub_;
  // ros::Publisher hover_pub_;
  ros::ServiceClient FCU_command_srv;

  int plan_hz_;

  ros::Timer plan_timer_;
  ros::Timer cmd_timer_;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // Using for prediction
  // int predict_seg;
  double sample_dur;
  double predict_dur;
  // Bezierpredict tgpredict;
  std::vector<Eigen::Vector4d> target_detect_list;
  bool predict_success = false;
  // std::vector<Eigen::MatrixXd> bezier_polyc_list;
  // std::vector<double> bezierT_list;
  // std::vector<double> bezier_init_time_list;

  // Using for planning timer
  Eigen::MatrixXd iniState;
  int plan_type; // 0 for sim, 1 for real
  double target_odom_time = 0;
  bool generate_new_traj_success = false;
  bool visualize_sig;
  bool target_odom_recrived = false;
  bool land_first = false;
  Trajectory traj;
  Eigen::Vector3d target_p, target_v, uav_p, uav_v;
  Eigen::Vector3d target_p_last, target_v_last;
  double vision_stamp = 0;
  double trajStamp_observe;
  Eigen::Quaterniond target_q, uav_q;
  traj_opt::TrajOpt::plan_s plan_state = traj_opt::TrajOpt::HOVER;

  Eigen::Vector3d follow_p;
  Eigen::Vector3d follow_v;

  // static param
  double vehicleMass;
  double gravAcc;
  double horizDrag;
  double vertDrag;
  double parasDrag;
  double speedEps;
  double robot_l_;
  double land_r_;
  double omega_yaw_max_;

  // NOTE just for debug
  bool debug_replan_ = false;
  bool ifanalyse =false;

  // double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_; // for simulation
  double perching_theta_;

  Trajectory traj_poly_;
  double trajStamp;
  double trigerStamp = 0; // time stamp for current plan start triger from other program

  bool ctrl_ready_triger = false;
  bool publishing_cmd = false;
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);

  //--------------------- func ---------------------------

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) 
  {
    triger_received_ = true; // for static platfrom landing
    ROS_INFO("\033[32m[planning]:plan triger received!\033[32m");
  }

  void ctrl_ready_tri_callback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    ctrl_ready_triger = true;
    ROS_INFO("\033[32m[planning]:ctrl triger accept!\033[32m");
  }

  void uav_odom_callback(const nav_msgs::OdometryConstPtr& msg)
  {
    uav_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    uav_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    uav_q.w() = msg->pose.pose.orientation.w;
    uav_q.x() = msg->pose.pose.orientation.x;
    uav_q.y() = msg->pose.pose.orientation.y;
    uav_q.z() = msg->pose.pose.orientation.z;
  }

  void vision_statu_callback(const std_msgs::Float64ConstPtr& msg)
  {
    vision_stamp = msg->data;
  }

  void target_odom_callback(const nav_msgs::OdometryConstPtr& msg)
  {
    // if(msg->pose.pose.position.x < 10 && msg->pose.pose.position.y < 5 && msg->pose.pose.position.z < 3)
    // {
    //   target_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    //   target_odom_time = msg->header.stamp.toSec();
    // }
    target_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    target_odom_time = msg->header.stamp.toSec();
    target_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    target_q.w() = msg->pose.pose.orientation.w;
    target_q.x() = msg->pose.pose.orientation.x;
    target_q.y() = msg->pose.pose.orientation.y;
    target_q.z() = msg->pose.pose.orientation.z;
    if(!target_odom_recrived)
    {
      target_odom_recrived = true;
    }
  }

  void planning_fsm(const ros::TimerEvent& event)
  {
    /* ________________________________ Predict entry condition _________________________________ */
    if(plan_type == 1 && !target_odom_recrived)
    {
      ROS_ERROR("[planning]:target odom haven't received!");
      ros::Duration(1.0).sleep();
      return;
    }

    iniState.setZero(3, 4);
    // target_q.x() = 0.0;
    // target_q.y() = 0.0;
    // target_q.z() = 0.0;
    // target_q.w() = 1.0; // target_q表示平台的预设姿态
    Eigen::Quaterniond uav_q_ = uav_q;
    bool static_landing = true; // test

    //TODO plaform predict
    /* ______________________________________ Prediction _________________________________________ */
    bool prediction_flag = false; // for static landing test, always false
    if(prediction_flag)
    {
      // double detect_time = target_odom_time;
      // if(abs(detect_time - ros::Time::now().toSec()) < 0.1)
      // {
      //   target_detect_list.push_back(Eigen::Vector4d(target_p[0], target_p[1], target_p[2], detect_time));
      //   if(target_detect_list.size() >= sample_dur * plan_hz_)
      //   {
      //     int bezier_flag = tgpredict.TrackingGeneration(4,1.5,target_detect_list);
      //     if(bezier_flag != 0)
      //     {
      //       ROS_WARN("[planning]:platform predict error");
      //       // using velocity stable assumption while bezier failed
      //       // static_landing = true;
      //     }
      //     else
      //     {
      //       // bezier_polyc_list.push_back(tgpredict.getPolyCoeff());
      //       // bezierT_list.push_back(tgpredict.getPolyTime()(0));
      //       // bezier_init_time_list.push_back(detect_time);

      //       // if(bezier_init_time_list.size() > predict_dur * plan_hz_)
      //       // {
      //       //     bezier_polyc_list.erase(bezier_polyc_list.begin());
      //       //     bezier_init_time_list.erase(bezier_init_time_list.begin());
      //       //     bezierT_list.erase(bezierT_list.begin());
      //       // }
      //       predict_success = true; 
      //     }
      //     target_detect_list.erase(target_detect_list.begin());
      //   }
      // }
      // else
      // {
      //   predict_success = false;
      //   std::cout<<"detect_time = "<<detect_time<<" now_time = "<<ros::Time::now().toSec()<<std::endl;
      //   ROS_ERROR("[planning]:predict error, failed to align target odom, target odom delay:%f", abs(detect_time - ros::Time::now().toSec()));
      //   return; // for test
      // }
    }

    /* ________________________________ plan entry condition _________________________________ */
    if(!ctrl_ready_triger || !triger_received_)
    {
      // planning may failed to start if px4ctrl shutdown unexpectly
      ros::Duration(1.0).sleep();
      return;
    }
    
    /* ________________________________________ FSM ________________________________________________ */
    double delta_from_last = ros::Time::now().toSec() - trajStamp;
    // if(sqrt(pow(uav_p[0] - target_p[0], 2) + pow(uav_p[1] - target_p[1], 2)) < abs(uav_p[2] - target_p[2]) * std::tan(M_PI / 4));
    //   vision_stamp = ros::Time::now().toSec(); //test
    switch(plan_state)
    {
      case traj_opt::TrajOpt::HOVER:
      {
        if(generate_new_traj_success)
        {
          plan_state = traj_opt::TrajOpt::FOLLOW;
          ROS_INFO("\033[32m[planning]:Change to FOLLOW state!\033[32m");
          return;
        }
        delta_from_last = -1.0;
        // planning from hover state     
        iniState.col(0) = uav_p;
        iniState.col(1) = uav_v;
        break;
      }
      case traj_opt::TrajOpt::FOLLOW:
      {
        // double T = traj.getTotalDuration();
        if((static_landing || predict_success) && (ros::Time::now().toSec() - vision_stamp < 0.1)) // 0.3 = platform_r_
        {
          if(sqrt(pow(uav_p[0] - target_p[0], 2) + pow(uav_p[1] - target_p[1], 2)) < 0.8)
          {
            // Eigen::Vector3d acc = traj.getAcc(delta_from_last);
            // Eigen::Vector3d jer = traj.getJer(delta_from_last);
            // if(acc[0] < 0.1 && acc[1] < 0.1 && jer.norm() < 0.1)
            // {
            ros::Duration(0.1).sleep();
            land_first = false;
            plan_state = traj_opt::TrajOpt::LAND;
            ROS_INFO("\033[32m[planning]:Change to LAND state!\033[32m");
            // ros::Duration(0.2).sleep();
            return;
            // }
          }
        }
        else if(delta_from_last > traj.getTotalDuration())
        {
          plan_state = traj_opt::TrajOpt::HOVER;
          generate_new_traj_success = false;
          ROS_INFO("\033[32m[planning]:Change to HOVER state!\033[32m");
          return;
        }
        else if(generate_new_traj_success && delta_from_last < 0.2) // replan from last traj after 0.2s
        {
          return;
        }
        // else if(predict_success)

        delta_from_last = ros::Time::now().toSec() - trajStamp; // get a future state as replan initial state
        iniState.col(0) = traj.getPos(delta_from_last);
        iniState.col(1) = traj.getVel(delta_from_last);
        iniState.col(2) = traj.getAcc(delta_from_last);
        iniState.col(3) = traj.getJer(delta_from_last);
        break;
      }
      case traj_opt::TrajOpt::LAND:
      {
        if(!land_first)
        {
          double T = traj.getTotalDuration();
          Eigen::Vector3d delta_p = target_p + target_v * (T - delta_from_last) - traj.getPos(T);
          if(plan_type == 1 && ( (ros::Time::now().toSec() - target_odom_time > 0.1) || (ros::Time::now().toSec() - vision_stamp > 0.1) ) ) // if target msg dosen't refresh
          {
            plan_state = traj_opt::TrajOpt::FOLLOW;
            ROS_INFO("\033[32m[planning]:Change to FOLLOW state!\033[32m");
            return;
          }
          // else if( delta_from_last < 0.2 )
          // {
          //   return;
          // }
          // else if( T < 0.6 )
          // {
          //   ROS_INFO("[planning]: close to platform!");
          //   ros::Duration(T).sleep();
          //   return;
          // }
          else if(delta_from_last > T)
          {
            plan_state = traj_opt::TrajOpt::HOVER;
            generate_new_traj_success = false;
            ROS_INFO("\033[32m[planning]:Change to HOVER state!\033[32m");
            return;
          }

          delta_from_last = ros::Time::now().toSec() - trajStamp; // get a future state as replan initial state
          // // else if(delta_from_last < 0.2 && ( (target_p_last + target_v_last * delta_from_last) - target_p).norm() < 0.15)
          // else if(abs(delta_p[0]) < land_r_ && abs(delta_p[1] < land_r_) ) 
          // {
          //   ROS_INFO("\033[32m[planning]:Predict effected!\033[32m");
          //   return;
          // }
          // else if( abs(target_p[2] - uav_p[2]) < 1.0 && delta_from_last < 0.2 )
          // {
          //   return;
          // }
        }
        else if(land_first)
        {
          delta_from_last = -1.0;
          land_first = false;
        }

        // delta_from_last = ros::Time::now().toSec() - trajStamp; // get a future state as replan initial state
        iniState.col(0) = traj.getPos(delta_from_last);
        iniState.col(1) = traj.getVel(delta_from_last);
        iniState.col(2) = traj.getAcc(delta_from_last);
        iniState.col(3) = traj.getJer(delta_from_last);

        break;
      }
    }
    std::cout<<"planning state: "<<plan_state<<std::endl;
    std::cout<<"uav_p = "<<uav_p.transpose()<<" uav_v = "<<uav_v.transpose()<<std::endl;
    std::cout<<"inital_p = "<<iniState.col(0).transpose()<<" inital_v = "<<iniState.col(1).transpose()<<std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl;
    std::cout << "target_v: " << target_v.transpose() << std::endl;
    ROS_INFO("\033[32m[planning]:start planning!\033[32m");

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

    /* 轨迹生成器traj_opt::TrajOpt::generate_traj
      input：初始状态iniState、目标位置target_p、目标速度target_v、降落点四元数land_q、段数N
      output：轨迹tarj
    */
    bool generate_new_traj; 
    double stamp_tmp = ros::Time::now().toSec();
    Eigen::Vector3d target_p_tmp = target_p;
    Eigen::Vector3d target_v_tmp = target_v;
    
    generate_new_traj = trajOptPtr_->generate_traj(iniState, target_p, target_v, target_q, uav_q_, 
                                                   predict_success, 10, traj, &plan_state, delta_from_last); 

    if (generate_new_traj) 
    {
      trajStamp = stamp_tmp;
      target_v_last = target_p_tmp;
      target_p_last = target_v_tmp;
      generate_new_traj_success = true;
      ROS_INFO("\033[32m[planning]:Traj generate succeed\033[32m");
      std::cout<<"traj_duration = "<<traj.getTotalDuration()<<std::endl;
    }
    else if(!generate_new_traj)
    {
      generate_new_traj_success = false;
      ROS_ERROR("[planning]:Traj generate fail!");
    }
    // triger_received_ = false;
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
        ROS_INFO("\033[32m ARM rejected by PX4!\033[32m");
      else
        ROS_ERROR("DISARM rejected by PX4!");

      return false;
    }
    return true;
  }

  void cmd_pub(const ros::TimerEvent& event)
  {
    if(ctrl_ready_triger && triger_received_)
    {
      // abs(uav_v[0] - target_v[0]) < land_r_ && abs(uav_v[1] - target_v[1]) < land_r_ && 
      if(uav_p[2] - target_p[2] <= robot_l_) // set horizental restrictions if odom msg highly reliable
      {
        generate_new_traj_success = false;
        triger_received_ = false;
        ctrl_ready_triger = false;

        std::cout<<"uav_p = "<<uav_p.transpose()<<" car_p = "<<target_p.transpose()<<" differ = "<< abs(uav_p[0] - target_p[0]) <<" "<< abs(uav_p[1] - target_p[1])<<std::endl;

        force_arm_disarm(false);
        ROS_INFO("\033[32m [planning]: land triger published \033[32m");
      }
      // publishing_cmd = false;

      if(generate_new_traj_success)
      {
        publishing_cmd = true;
        ros::Time current_time = ros::Time::now();
        double delta_from_start = current_time.toSec() - trajStamp;
        // quadrotor_msgs::PositionCommandPtr cmdMsg(new quadrotor_msgs::PositionCommand());
        if (delta_from_start > 0.0 && delta_from_start <= traj.getTotalDuration())
        {
          // Eigen::VectorXd physicalParams(6); //物理参数
          // physicalParams(0) = vehicleMass;//质量
          // physicalParams(1) = gravAcc;//重力加速度
          // physicalParams(2) = horizDrag;//水平阻力系数
          // physicalParams(3) = vertDrag;//垂直阻力系数
          // physicalParams(4) = parasDrag;//附加阻力系数
          // physicalParams(5) = speedEps;//速度平滑因子

          // flatness::FlatnessMap flatmap;
          // flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
          //                 physicalParams(3), physicalParams(4), physicalParams(5));//将物理参数赋值给微分平坦的私有变量

          // double thr;//总推力
          // Eigen::Vector4d quat;//四元数
          // Eigen::Vector3d omg;//角速率
          Eigen::Vector3d pos;
          Eigen::Vector3d vel;
          Eigen::Vector3d acc;
          Eigen::Vector3d jer;

          pos = traj.getPos(delta_from_start);
          vel = traj.getVel(delta_from_start);
          acc = traj.getAcc(delta_from_start);
          jer = traj.getJer(delta_from_start);

          // if(abs(uav_p[2] - target_p[2]) <= robot_l_ + 0.05)
          // if(abs(pos[2] - target_p[2]) <= robot_l_ + 0.05)
          // {
          //   generate_new_traj_success = false;
          //   triger_received_ = false;
          //   ctrl_ready_triger = false;

          //   std::cout<<"uav_p = "<<uav_p.transpose()<<" car_p = "<<target_p.transpose()<<" differ = "<< abs(uav_p[0] - target_p[0]) <<" "<< abs(uav_p[1] - target_p[1])<<std::endl;

          //   force_arm_disarm(false);
          //   ROS_INFO("\033[32m [planning]: land triger published \033[32m");
          // }

          // flatmap.forward(vel,
          //                 acc,
          //                 jer,
          //                 0.0, 0.0,
          //                 thr, quat, omg); //利用微分平坦特性计算出总推力，姿态四元数，机体角速率
          
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

          double yaw_des = atan2(2.0*(target_q.x()*target_q.y() + target_q.w()*target_q.z()), 1.0 - 2.0 * (target_q.y() * target_q.y() + target_q.z() * target_q.z())); // quat=[w,x,y,z]
          double yaw_cur = atan2(2.0*(uav_q.x()*uav_q.y() + uav_q.w()*uav_q.z()), 1.0 - 2.0 * (uav_q.y() * uav_q.y() + uav_q.z() * uav_q.z()));
          // double yaw_delta = acos(uav_q.dot(target_q));
          // double yaw_delta = yaw_des - yaw_cur;
          // if(yaw_delta > M_PI)
          // {
          //   yaw_delta = yaw_delta -  2 * M_PI;
          // }
          // else if(yaw_delta < - M_PI)
          // {
          //   yaw_delta = 2 * 6.283185307 + yaw_delta;
          // }

          if(yaw_des > 0)
          {
            cmdMsg->yaw = std::min(yaw_cur + omega_yaw_max_ , yaw_des);
          }
          else
          {
            cmdMsg->yaw = std::max(yaw_cur - omega_yaw_max_ , yaw_des);
          }
          // cmdMsg->yaw = atan2(2.0*(quat(1)*quat(2) + quat(0)*quat(3)), 1.0 - 2.0 * (quat(2) * quat(2) + quat(3) * quat(3))); // quat=[w,x,y,z]
          // cmdMsg->yaw_dot = omg[2];

          cmd_pub_.publish(cmdMsg);

          if(ifanalyse)
          {
            /* for traj analyse */
            quadrotor_msgs::TrajcurDesirePtr desMsg(new quadrotor_msgs::TrajcurDesire());
            desMsg->header.stamp = current_time;
            // desMsg->pos.orientation.w = quat(0);
            // desMsg->pos.orientation.x = quat(1);
            // desMsg->pos.orientation.y = quat(2);
            // desMsg->pos.orientation.z = quat(3);
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

            // if (trajOptPtr_->check_collilsion(pos, acc, target_p)) {
            // std::cout << "collide!  t: " << delta_from_start << std::endl;
            // }
          }
        }
        // else if(delta_from_start >= traj.getTotalDuration())
        // {
        // }
      }
    }

    if(visualize_sig)
    {
      nav_msgs::Odometry msg;
      msg.header.frame_id = "world";
      
      msg.pose.pose.position.x = uav_p.x();
      msg.pose.pose.position.y = uav_p.y();
      msg.pose.pose.position.z = uav_p.z();
      msg.pose.pose.orientation.w = uav_q.w();
      msg.pose.pose.orientation.x = uav_q.x();
      msg.pose.pose.orientation.y = uav_q.y();
      msg.pose.pose.orientation.z = uav_q.z();
      msg.header.stamp = ros::Time::now();
      visPtr_->visualize_traj(traj, "traj");
      visPtr_->pub_msg(msg, "odom"); //此处的odom是无人机的odom，默认话题名/drone/planning/odom

      if(target_odom_recrived)
      {
        msg.pose.pose.position.x = target_p.x();
        msg.pose.pose.position.y = target_p.y();
        msg.pose.pose.position.z = target_p.z();
        msg.pose.pose.orientation.w = target_q.w();
        msg.pose.pose.orientation.x = target_q.x();
        msg.pose.pose.orientation.y = target_q.y();
        msg.pose.pose.orientation.z = target_q.z();
        msg.header.stamp = ros::Time::now();
        visPtr_->pub_msg(msg, "target");
      }

      // if(predict_success)
      // {
      //   std::vector<Eigen::Vector3d> path;
      //   auto duration = tgpredict.getPolyTime()(0);
      //   for (double t = 0; t < duration; t += 0.01) {
      //     path.push_back(tgpredict.getPosFromBezier(t,0));
      //   }
      //   visPtr_->visualize_path(path, "target_bezier");
      // } 
    }
  }

  void init(ros::NodeHandle& nh) {
    double platform_r_;
    double robot_r_;
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
    nh.getParam("omega_yaw_max", omega_yaw_max_);
    nh.param("visualize_sig", visualize_sig, true); // true for visualize
    nh.getParam("VehicleMass", vehicleMass);
    nh.getParam("GravAcc", gravAcc);
    nh.getParam("HorizDrag", horizDrag);
    nh.getParam("VertDrag", vertDrag);
    nh.getParam("ParasDrag", parasDrag);
    nh.getParam("SpeedEps", speedEps);
    nh.getParam("robot_l", robot_l_);
    nh.getParam("platform_r", platform_r_);
    nh.getParam("robot_r", robot_r_);
    nh.param("bezier_sample_dur", sample_dur, 3.0);
    nh.param("bezier_predict_dur", predict_dur, 3.0);

    // nh.param("Predic_seg", predict_seg, 30); // platform observations used to genetrate prediction curves
    target_p = perching_p_;// set initial state
    target_v = perching_v_;

    land_r_ = platform_r_ - robot_r_;

    // tgpredict.init(sample_dur * plan_hz_,predict_dur * plan_hz_);

    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh); // debug
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);

    target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("target_odom", 10, &Nodelet::target_odom_callback, this, ros::TransportHints().tcpNoDelay());
    vision_statu_sub_ = nh.subscribe<std_msgs::Float64>("/vision_received", 1, &Nodelet::vision_statu_callback, this, ros::TransportHints().tcpNoDelay());
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
    // cmd_timer_ = nh.createTimer(ros::Duration(1.0 / (plan_hz_ * 10)), &Nodelet::cmd_pub, this);
    cmd_timer_ = nh.createTimer(ros::Duration(0.01), &Nodelet::cmd_pub, this);
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