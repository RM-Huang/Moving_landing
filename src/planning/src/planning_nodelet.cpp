#include "planning_nodelet.hpp"

namespace planning {

  //--------------------- func ---------------------------

  void Nodelet::triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) 
  {
    triger_received_ = true; // for static platfrom landing
    ROS_INFO("\033[32m[planning]:plan triger received!\033[32m");
  }

  void Nodelet::ctrl_ready_tri_callback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    ctrl_ready_triger = true;
    ROS_INFO("\033[32m[planning]:ctrl triger accept!\033[32m");
  }

  void Nodelet::uav_odom_callback(const nav_msgs::OdometryConstPtr& msg)
  {
    uav_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    uav_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    uav_q.w() = msg->pose.pose.orientation.w;
    uav_q.x() = msg->pose.pose.orientation.x;
    uav_q.y() = msg->pose.pose.orientation.y;
    uav_q.z() = msg->pose.pose.orientation.z;
  }

  void Nodelet::vision_statu_callback(const std_msgs::Float64ConstPtr& msg)
  {
    vision_stamp = msg->data;
  }

  void Nodelet::target_odom_callback(const nav_msgs::OdometryConstPtr& msg)
  {
    target_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    ekf_error << msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[2];
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

  bool Nodelet::v2q(const Eigen::Vector3d& v, Eigen::Quaterniond& q){
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

  void Nodelet::planning_fsm(const ros::TimerEvent& event) // for moving platform
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

    /* ______________________________________ Prediction _________________________________________ */
    bool prediction_flag = false; // for static landing test, always false
    if(prediction_flag)
    {
      //TODO:prediction
    }

    /* ________________________________ plan entry condition _________________________________ */
    if(!ctrl_ready_triger || !triger_received_)
    {
      // planning may failed to start if px4ctrl shutdown unexpectly
      ros::Duration(1.0).sleep();
      return;
    }
    
    /* ________________________________________ FSM ________________________________________________ */
    // double delta_from_last = ros::Time::now().toSec() - trajStamp;
    double cur_time = ros::Time::now().toSec();
    double delta_from_last = cur_time - trajStamp;
    
    switch(plan_state)
    {
      case traj_opt::TrajOpt::HOVER:
      {
        if(generate_new_traj_success)
        {
          plan_state = traj_opt::TrajOpt::FOLLOW;
          // generate_new_traj_success = false;
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
        if((static_landing || predict_success) && (vision_stamp == 1)) // 0.3 = platform_r_
        {
          // std::cout<<"dist_ = "<<sqrt(pow(uav_p[0] - target_p[0], 2) + pow(uav_p[1] - target_p[1], 2))<<std::endl;
          // if((sqrt(pow(uav_p[0] - target_p[0], 2) + pow(uav_p[1] - target_p[1], 2)) < 1.0) && (abs(uav_v[0] - target_v[0]) < 0.5) && (abs(uav_v[1] - target_v[1]) < 0.5))
          if(ekf_error[0] <= 0.1 && ekf_error[1] <= 0.1 && ekf_error[2] <= 0.1 && abs(uav_v[0] - target_v[0]) < 0.5 && abs(uav_v[1] - target_v[1]) < 0.5)
          {
            land_first = false;
            // generate_new_traj_success = false;
            plan_state = traj_opt::TrajOpt::LAND;
            ROS_INFO("\033[32m[planning]:Change to LAND state!\033[32m");
            // ros::Duration(0.2).sleep();
            return;
            // }
          }
        }

        if(delta_from_last > traj.getTotalDuration())
        {
          plan_state = traj_opt::TrajOpt::HOVER;
          generate_new_traj_success = false;
          ROS_INFO("\033[32m[planning]:Change to HOVER state!\033[32m");
          return;
        }

        // delta_from_last = ros::Time::now().toSec() - trajStamp; // get a future state as replan initial state
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
          // Eigen::Vector3d delta_p = target_p + target_v * (T - delta_from_last) - traj.getPos(T);
          // if(plan_type == 1 && ( (cur_time - target_odom_time > 0.1) || !vision_stamp ) ) // if target msg dosen't refresh
          if(plan_type == 1 && ( (cur_time - target_odom_time > 0.1)) ) // debug
          {
            // generate_new_traj_success = false;
            plan_state = traj_opt::TrajOpt::FOLLOW;
            ROS_INFO("\033[32m[planning]:Change to FOLLOW state!\033[32m");
            return;
          }
          else if(delta_from_last > T)
          {
            plan_state = traj_opt::TrajOpt::HOVER;
            // generate_new_traj_success = false;
            ROS_INFO("\033[32m[planning]:Change to HOVER state!\033[32m");
            return;
          }
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
    std::cout<<"inital_state = " << std::endl;
    std::cout << iniState.transpose() <<std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl;
    std::cout << "target_v: " << target_v.transpose() << std::endl;
    std::cout << "target_q: "
              << target_q.w() << ","
              << target_q.x() << ","
              << target_q.y() << ","
              << target_q.z() << std::endl;
    std::cout << "delta_from_last: " << delta_from_last << std::endl;
    ROS_INFO("\033[32m[planning]:start planning!\033[32m");

    bool generate_new_traj; 
    // double stamp_tmp = ros::Time::now().toSec();
    // Eigen::Vector3d target_p_tmp = target_p;
    // Eigen::Vector3d target_v_tmp = target_v;
    
    generate_new_traj = trajOptPtr_->generate_traj(iniState, target_p, target_v, target_q, uav_q_, 
                                                   predict_success, 10, traj, &plan_state, delta_from_last); 

    if (generate_new_traj) 
    {
      trajStamp = cur_time;
      // trajStamp = stamp_tmp;
      // target_v_last = target_p_tmp;
      // target_p_last = target_v_tmp;
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

  bool Nodelet::force_arm_disarm(bool arm)
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

  void Nodelet::debug_pub(const double& delta_from_start){
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d jer;

    pos = traj.getPos(delta_from_start);
    vel = traj.getVel(delta_from_start);
    acc = traj.getAcc(delta_from_start);
    jer = traj.getJer(delta_from_start);
    Eigen::Vector3d g(0, 0, -9.8);
    Eigen::Vector3d thrust = acc - g;

    Eigen::Vector3d zb = thrust.normalized();

    Eigen::Quaterniond q;
    double t_cur = ros::Time::now().toSec();
    bool no_singlarity = v2q(zb, q);
    Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / (t_cur - t_last);
    Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
    // std::cout << "omega_M: \n" << omega_M << std::endl;
    Eigen::Vector3d omega_real;
    omega_real.x() = -omega_M(1, 2);
    omega_real.y() = omega_M(0, 2);
    omega_real.z() = -omega_M(0, 1);
    q_last = q;
    t_last = t_cur;

    int colli = 0;
    if (trajOptPtr_->check_collilsion(pos, acc, target_p)) {
      colli = 1;
    }

    double vis = trajOptPtr_->check_visible(pos, acc, target_p);

    quadrotor_msgs::OutputData debug_msg;
    debug_msg.p.x = pos(0);
    debug_msg.p.y = pos(1);
    debug_msg.p.z = pos(2);
    debug_msg.v.x = vel(0);
    debug_msg.v.y = vel(1);
    debug_msg.v.z = vel(2);
    debug_msg.thrust = thrust.norm();
    debug_msg.omega = omega_real.norm();
    debug_msg.colli = colli;
    debug_msg.vis_ang = vis;
    debug_msg.header.stamp = ros::Time::now();
    debug_pub_.publish(debug_msg);
  }

  void Nodelet::cmd_pub(const ros::TimerEvent& event)
  {
    if(ctrl_ready_triger && triger_received_)
    {
      // abs(uav_v[0] - target_v[0]) < land_r_ && abs(uav_v[1] - target_v[1]) < land_r_ && 
      if(abs(uav_p[0] - target_p[0]) < land_r_ && abs(uav_p[1] - target_p[1]) < land_r_ && 
        uav_p[2] - target_p[2] <= robot_l_ + 0.05) // set horizental restrictions if odom msg highly reliable
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
          Eigen::Vector3d pos;
          Eigen::Vector3d vel;
          Eigen::Vector3d acc;
          Eigen::Vector3d jer;

          pos = traj.getPos(delta_from_start);
          vel = traj.getVel(delta_from_start);
          acc = traj.getAcc(delta_from_start);
          jer = traj.getJer(delta_from_start);
          
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

          debug_pub(delta_from_start);

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

  void Nodelet::init(ros::NodeHandle& nh) {
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

    target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("target_odom", 1, &Nodelet::target_odom_callback, this, ros::TransportHints().tcpNoDelay());
    vision_statu_sub_ = nh.subscribe<std_msgs::Float64>("/land_state_triger", 1, &Nodelet::vision_statu_callback, this, ros::TransportHints().tcpNoDelay());
    uav_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("uav_odom", 1, &Nodelet::uav_odom_callback, this, ros::TransportHints().tcpNoDelay());
    ctrl_ready_tri_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("ctrl_triger", 1, &Nodelet::ctrl_ready_tri_callback, this, ros::TransportHints().tcpNoDelay()); // debug
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 1, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    
    if(ifanalyse)
    {
      des_pub_ = nh.advertise<quadrotor_msgs::TrajcurDesire>("/desire_pose_current_traj", 1); // debug
    }
    cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 1);
    debug_pub_ = nh.advertise<quadrotor_msgs::OutputData>("planner_debug", 1);
    FCU_command_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::planning_fsm, this);
    cmd_timer_ = nh.createTimer(ros::Duration(0.01), &Nodelet::cmd_pub, this);
    ROS_WARN("Planning node initialized!");
  }

  void Nodelet::onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh)); //在单独的线程中运行Nodelet::init()
  }

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);