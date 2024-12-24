#include "fsm_nodelet.hpp"

namespace ctrl_node{

    void FSM::fsm_timer(const ros::TimerEvent& event){
        
        ros::Time now_time = ros::Time::now();
        if(!odom_is_received(now_time)){
            ROS_ERROR("[FSM]:No odom! Do Not switch to OFFBOARD mode now!");
            takeoff_triger_received = false;
            current_state = MANUAL;
            ros::Duration(1.0).sleep();
            return;
        }

        // if(state_data.current_state.mode != "OFFBOARD"){ // 若遥控器没切到OFFBOARD模式则退出
        //     takeoff_triger_received = false;
        //     current_state = MANUAL;
        //     return;
        // }

        if(state_data.previous_state.mode != state_data.current_state.mode){ // 若遥控器没切到OFFBOARD模式则退出
            if(state_data.current_state.mode == "OFFBOARD"){
                ROS_INFO("\033[32m[FSM]:Switch to OFFBOARD mode!\033[32m");
            }else if(state_data.previous_state.mode == "OFFBOARD"){
                ROS_INFO("\033[32m[FSM]:Exit OFFBOARD mode! Switch to %s state.\033[32m", state_data.current_state.mode.c_str());
                current_state = MANUAL;
            }
            state_data.previous_state = state_data.current_state;
        }

        Controller::Desired_State_t des(odom_data);
        Controller::Controller_Output_t u;

        switch (current_state){
            case MANUAL:
                if(state_data.current_state.mode == "OFFBOARD"){
                    if(takeoff_triger_received){
                        // Auto_Takeoff conditions check
                        if(state_data.current_state.armed){
                            ROS_ERROR("[FSM]:Reject Auto_Takeoff, vehicle is already armed!");
                        // }else if(odom_data.v.norm() > 0.1){
                        //     ROS_ERROR("[FSM]:Reject Auto_Takeoff, Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
                        }else{
                            if(!toggle_arm_disarm(true)){ //arming rejected
                                ROS_ERROR("[FSM]:takeoff arming rejected by PX4!");
                            }else{
                                current_state = TAKEOFF;
                                att_ang_controller.resetThrustMapping();
                                set_start_pose_for_takeoff(odom_data);
                                ROS_INFO("\033[32m[FSM] MANUAL --> TAKEOFF(L1)\033[32m");
                            }
                        }
                    }else if(state_data.current_state.armed){ // already in flight
                        current_state = HOVER;
                        att_ang_controller.resetThrustMapping();
                        set_hov_with_odom();
                        ROS_INFO("\033[32m[FSM] MANUAL(L1) --> HOVER(L2)\033[32m");
                    }
                }
                break;
            
            case TAKEOFF:
                if(state_data.current_state.mode != "OFFBOARD"){
                    current_state = MANUAL;
                    ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, TAKEOFF --> MANUAL(L2)\033[32m");
                }else if((now_time - takeoff_state.toggle_takeoff_time).toSec() < AutoTakeoff_t::MOTORS_SPEEDUP_TIME){
                    des = get_pv_speed_up_des(now_time);
                }else if(odom_data.p(2) >= (takeoff_state.start_pose(2) + param.takeoff_state.height-0.1)){ // reach desired height
                    set_hov_with_odom();
                    takeoff_state.delay_trigger.first = true;
                    takeoff_state.delay_trigger.second = now_time + ros::Duration(AutoTakeoff_t::DELAY_TRIGGER_TIME);

                    current_state = HOVER;

                    ROS_INFO("\033[32m[FSM] TAKEOFF --> HOVER(L2)\033[32m");
                }else{
                    // if(takeoff_state.first_time){
                    //     set_start_pose_for_takeoff(odom_data);
                    //     takeoff_state.first_time = false;
                    // }
                    des = get_takeoff_des(odom_data);
                }
                break;

            case HOVER:
                if(state_data.current_state.mode != "OFFBOARD"){
                    current_state = MANUAL;
                    ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, HOVER --> MANUAL(L2)\033[32m");
                }else if(cmd_is_received(now_time) && state_data.current_state.armed){
                    current_state = MISSION;
                    des = get_cmd_des();
                    ROS_INFO("\033[32m[FSM] HOVER --> MISSION(L2)\033[32m");
                }else if(!state_data.current_state.armed){
                    current_state = MANUAL;
                    ROS_INFO("\033[32m[FSM] HOVER --> MANUAL(L2)\033[32m");
                }else{
                    des = get_hover_des();

                    if(takeoff_state.delay_trigger.first && now_time > takeoff_state.delay_trigger.second){
                        takeoff_state.delay_trigger.first = false;
                        publish_trigger(odom_data.msg);
                        ROS_INFO("\033[32m[FSM]:TRIGGER sent, allow user command.\033[32m");
                    }  
                }
                break;

            case MISSION:
                if(state_data.current_state.mode != "OFFBOARD"){
                    current_state = MANUAL;
                    ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, MISSION --> MANUAL(L2)\033[32m");
                }else if (!cmd_is_received(now_time)){
                    current_state = HOVER;
                    set_hov_with_odom();
                    des = get_hover_des();
                    ROS_INFO("[FSM]:From MISSION(L3) to HOVER(L2)!");
                }
                else{
                    des = get_cmd_des();
                }
                break;
        }

        if (current_state == HOVER || current_state == MISSION){
            // controller.estimateThrustModel(imu_data.a, bat_data.volt, param);
            att_ang_controller.estimateThrustModel(imu_data.a,param);

        }

        switch (param.controller_type)
        {
        case 0:
            //TODO position controller
            debug_msg = pos_controller.calculateControl(des, odom_data, u);
            publish_position_ctrl(u, now_time);
            break;

        case 1:
            debug_msg = vel_controller.calculateControl(des, odom_data, u);
            publish_velocity_ctrl(u, now_time);
            break;
        
        default:
            if(current_state == MANUAL){
                debug_msg = att_ang_controller.calculateControl(des, odom_data, imu_data, u);
            }else{
                debug_msg = att_ang_controller.calculateControlCMD(des, odom_data, imu_data, u, now_time);
            }

            if(param.controller_type == 2){
                publish_attitude_ctrl(u, now_time);
            }else if(param.controller_type == 3){
                publish_bodyrate_ctrl(u, now_time);
            }
            break;
        }

        debug_msg.header.stamp = now_time;
		debug_pub.publish(debug_msg);

        takeoff_triger_received = false;
    }

    void FSM::publish_trigger(const nav_msgs::Odometry &odom_msg){
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "world";
        msg.pose = odom_msg.pose.pose;

        traj_start_triger_pub.publish(msg);
    }

    void FSM::publish_position_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp){
        mavros_msgs::PositionTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        msg.position.x = u.position.x();
        msg.position.y = u.position.y();
        msg.position.z = u.position.z();
        msg.yaw = u.yaw;

        ctrl_pv_pub.publish(msg);
    }

    void FSM::publish_velocity_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp){
        mavros_msgs::PositionTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        msg.velocity.x = u.velocity.x();
        msg.velocity.y = u.velocity.y();
        msg.velocity.z = u.velocity.z();
        msg.yaw = u.yaw;

        ctrl_pv_pub.publish(msg);
    }

    void FSM::publish_bodyrate_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp){
        mavros_msgs::AttitudeTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

        msg.body_rate.x = u.bodyrates.x();
        msg.body_rate.y = u.bodyrates.y();
        msg.body_rate.z = u.bodyrates.z();

        msg.thrust = u.thrust;

        ctrl_aw_pub.publish(msg);
    }

    void FSM::publish_attitude_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp){
        mavros_msgs::AttitudeTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

        msg.orientation.x = u.q.x();
        msg.orientation.y = u.q.y();
        msg.orientation.z = u.q.z();
        msg.orientation.w = u.q.w();

        msg.thrust = u.thrust;

        ctrl_aw_pub.publish(msg);
    }

    bool FSM::toggle_arm_disarm(bool arm){
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;
        //申请解锁不成功
        if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
        {
            if (arm)
                ROS_ERROR("ARM rejected by PX4!");
            else
                ROS_ERROR("DISARM rejected by PX4!");

            return false;
        }

        return true;
    }

    void FSM::set_hov_with_odom(){
        hover_pose.head<3>() = odom_data.p;
        hover_pose(3) = Controller::q2yaw(odom_data.q); // get yaw
        std::cout<<"hover_pose = "<< odom_data.p.reverse()<<std::endl;

        last_set_hover_pose_time = ros::Time::now();
    }

    void FSM::set_start_pose_for_takeoff(const Odom_Data_t &odom){
        takeoff_state.start_pose.head<3>() = odom.p;
        takeoff_state.start_pose(3) = Controller::q2yaw(odom.q); //get yaw

        takeoff_state.toggle_takeoff_time = ros::Time::now();
    }

    Controller::Desired_State_t FSM::get_pv_speed_up_des(const ros::Time& now){
        double delta_t = (now - takeoff_state.toggle_takeoff_time).toSec();
        double des_a_z = exp((delta_t - AutoTakeoff_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
        if (des_a_z > 0.1)
        {
            ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
            des_a_z = 0.0;
        }
        // double des_a_z = 0.05;

        Controller::Desired_State_t des;
        des.p = takeoff_state.start_pose.head<3>();
        des.v = Eigen::Vector3d(0, 0, 0);
        des.a = Eigen::Vector3d(0, 0, des_a_z);
        des.j = Eigen::Vector3d::Zero();
        des.yaw = takeoff_state.start_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_takeoff_des(const Odom_Data_t &odom){
        ros::Time now = ros::Time::now();
        double delta_t = (now - takeoff_state.toggle_takeoff_time).toSec() - (param.takeoff_state.speed > 0 ? AutoTakeoff_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff

        Controller::Desired_State_t des;
        des.p = takeoff_state.start_pose.head<3>() + Eigen::Vector3d(0, 0, param.takeoff_state.speed * delta_t);
        des.v = Eigen::Vector3d(0, 0, param.takeoff_state.speed);
        des.a = Eigen::Vector3d::Zero();
        des.j = Eigen::Vector3d::Zero();
        des.yaw = takeoff_state.start_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_hover_des(){
        Controller::Desired_State_t des;
        des.p = hover_pose.head<3>();
        des.v = Eigen::Vector3d::Zero();
        des.a = Eigen::Vector3d::Zero();
        des.j = Eigen::Vector3d::Zero();
        des.yaw = hover_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_cmd_des(){
        Controller::Desired_State_t des;
        des.p = cmd_data.p;
        des.v = cmd_data.v;
        des.a = cmd_data.a;
        des.j = cmd_data.j;
        des.omg = cmd_data.omg;
        des.yaw = cmd_data.yaw;
        des.yaw_rate = cmd_data.yaw_rate;

        return des;
    }
}