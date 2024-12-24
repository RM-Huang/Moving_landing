#include "fsm_nodelet.hpp"

namespace ctrl_node{

    void FSM::takeoff_triger_callback(const quadrotor_msgs::TakeoffLandConstPtr& msgPtr){
        if(!takeoff_triger_received){
            takeoff_triger_received = true;
            ROS_INFO("\033[32m[FSM]:Takeoff triger received!\033[32m");
        }else{
            ROS_ERROR("[FSM]:Takeoff triger duplicated!");
        }
    }
    
    void FSM::init(ros::NodeHandle& nh){
        param.config_from_ros_handle(nh);
        vel_controller.init(param);
        pos_controller.init(param);
        att_ang_controller.init(param);

        takeoff_triger_sub_ = nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land", 1, &FSM::takeoff_triger_callback, this, 
                                                                        ros::TransportHints().tcpNoDelay());

        state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&State_Data_t::feed, &state_data, _1));

        extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10,
                                                                        boost::bind(&ExtendedState_Data_t::feed, &extended_state_data, _1));

        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 100,
                                                    boost::bind(&Odom_Data_t::feed, &odom_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().tcpNoDelay());

        imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                                100,
                                                boost::bind(&Imu_Data_t::feed, &imu_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

        cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("cmd", 100,
                                                                boost::bind(&Command_Data_t::feed, &cmd_data, _1),
                                                                ros::VoidConstPtr(),
                                                                ros::TransportHints().tcpNoDelay());

        ctrl_pv_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ctrl_aw_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
        traj_start_triger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
        debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

        arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        int trials = 0;
        //判断是否连接上PX4
        while (!state_data.current_state.connected){
            ros::Duration(1.0).sleep();
            if (trials++ > 5)
                ROS_ERROR("Unable to connnect to PX4!!!");
        }

        current_state = MANUAL;
        fsm_timer_ = nh.createTimer(ros::Duration(1.0 / param.fsmparam.frequncy), &FSM::fsm_timer, this);

        ROS_INFO("\033[32m[FSM]:Init completed, change to MANUAL state!\033[32m");
    }
    
    void FSM::onInit(void){
        ros::NodeHandle nh(getMTPrivateNodeHandle());
        initThread_ = std::thread(std::bind(&FSM::init, this, nh));
    }

    bool FSM::odom_is_received(const ros::Time &now_time){
        return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
    }

    bool FSM::cmd_is_received(const ros::Time &now_time){
        return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ctrl_node::FSM, nodelet::Nodelet);