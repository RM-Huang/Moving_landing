#ifndef __FSM_HPP
#define __FSM_HPP

#include <thread>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include "input.hpp"
#include "controller.hpp"
#include "param.hpp"

namespace ctrl_node{

    struct AutoTakeoff_t
    {
        ros::Time toggle_takeoff_time;
        Eigen::Vector4d start_pose;
        bool first_time = false;
        std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};

        static constexpr double MOTORS_SPEEDUP_TIME = 1.0; // motors idle running for 3 seconds before takeoff
        static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
    };
    
    class FSM : public nodelet::Nodelet{
        private:
            enum CurrentState{
                MANUAL = 0,
                TAKEOFF = 1,
                HOVER,
                MISSION
            };

            Eigen::Vector4d hover_pose;
            ros::Time last_set_hover_pose_time;

            bool takeoff_triger_received = false;

            CurrentState current_state;
            AutoTakeoff_t takeoff_state;
            quadrotor_msgs::Px4ctrlDebug debug_msg;

            std::thread initThread_;
            
            ros::Timer fsm_timer_;

            ros::Subscriber takeoff_triger_sub_;
            ros::Subscriber state_sub;
            ros::Subscriber extended_state_sub;
            ros::Subscriber odom_sub;
            ros::Subscriber imu_sub;
            ros::Subscriber cmd_sub;

            ros::Publisher ctrl_pv_pub; // position, velocity ctrl cmd
            ros::Publisher ctrl_aw_pub; // attitude, angle velocity ctrl cmd
            ros::Publisher traj_start_triger_pub;
            ros::Publisher debug_pub; //debug

            ros::ServiceClient arming_client_srv;

            void init(ros::NodeHandle& nh);

            void fsm_timer(const ros::TimerEvent& event);

            void takeoff_triger_callback(const quadrotor_msgs::TakeoffLandConstPtr& msgPtr);

            bool toggle_arm_disarm(bool arm);

            void set_hov_with_odom();

            void set_start_pose_for_takeoff(const Odom_Data_t &odom);

            Controller::Desired_State_t get_pv_speed_up_des(const ros::Time& now);

            Controller::Desired_State_t get_takeoff_des(const Odom_Data_t &odom);

            Controller::Desired_State_t get_hover_des();

            Controller::Desired_State_t get_cmd_des();

            void publish_trigger(const nav_msgs::Odometry &odom_msg);

            void publish_position_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp);

            void publish_velocity_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp);

            void publish_bodyrate_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp);

            void publish_attitude_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp);

            bool odom_is_received(const ros::Time &now_time);

            bool cmd_is_received(const ros::Time &now_time);

        public:

            State_Data_t state_data;
            ExtendedState_Data_t extended_state_data;
            Odom_Data_t odom_data;
            Imu_Data_t imu_data;
            Command_Data_t cmd_data;
            Parameter_t param;
            Controller::Velocity_Control vel_controller;
            Controller::Position_Control pos_controller;
            Controller::Attitude_Angular_Control att_ang_controller;

            void onInit(void);
    };
}

#endif