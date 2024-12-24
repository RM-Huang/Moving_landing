#ifndef __READPARAM_HPP
#define __READPARAM_HPP

#include <ros/ros.h>
#include <Eigen/Dense>

namespace ctrl_node{

    class Parameter_t{
        private:

        template <typename TName, typename TVal>
        void read_param(const ros::NodeHandle &nh, const TName &name, TVal &val);

        public:
        struct Pos_Control_Pid{
            double kp0, kp1, kp2;
            double ki0, ki1, ki2;
            double kd0, kd1, kd2;
        };

        struct Vel_Control_Pid{
            double Kvp0, Kvp1, Kvp2;
            double Kvi0, Kvi1, Kvi2;
            double Kvd0, Kvd1, Kvd2;
        };
        
        struct Att_Control_Pid{
            double Kp0, Kp1, Kp2;
            double Kv0, Kv1, Kv2;
        };

        struct Att_Control_Ude{
            double Trou0, Trou1, Trou2;
        };

        struct Att_Control_Tvude{
            double Trou_min0, Trou_min1, Trou_min2;
            double Trou_max0, Trou_max1, Trou_max2;
            double t_min, t_max;
        };

        struct Att_Control_Eso{
            double Omg_eso_pos0, Omg_eso_pos1, Omg_eso_pos2;
        };

        struct Gain{
            Pos_Control_Pid pos;
            Vel_Control_Pid vel;
            Att_Control_Pid att_pid;
            Att_Control_Ude att_ude;
            Att_Control_Tvude att_tvude;
            Att_Control_Eso att_eso;
        };

        struct AutoTakeoff{
            double height;
            double speed;
        };

        struct kinematicsConstains{
            double vel_ver_max;
            double vel_hor_max;
            double acc_ver_max;
            double acc_hor_max;
            double omega_yaw_max;
        };

        struct FsmParam{
            int frequncy;
        };

        struct MsgTimeOut{
            double odom;
            double cmd;
        };

        struct ThrustMapping{
            bool print_val;
            double K1;
            double K2;
            double K3;
            bool accurate_thrust_model;
            double hover_percentage;
        };

        int controller_type = 0; // 0 for position control, 1 for velocity control, 2:attitude, 3:angular_velocity
        int estimator_type = 0; // 0:ude, 1:TVUDE, 2:ESO
        double mass;
        double gra;
        double max_manual_vel;

        Gain gain;
        kinematicsConstains kine_cons;
        AutoTakeoff takeoff_state;
        FsmParam fsmparam;
        MsgTimeOut msg_timeout;
        ThrustMapping thr_map;
        
        void config_from_ros_handle(const ros::NodeHandle &nh);
    };
}
#endif