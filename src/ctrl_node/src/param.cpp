#include "param.hpp"

namespace ctrl_node{

    template <typename TName, typename TVal>
	void Parameter_t::read_param(const ros::NodeHandle &nh, const TName &name, TVal &val){
		if (!nh.getParam(name, val)){
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};

    void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh){
        read_param(nh, "gain/Kvp0", gain.vel.Kvp0);
        read_param(nh, "gain/Kvp1", gain.vel.Kvp1);
        read_param(nh, "gain/Kvp2", gain.vel.Kvp2);
        read_param(nh, "gain/Kvi0", gain.vel.Kvi0);
        read_param(nh, "gain/Kvi1", gain.vel.Kvi1);
        read_param(nh, "gain/Kvi2", gain.vel.Kvi2);
        read_param(nh, "gain/Kvd1", gain.vel.Kvd1);
        read_param(nh, "gain/Kvd2", gain.vel.Kvd2);
        read_param(nh, "gain/Kp0", gain.att_pid.Kp0);
        read_param(nh, "gain/Kp1", gain.att_pid.Kp1);
        read_param(nh, "gain/Kp2", gain.att_pid.Kp2);
        read_param(nh, "gain/Kv0", gain.att_pid.Kv0);
        read_param(nh, "gain/Kv1", gain.att_pid.Kv1);
        read_param(nh, "gain/Kv2", gain.att_pid.Kv2);
        read_param(nh, "gain/Trou0", gain.att_ude.Trou0);
        read_param(nh, "gain/Trou1", gain.att_ude.Trou1);
        read_param(nh, "gain/Trou2", gain.att_ude.Trou2);
        read_param(nh, "gain/Trou_min0", gain.att_tvude.Trou_min0);
        read_param(nh, "gain/Trou_min1", gain.att_tvude.Trou_min1);
        read_param(nh, "gain/Trou_min2", gain.att_tvude.Trou_min2);
        read_param(nh, "gain/Trou_max0", gain.att_tvude.Trou_max0);
        read_param(nh, "gain/Trou_max1", gain.att_tvude.Trou_max1);
        read_param(nh, "gain/Trou_max2", gain.att_tvude.Trou_max2);
        read_param(nh, "gain/t_min", gain.att_tvude.t_min);
        read_param(nh, "gain/t_max", gain.att_tvude.t_max);
        read_param(nh, "gain/Omg_eso_pos0", gain.att_eso.Omg_eso_pos0);
        read_param(nh, "gain/Omg_eso_pos1", gain.att_eso.Omg_eso_pos1);
        read_param(nh, "gain/Omg_eso_pos2", gain.att_eso.Omg_eso_pos2);

        read_param(nh, "takeoff_state/height", takeoff_state.height);
        read_param(nh, "takeoff_state/speed", takeoff_state.speed);

        read_param(nh, "fsmparam/frequncy", fsmparam.frequncy);

        read_param(nh, "msg_timeout/odom", msg_timeout.odom);
        read_param(nh, "msg_timeout/cmd", msg_timeout.cmd);

        read_param(nh, "controller_type", controller_type);
        read_param(nh, "estimator_type", estimator_type);
        read_param(nh, "mass", mass);
        read_param(nh, "gra", gra);
        read_param(nh, "max_manual_vel", max_manual_vel);

        read_param(nh, "kine_cons/vel_ver_max", kine_cons.vel_ver_max);
        read_param(nh, "kine_cons/vel_hor_max", kine_cons.vel_hor_max);
        read_param(nh, "kine_cons/acc_ver_max", kine_cons.acc_ver_max);
        read_param(nh, "kine_cons/acc_hor_max", kine_cons.acc_hor_max);
        read_param(nh, "kine_cons/omega_yaw_max", kine_cons.omega_yaw_max);

        read_param(nh, "thrust_model/print_value", thr_map.print_val);
        read_param(nh, "thrust_model/K1", thr_map.K1);
        read_param(nh, "thrust_model/K2", thr_map.K2);
        read_param(nh, "thrust_model/K3", thr_map.K3);
        read_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
        read_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);

    }
}