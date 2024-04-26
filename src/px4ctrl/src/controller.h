/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>
#include <ros/ros.h>

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	Eigen::Vector3d omg;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  omg(Eigen::Vector3d::Zero()),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};


class LinearControl
{
public:
  LinearControl(Parameter_t &);
  quadrotor_msgs::Px4ctrlDebug calculateControlCMD(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      Controller_Output_t &u,
	  ros::Time &t,
	  const int ude_type);
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
	  const Odom_Data_t &odom,
	  const Imu_Data_t &imu, 
	  Controller_Output_t &u);
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);
  void resetThrustMapping(void);

  Eigen::Vector3d gettimevaryingT(double const t,
  	  double const t_min,
	  double const t_max,
	  Eigen::Vector3d const Trou_min,
	  Eigen::Vector3d const Trou_max);
  Eigen::Vector3d gettimevaryingTdot(double const t,
      double const t_min,
	  double const t_max,
	  Eigen::Vector3d const Trou_min,
	  Eigen::Vector3d const Trou_max);

  double getdisturbfromESO(Eigen::Vector3d parameter,
	  Eigen::Vector3d &x_est,
	  Eigen::Vector3d &x_est_dot,
	  double step,
  	  double u_0,
	  double x);

  bool is_first_in_control = true;
  ros::Time last_time;
  Eigen::Vector3d u0_integral_pos; //Interference estimation value
  Eigen::Vector3d u0_integral_att;
  Eigen::Vector3d init_state;
  ros::Time init_t;
  
  // use in postion ESO est
  Eigen::Vector3d pos_x_est;
  Eigen::Vector3d pos_x_est_dot;
  Eigen::Vector3d pos_y_est;
  Eigen::Vector3d pos_y_est_dot;
  Eigen::Vector3d pos_z_est;
  Eigen::Vector3d pos_z_est_dot;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};


#endif
