#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include <ros/ros.h>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/Px4ctrlDebug.h>

#include "input.hpp"
#include "param.hpp"

namespace Controller{

    // template <typename Scalar_t>
    // Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_) {
    //     Eigen::Quaternion<Scalar_t> q = q_.normalized();

    //     Eigen::Matrix<Scalar_t, 3, 1> ypr;
    //     ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    //     ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
    //     ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    //     return ypr;
    // }
    static double q2yaw(const Eigen::Quaterniond &ori)
    {
        return atan2(2.0*(ori.x()*ori.y() + ori.w()*ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z()));
    }

    struct Desired_State_t{
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Vector3d a;
        Eigen::Vector3d j;
        Eigen::Quaterniond q;
        Eigen::Vector3d omg;
        double yaw;
        double yaw_rate;

        Desired_State_t(){};

        Desired_State_t(ctrl_node::Odom_Data_t &odom):
            p(odom.p),
            v(Eigen::Vector3d::Zero()),
            a(Eigen::Vector3d::Zero()),
            j(Eigen::Vector3d::Zero()),
            q(odom.q),
            omg(Eigen::Vector3d::Zero()),
            yaw(q2yaw(odom.q)),
            yaw_rate(0){};
    };

    struct Controller_Output_t
    {
        /* position and velocity controller output */
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d pos_last;
        Eigen::Vector3d vel_last;
        double yaw, yaw_last;

        /* attitude and rate controller output */
        Eigen::Quaterniond q; // Orientation of the body frame with respect to the world frame
        Eigen::Vector3d bodyrates; // Body rates in body frame, [rad/s]
        double thrust; // Collective mass normalized thrust
    };

    class Position_Control{
        private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;

        public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, 
                                                      const ctrl_node::Odom_Data_t &odom, 
                                                      Controller_Output_t &u);
    };

    class Velocity_Control{
        private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;
        ctrl_node::Odom_Data_t odom_last_;
        Controller_Output_t input;

        double get_vel_err(const Desired_State_t &des, const ctrl_node::Odom_Data_t &odom);

        public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, 
                                                      const ctrl_node::Odom_Data_t &odom, 
                                                      Controller_Output_t &u);
    };

    class Attitude_Angular_Control{
        private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;
        std::queue<std::pair<ros::Time, double>> timed_thrust_;
        static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

        // Thrust-accel mapping params
        const double rho2_ = 0.998; // do not change
        double thr2acc_;
        double P_;

        double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);

        public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControlCMD(const Desired_State_t &des,
                                                        const ctrl_node::Odom_Data_t &odom,
                                                        const ctrl_node::Imu_Data_t &imu, 
                                                        Controller_Output_t &u,
                                                        ros::Time &t);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                        const ctrl_node::Odom_Data_t &odom,
                                                        const ctrl_node::Imu_Data_t &imu, 
                                                        Controller_Output_t &u);
        bool estimateThrustModel(const Eigen::Vector3d &est_v, const ctrl_node::Parameter_t &param);
        void resetThrustMapping();

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
    };
}

#endif