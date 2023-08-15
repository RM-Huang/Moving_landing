#ifndef __UAVUTILS_CONVERTERS_H
#define __UAVUTILS_CONVERTERS_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace uav_utils {

inline void extract_odometry_gps(geometry_msgs::PoseStampedConstPtr msg, Eigen::Vector3d& p, Eigen::Vector3d& p_l,
                      Eigen::Vector3d p_init, double dur, Eigen::Vector3d& v, Eigen::Quaterniond& q)
{
    p(0) = - ( msg->pose.position.x - p_init(0) );
    p(1) = - ( msg->pose.position.y - p_init(1) );
    p(2) = msg->pose.position.z - p_init(2);

    v(0) = (p(0) - p_l(0)) / dur;
    v(1) = (p(1) - p_l(1)) / dur;
    v(2) = (p(2) - p_l(2)) / dur;

    p_l = p;

    q.w() = msg->pose.orientation.w;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
}

inline void extract_odometry(geometry_msgs::PoseStampedConstPtr msg, Eigen::Vector3d& p, Eigen::Vector3d& p_l,
                      Eigen::Vector3d p_init, double dur, Eigen::Vector3d& v, Eigen::Quaterniond& q)
{
    p(0) = msg->pose.position.x - p_init(0);
    p(1) = msg->pose.position.y - p_init(1);
    p(2) = msg->pose.position.z - p_init(2);

    // if ( ( ( abs(p(0) - p_l(0)) > abs(2 * v(0) * dur) ) || ( abs(p(1) - p_l(1)) > abs(2 * v(1) * dur) ) || ( abs(p(2) - p_l(2)) > abs(2 * v(2) * dur) ) ) && ( v(0) != 0 ) && ( v(1) != 0) && ( v(2) != 0) )
    // {
    //     std::cout<<"p = "<<p(0)<<" "<<p(1)<<" "<<p(2)<<std::endl;
    //     std::cout<<"v = "<<v(0)<<" "<<v(1)<<" "<<v(2)<<std::endl;
    //     p(0) = p_l(0) + v(0) * dur * 0.3;
    //     p(1) = p_l(1) + v(1) * dur * 0.3;
    //     p(2) = p_l(2) + v(2) * dur * 0.3;
    // }

    v(0) = (p(0) - p_l(0)) / dur;
    v(1) = (p(1) - p_l(1)) / dur;
    v(2) = (p(2) - p_l(2)) / dur;

    p_l = p;

    q.w() = msg->pose.orientation.w;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
}

inline void extract_odometry(geometry_msgs::PoseStampedConstPtr msg, Eigen::Vector3d& p, Eigen::Vector3d& p_l,
                      Eigen::Vector3d p_init, double dur, Eigen::Vector3d& v, Eigen::Quaterniond& q, Eigen::Vector3d& w, int o_s)
{
    if(o_s == 1)
        extract_odometry_gps(msg, p, p_l, p_init, dur, v, q);
    else
        extract_odometry(msg, p, p_l, p_init, dur, v, q);

    // w(0) = msg->twist.twist.angular.x;
    // w(1) = msg->twist.twist.angular.y;
    // w(2) = msg->twist.twist.angular.z;
    w(0) = 0;
    w(1) = 0;
    w(2) = 0;
}


template <typename Scalar_t = double>
Eigen::Matrix<Scalar_t, 3, 1> from_vector3_msg(const geometry_msgs::Vector3& msg) {
    return Eigen::Matrix<Scalar_t, 3, 1>(msg.x, msg.y, msg.z);
}

template <typename Derived>
geometry_msgs::Vector3 to_vector3_msg(const Eigen::DenseBase<Derived>& v) {
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    geometry_msgs::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

template <typename Scalar_t = double>
Eigen::Matrix<Scalar_t, 3, 1> from_point_msg(const geometry_msgs::Point& msg) {
    return Eigen::Matrix<Scalar_t, 3, 1>(msg.x, msg.y, msg.z);
}

template <typename Derived>
geometry_msgs::Point to_point_msg(const Eigen::DenseBase<Derived>& v) {
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    
    geometry_msgs::Point msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

template <typename Scalar_t = double>
Eigen::Quaternion<Scalar_t> from_quaternion_msg(const geometry_msgs::Quaternion& msg) {
    return Eigen::Quaternion<Scalar_t>(msg.w, msg.x, msg.y, msg.z);
}

template <typename Scalar_t>
geometry_msgs::Quaternion to_quaternion_msg(const Eigen::Quaternion<Scalar_t>& q) {
    geometry_msgs::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}
}

#endif
