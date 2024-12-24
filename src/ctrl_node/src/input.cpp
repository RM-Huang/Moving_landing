#include "input.hpp"

namespace ctrl_node{

    template <typename Scalar_t>
    Scalar_t normalize_angle(Scalar_t a) {
        int cnt = 0;
        while (true) {
            cnt++;

            if (a < -M_PI) {
                a += M_PI * 2.0;
            } else if (a > M_PI) {
                a -= M_PI * 2.0;
            }

            if (-M_PI <= a && a <= M_PI) {
                break;
            };

            assert(cnt < 10 && "[INPUT]: INVALID INPUT ANGLE");
        }

        return a;
    };

    Odom_Data_t::Odom_Data_t(){
        rcv_stamp = ros::Time(0);
        q.setIdentity();
        recv_new_msg = false;
    };

    void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg){
        ros::Time now = pMsg->header.stamp;
        msg.pose.pose = pMsg->pose.pose;
        rcv_stamp = now;
        recv_new_msg = true;

        if(odom_source == 0){
            // pmsg pvq is in world frame, w is in body frame
            p(0) = pMsg->pose.pose.position.x;
            p(1) = pMsg->pose.pose.position.y;
            p(2) = pMsg->pose.pose.position.z;

            v(0) = pMsg->twist.twist.linear.x;
            v(1) = pMsg->twist.twist.linear.y;
            v(2) = pMsg->twist.twist.linear.z;

            q.w() = pMsg->pose.pose.orientation.w;
            q.x() = pMsg->pose.pose.orientation.x;
            q.y() = pMsg->pose.pose.orientation.y;
            q.z() = pMsg->pose.pose.orientation.z;

            w(0) = pMsg->twist.twist.angular.x;
            w(1) = pMsg->twist.twist.angular.y;
            w(2) = pMsg->twist.twist.angular.z;
        }else if(odom_source == 1){
            //TODO
            ROS_ERROR("[INPUT]:odom source error");
        }
        
        // check the frequency
        static int one_min_count = 9999;
        static ros::Time last_clear_count_time = ros::Time(0.0);
        if ( (now - last_clear_count_time).toSec() > 1.0 )
        {
            if ( one_min_count < 100 )
            {
                ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
            }
            one_min_count = 0;
            last_clear_count_time = now;
        }
        one_min_count ++;
    };

    Imu_Data_t::Imu_Data_t(){
        rcv_stamp = ros::Time(0);
    }

    void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg){
        ros::Time now = ros::Time::now();

        msg = *pMsg;
        rcv_stamp = now;

        w(0) = msg.angular_velocity.x;
        w(1) = msg.angular_velocity.y;
        w(2) = msg.angular_velocity.z;

        a(0) = msg.linear_acceleration.x;
        a(1) = msg.linear_acceleration.y;
        a(2) = msg.linear_acceleration.z;

        q.x() = msg.orientation.x;
        q.y() = msg.orientation.y;
        q.z() = msg.orientation.z;
        q.w() = msg.orientation.w;

        // check the frequency
        static int one_min_count = 9999;
        static ros::Time last_clear_count_time = ros::Time(0.0);
        if ( (now - last_clear_count_time).toSec() > 1.0 )
        {
            if ( one_min_count < 100 )
            {
                ROS_WARN("IMU , which is too low!");
            }
            one_min_count = 0;
            last_clear_count_time = now;
        }
        one_min_count ++;
    }

    State_Data_t::State_Data_t(){
        previous_state.mode = "OFFBOARD";
    };

    void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg){
        current_state = *pMsg;
    };

    ExtendedState_Data_t::ExtendedState_Data_t(){
    };

    void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg){
        current_extended_state = *pMsg;
    };

    Command_Data_t::Command_Data_t(){
        rcv_stamp = ros::Time(0);
    }

    void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
    {

        msg = *pMsg;
        rcv_stamp = ros::Time::now();

        p(0) = msg.position.x;
        p(1) = msg.position.y;
        p(2) = msg.position.z;

        v(0) = msg.velocity.x;
        v(1) = msg.velocity.y;
        v(2) = msg.velocity.z;

        a(0) = msg.acceleration.x;
        a(1) = msg.acceleration.y;
        a(2) = msg.acceleration.z;

        j(0) = msg.jerk.x;
        j(1) = msg.jerk.y;
        j(2) = msg.jerk.z;

        yaw = normalize_angle(msg.yaw);
        yaw_rate = msg.yaw_dot;
    }
}