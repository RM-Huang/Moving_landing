#ifndef __TRAJ_FOLLOW_H
#define __TRAJ_FOLLOW_H

#include <ros/ros.h>
// #include <Eigen/Dense>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#define PI 3.141592653589793238327950
// #define flt_altitude 1.5f
// #define rate            20  // 频率 hz
// #define RADIUS          5   // 绕八运动的半径大小 m
// #define CYCLE_S         15  // 完成一次绕八运动所花费的时间
// #define STEPS           (CYCLE_S*rate) // 步数


class Traj_Follow
{
public:
    void init_path();

private:
    int rate;
    int steps;
    double cycle_s;
    double r;
    float flt_altitude;
    mavros_msgs::State current_state; 
};

void Traj_Follow::init_path()
{
    mavros_msgs::PositionTarget path[steps];

    int i;
    const double dt = 1.0/rate;
    const double dadt = (2.0*PI)/cycle_s;   // 恒定角速度

    for(i=0;i<steps;i++)
    {
        path[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 本地坐标系
        path[i].type_mask = 0; // 不设置忽略

        
        double a = (-PI/2.0) + i*(2.0*PI/steps); // 起始角为270度
        double c = cos(a);
        double c2a = cos(2.0*a);
        double c4a = cos(4.0*a);
        double c2am3 = c2a - 3.0;
        double s = sin(a);
        double cc = c*c;
        double ss = s*s;
        double sspo = (s*s)+1.0;
        double ssmo = (s*s)-1.0;
        double sspos = sspo * sspo;

        path[i].position.x = (r*c) / sspo; 
        path[i].position.y = -(r*c*s) / sspo;
        path[i].position.z = flt_altitude;

        path[i].velocity.x = -dadt*r*s*( ss + 2.0f*cc + 1.0f) / sspos;
        path[i].velocity.y = dadt*r*( ss*ss + ss + ssmo*cc) /sspos;
        path[i].velocity.z = 0;

        path[i].acceleration_or_force.x = -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/(c2am3*c2am3*c2am3);
        path[i].acceleration_or_force.y = dadt*dadt*r*((44.0*c2a) + c4a - 21.0)/(c2am3*c2am3*c2am3);
        path[i].acceleration_or_force.z = 0.0;

        path[i].yaw = atan2(-path[i].velocity.x,path[i].velocity.y) + (PI/2.0f);

        printf("x:%7.3f y:%7.3f yaw:%7.1f\n",path[i].position.x,path[i].position.y,path[i].yaw*180.0f/PI);

    }
    for(i=0;i<steps;i++){
        double next = path[(i+1)%steps].yaw;
        double curr = path[i].yaw;
        if((next-curr) < -PI) next+=(2.0*PI);
        if((next-curr) > PI) next-=(2.0*PI);
        path[i].yaw_rate = (next-curr)/dt;
    }
}


#endif