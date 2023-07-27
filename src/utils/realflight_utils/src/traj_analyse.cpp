#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "std_msgs/String.h"

#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>

namespace trajAnalyse {

class trajAls : public nodelet::Nodelet
{
  private:
    quadrotor_msgs::PositionCommand cmd;
    mavros_msgs::AttitudeTarget ctrl;
    geometry_msgs::PoseStamped gtruth;

    ros::Subscriber cmdSub;
    ros::Subscriber ctrlSub;
    ros::Subscriber gtruthSub;

    void cmdCallback(quadrotor_msgs::PositionCommand::ConstPtr &cmdMsg)
    {
        cmd = *cmdMsg;
    }

    void ctrlCallback(mavros_msgs::AttitudeTarget::ConstPtr &ctrlMsg)
    {
        ctrl = *ctrlMsg;
    }

    void gtruthCallback(geometry_msgs::PoseStamped::ConstPtr &gtruthMsg)
    {
        gtruth = *gtruthMsg;
    }

    void init(ros::NodeHandle& nh)
    {
        cmdSub = nh.subscribe(cmdTopic, 1, &trajAls::cmdCallback, this,
                                   ros::TransportHints().tcpNoDelay())
    }

  public:


};

} //namespace tarjAls
