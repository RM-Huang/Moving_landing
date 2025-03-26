#include "simulation_odom.hpp"

namespace odomSim{

    void odomRemap::missiontriCallback(const geometry_msgs::PoseStamped::ConstPtr &triMsg){
        mission_start_tri = true;
    }

    void odomRemap::uavsimCallback(const gazebo_msgs::ModelStates::ConstPtr &modelMsg){
        for(int i = 0; i < modelMsg->name.size(); i++){
            if(modelMsg->name[i] == "iris_0"){
                uav_odom.header.stamp = ros::Time::now();
                uav_odom.pose.pose = modelMsg->pose[i];
                uav_odom.twist.twist = modelMsg->twist[i];

                if(uav_sub_tri){
                    double uav_dur = abs(uav_odom.header.stamp.toSec() - uav_time_l);
                    if( uav_dur > DELAY_DUR_MAX ){
                        ROS_WARN("[odom_remap]:uav odom data update rate is too low! Update Dur: %f", uav_dur);
                    }
                }else{
                    uav_sub_tri = true;
                }
                uav_time_l = uav_odom.header.stamp.toSec();
            }
        }
    }

    void odomRemap::carsimCallback(const quadrotor_msgs::EstimatorOdom::ConstPtr &carMsg){
        car_odom = carMsg->car_odom;

        if(car_sub_tri){
            double car_dur = abs(car_odom.header.stamp.toSec() - car_time_l);
            if( car_dur > DELAY_DUR_MAX ){
                ROS_WARN("[odom_remap]:car odom data update rate is too low! Update Dur: %f", car_dur);
            }
        }else{
            car_sub_tri = true;
        }
        car_time_l = car_odom.header.stamp.toSec();
    }

    void odomRemap::visionsimCallback(const nav_msgs::Odometry::ConstPtr &visMsg){
        vision_odom = *visMsg;
    }

    void odomRemap::carbiasCallback(const quadrotor_msgs::EstimatorDebug::ConstPtr &biasMsg){
        car_bias.pos = Eigen::Vector3d(biasMsg->pose_bias.pose.position.x, 
                                        biasMsg->pose_bias.pose.position.y, 
                                        biasMsg->pose_bias.pose.position.z);
        car_bias.qua = Eigen::Quaterniond(biasMsg->pose_bias.pose.orientation.w,
                                            biasMsg->pose_bias.pose.orientation.x,
                                            biasMsg->pose_bias.pose.orientation.y,
                                            biasMsg->pose_bias.pose.orientation.z);
        car_bias.t = biasMsg->time_bias;
    }

    // 用来检测视觉偏差估计是否符合要求
    void odomRemap::check_vision_bias(const Eigen::Vector3d& remap_pos, const Eigen::Vector3d& vis_pos){
        if((remap_pos - vis_pos).norm() < 0.2){
            std_msgs::Float64 tri;
            tri.data = 1;
            land_tri_pub.publish(tri);
        }
    }

    void odomRemap::car_odom_remap(const Eigen::Vector3d& uav_pos, Eigen::Vector3d& car_pos, Eigen::Vector3d& car_vel, 
                                    Eigen::Quaterniond& car_qua, std_msgs::Float64& odom_source){

        Eigen::Vector3d vis_pos(vision_odom.pose.pose.position.x, vision_odom.pose.pose.position.y, vision_odom.pose.pose.position.z);

        car_vel = car_bias.qua * car_vel;
        car_qua = car_bias.qua * car_qua;
        car_pos = car_bias.qua * (car_pos + car_vel * car_bias.t) + car_bias.pos;

        nav_msgs::Odometry car_rec_msg;
        car_rec_msg.pose.pose.position.x = car_pos(0);
        car_rec_msg.pose.pose.position.y = car_pos(1);
        car_rec_msg.pose.pose.position.z = car_pos(2);
        car_rec_msg.pose.pose.orientation.w = car_qua.w();
        car_rec_msg.pose.pose.orientation.x = car_qua.x();
        car_rec_msg.pose.pose.orientation.y = car_qua.y();
        car_rec_msg.pose.pose.orientation.z = car_qua.z();
        car_rec_msg.twist.twist.linear.x = car_vel(0);
        car_rec_msg.twist.twist.linear.y = car_vel(1);
        car_rec_msg.twist.twist.linear.z = car_vel(2);

        if(sqrt(pow(uav_pos[0] - vis_pos[0], 2) + pow(uav_pos[1] - vis_pos[1], 2)) <= abs(uav_pos[2] - vis_pos[2]) * std::tan(M_PI / 4)){
            car_rec_msg.pose.pose.position.y = vision_odom.pose.pose.position.y;
            car_rec_msg.pose.pose.position.z = vision_odom.pose.pose.position.z;
            car_pos.y() = vision_odom.pose.pose.position.y;
            car_pos.z() = vision_odom.pose.pose.position.z;
        } // debug
        car_rec_msg.header.stamp = ros::Time::now();
        car_rec_pub.publish(car_rec_msg);

        if(abs(uav_pos[2] - vis_pos[2]) < 1.5 && abs(uav_pos[2] - vis_pos[2]) > 1.0){
            odom_source.data = 0;
        }else if(mission_start_tri && (abs(uav_pos[2] - vis_pos[2]) <= 0.5 ||
                sqrt(pow(uav_pos[0] - vis_pos[0], 2) + pow(uav_pos[1] - vis_pos[1], 2)) < abs(uav_pos[2] - vis_pos[2]) * std::tan(M_PI / 4))){
            check_vision_bias(car_pos, vis_pos);
            car_qua.w() = vision_odom.pose.pose.orientation.w;
            car_qua.x() = vision_odom.pose.pose.orientation.x;
            car_qua.y() = vision_odom.pose.pose.orientation.y;
            car_qua.z() = vision_odom.pose.pose.orientation.z;
            car_pos.x() = vision_odom.pose.pose.position.x;
            car_pos.y() = vision_odom.pose.pose.position.y;
            car_pos.z() = vision_odom.pose.pose.position.z;
            odom_source.data = 1;
        }
    }

    void odomRemap::read_odom(const nav_msgs::Odometry& msg, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Quaterniond& qua){
        pos(0) = msg.pose.pose.position.x;
        pos(1) = msg.pose.pose.position.y;
        pos(2) = msg.pose.pose.position.z;

        vel(0) = msg.twist.twist.linear.x;
        vel(1) = msg.twist.twist.linear.y;
        vel(2) = msg.twist.twist.linear.z;

        qua.w() = msg.pose.pose.orientation.w;
        qua.x() = msg.pose.pose.orientation.x;
        qua.y() = msg.pose.pose.orientation.y;
        qua.z() = msg.pose.pose.orientation.z;
    }

    void odomRemap::odom_handler(const ros::TimerEvent& time_event){
        if(uav_sub_tri && car_sub_tri){

            nav_msgs::Odometry car_msg;
            nav_msgs::Odometry uav_msg;
            std_msgs::Float64 source_msg;

            Eigen::Vector3d uav_pos, car_pos, uav_vel, car_vel;
            Eigen::Quaterniond uav_qua, car_qua;
            
            read_odom(uav_odom, uav_pos, uav_vel, uav_qua);
            read_odom(car_odom, car_pos, car_vel, car_qua);

            car_odom_remap(uav_pos, car_pos, car_vel, car_qua, source_msg);

            car_msg.pose.pose.position.x = car_pos(0);
            car_msg.pose.pose.position.y = car_pos(1);
            car_msg.pose.pose.position.z = car_pos(2);
            car_msg.pose.pose.orientation.w = car_qua.w();
            car_msg.pose.pose.orientation.x = car_qua.x();
            car_msg.pose.pose.orientation.y = car_qua.y();
            car_msg.pose.pose.orientation.z = car_qua.z();
            car_msg.twist.twist.linear.x = car_vel(0);
            car_msg.twist.twist.linear.y = car_vel(1);
            car_msg.twist.twist.linear.z = car_vel(2);
            car_msg.header.stamp = ros::Time::now();
            car_pub.publish(car_msg);

            uav_msg.pose.pose.position.x = uav_pos(0);
            uav_msg.pose.pose.position.y = uav_pos(1);
            uav_msg.pose.pose.position.z = uav_pos(2);
            uav_msg.pose.pose.orientation.w = uav_qua.w();
            uav_msg.pose.pose.orientation.x = uav_qua.x();
            uav_msg.pose.pose.orientation.y = uav_qua.y();
            uav_msg.pose.pose.orientation.z = uav_qua.z();
            uav_msg.twist.twist.linear.x = uav_vel(0);
            uav_msg.twist.twist.linear.y = uav_vel(1);
            uav_msg.twist.twist.linear.z = uav_vel(2);
            uav_msg.header.stamp = ros::Time::now();
            uav_pub.publish(uav_msg);

            vision_tri_pub.publish(source_msg);

        }else{
            while(!uav_sub_tri || !car_sub_tri)
            {
                if(!uav_sub_tri)
                {
                    ROS_ERROR("[odom_remap]:No uav odom data, please check rostopic.");
                }

                if(!car_sub_tri)
                {
                    ROS_ERROR("[odom_remap]:No car odom data, please check rostopic.");
                }
                ros::Duration(1.0).sleep();
            }   
        }
    }

    void odomRemap::init(ros::NodeHandle& nh){
        nh.param("use_vis", use_vis, 0); 

        uav_sub = nh.subscribe("uavTopic", 1, &odomRemap::uavsimCallback, this, ros::TransportHints().tcpNoDelay());
        car_sub = nh.subscribe("carTopic", 1, &odomRemap::carsimCallback, this, ros::TransportHints().tcpNoDelay());
        vision_sub = nh.subscribe("visTopic", 1, &odomRemap::visionsimCallback, this, ros::TransportHints().tcpNoDelay());
        car_bias_sub = nh.subscribe("carbiasTopic", 1, &odomRemap::carbiasCallback, this, ros::TransportHints().tcpNoDelay());
        mission_tri_sub = nh.subscribe("/landing_triger", 1, &odomRemap::missiontriCallback, this, ros::TransportHints().tcpNoDelay());

        uav_pub = nh.advertise<nav_msgs::Odometry>("/odom/remap", 5);
        car_pub = nh.advertise<nav_msgs::Odometry>("/odom/remap/car", 5);
        car_rec_pub = nh.advertise<nav_msgs::Odometry>("/odom/car_recover", 1);
        vision_tri_pub = nh.advertise<std_msgs::Float64>("/vision_received", 1);
        land_tri_pub = nh.advertise<std_msgs::Float64>("/land_state_triger", 1);


        handler_timer = nh.createTimer(ros::Duration(0.005), &odomRemap::odom_handler, this);

        ROS_INFO("\033[32m[simulation_odom]:node initiated!\033[32m");
    }

    void odomRemap::onInit(void){
        ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
        initThread_ = std::thread(std::bind(&odomRemap::init, this, nh)); //在单独的线程中运行Nodelet::init()      
    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(odomSim::odomRemap, nodelet::Nodelet);