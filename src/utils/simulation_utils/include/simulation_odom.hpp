#include <thread>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <quadrotor_msgs/EstimatorOdom.h>
#include <quadrotor_msgs/EstimatorDebug.h>

namespace odomSim{
    struct carBias
    {
        Eigen::Vector3d pos;
        Eigen::Quaterniond qua;
        double t = 0;
    };

    class odomRemap : public nodelet::Nodelet{
        private:
        std::thread initThread_;

        nav_msgs::Odometry uav_odom;
        nav_msgs::Odometry car_odom;
        nav_msgs::Odometry vision_odom;
        carBias car_bias;

        ros::Subscriber uav_sub;
        ros::Subscriber car_sub;
        ros::Subscriber vision_sub;
        ros::Subscriber car_bias_sub;
        ros::Subscriber mission_tri_sub;

        double DELAY_DUR_MAX = 0.02;
        bool uav_sub_tri = false;
        bool car_sub_tri = false;
        bool mission_start_tri = false;
        double uav_time_l = 0.0;
        double car_time_l = 0.0;

        ros::Publisher uav_pub;
        ros::Publisher car_pub;
        ros::Publisher car_rec_pub;
        ros::Publisher vision_tri_pub;
        ros::Publisher land_tri_pub;

        ros::Timer handler_timer;

        int use_vis = 0;

        void missiontriCallback(const geometry_msgs::PoseStamped::ConstPtr &triMsg);

        void uavsimCallback(const gazebo_msgs::ModelStates::ConstPtr &modelMsg);

        void carsimCallback(const quadrotor_msgs::EstimatorOdom::ConstPtr &carMsg);

        void visionsimCallback(const nav_msgs::Odometry::ConstPtr &visMsg);

        void carbiasCallback(const quadrotor_msgs::EstimatorDebug::ConstPtr &biasMsg);

        void read_odom(const nav_msgs::Odometry& msg, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Quaterniond& qua);

        void check_vision_bias(const Eigen::Vector3d& remap_pos, const Eigen::Vector3d& vis_pos);

        void car_odom_remap(const Eigen::Vector3d& uav_pos, Eigen::Vector3d& car_pos, Eigen::Vector3d& car_vel, 
                            Eigen::Quaterniond& car_qua, std_msgs::Float64& odom_source);

        void odom_handler(const ros::TimerEvent& time_event);

        void init(ros::NodeHandle& nh);

        public:
        void onInit(void);
    };
}