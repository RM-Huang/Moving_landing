#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TrajcurDesire.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/MotorlockTriger.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <traj_opt/traj_opt.h>
// #include "target_prediction/bezier_predict.h"

#include <Eigen/Core>
#include <atomic>
#include <thread>
#include <vis_utils/vis_utils.hpp>
#include <traj_opt/flatness.hpp>


namespace planning{
    class Nodelet : public nodelet::Nodelet {
        private:
        std::thread initThread_;
        ros::Subscriber triger_sub_;
        ros::Subscriber target_odom_sub_;
        ros::Subscriber vision_statu_sub_;
        ros::Subscriber uav_odom_sub_;
        ros::Subscriber ctrl_ready_tri_sub_;

        ros::Publisher cmd_pub_;
        ros::Publisher des_pub_;
        // ros::Publisher land_pub_;
        // ros::Publisher hover_pub_;
        ros::ServiceClient FCU_command_srv;

        int plan_hz_;

        ros::Timer plan_timer_;
        ros::Timer cmd_timer_;

        std::shared_ptr<vis_utils::VisUtils> visPtr_;
        std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

        // Using for prediction
        // int predict_seg;
        double sample_dur;
        double predict_dur;
        // Bezierpredict tgpredict;
        std::vector<Eigen::Vector4d> target_detect_list;
        bool predict_success = false;
        // std::vector<Eigen::MatrixXd> bezier_polyc_list;
        // std::vector<double> bezierT_list;
        // std::vector<double> bezier_init_time_list;

        // Using for planning timer
        Eigen::MatrixXd iniState;
        int plan_type; // 0 for sim, 1 for real
        double target_odom_time = 0;
        bool generate_new_traj_success = false;
        bool visualize_sig;
        bool target_odom_recrived = false;
        bool land_first = false;
        Trajectory traj;
        Eigen::Vector3d target_p, target_v, uav_p, uav_v;
        Eigen::Vector3d ekf_error;
        Eigen::Vector3d target_p_last, target_v_last;
        double vision_stamp = 0;
        double trajStamp_observe;
        Eigen::Quaterniond target_q, uav_q;
        traj_opt::TrajOpt::plan_s plan_state = traj_opt::TrajOpt::HOVER;

        Eigen::Vector3d follow_p;
        Eigen::Vector3d follow_v;

        // static param
        double vehicleMass;
        double gravAcc;
        double horizDrag;
        double vertDrag;
        double parasDrag;
        double speedEps;
        double robot_l_;
        double land_r_;
        double omega_yaw_max_;

        // NOTE just for debug
        bool debug_replan_ = false;
        bool ifanalyse =false;

        // double tracking_dur_, tracking_dist_, tolerance_d_;
        Eigen::Vector3d perching_p_, perching_v_, perching_axis_; // for simulation
        double perching_theta_;

        Trajectory traj_poly_;
        double trajStamp;
        double trigerStamp = 0; // time stamp for current plan start triger from other program

        bool ctrl_ready_triger = false;
        bool publishing_cmd = false;
        std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);


        void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr);

        void ctrl_ready_tri_callback(const geometry_msgs::PoseStampedConstPtr& msg);
        
        void vision_statu_callback(const std_msgs::Float64ConstPtr& msg);

        void uav_odom_callback(const nav_msgs::OdometryConstPtr& msg);

        void target_odom_callback(const nav_msgs::OdometryConstPtr& msg);

        void planning_fsm(const ros::TimerEvent& event);

        void cmd_pub(const ros::TimerEvent& event);

        bool force_arm_disarm(bool arm);

        void init(ros::NodeHandle& nh);

        public:
        void onInit(void);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}