#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include <nodelet/nodelet.h>
#include "polyfit.hpp"
#include "ekf.hpp"

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geodesy/utm.h>
#include </home/rose/Moving_landing/devel/include/chcnav/hcinspvatzcb.h>
#include <car_odom_server/car_status.h>
#include <car_odom_server/SerialPort.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
// #include </home/nuc/Moving_landing/src/utils/mavlink_msg/common/common/mavlink.h>  // change to your PC
// #include </home/pc205/Moving_landing/src/utils/mavlink_msg/common/common/mavlink.h>  // change to your PC

namespace odomRemap{

class odomRemap : public nodelet::Nodelet
{
private:
    polyfit::Fit fit;
    SerialPort port2;

    std::thread initThread_;
    std::recursive_mutex mtx;
    ros::Timer odom_timer;
    ros::Timer car_odom_timer;

    nav_msgs::Odometry gtruth;
    geometry_msgs::TwistStamped gtruth_enu_vel;
    std::vector<double> uav_global;
    car_odom_server::car_status car_odom;
    // nav_msgs::Odometry car_odom;
    geometry_msgs::Vector3 CU_pos_init_differ;
    // geometry_msgs::Vector3 car_rpy_bias;
    geometry_msgs::Vector3 gtruth_pos_bias;
    geometry_msgs::Vector3 gtruth_rpy_bias;
    geometry_msgs::Vector3 gtruth_rpy;
    Eigen::Quaterniond gtruth_qua_bias;
    Eigen::Quaterniond car_qua_bias;
    tf::Quaternion gtruth_Q2T;
          
    sensor_msgs::Imu imu;

    ros::Subscriber gtruthSub;
    ros::Subscriber gtruthvelSub;
    ros::Subscriber imuSub;
    ros::Subscriber uav_globalposSub;
    // use for simulation
    bool issimulation;
    ros::Subscriber car_odomSub;
    nav_msgs::Odometry car_odom_sim;

    std::string gtruthTopic;

    ros::Publisher odomPub;
    ros::Publisher car_odomPub;
    ros::Publisher car_odom_rawPub;
    // ros::Publisher imuvelrawPub; // publish imu_vel msg with merely acc integral

    std::vector<double> imuvel_x; // imuvel store the unfit data
    std::vector<double> imuvel_y;
    std::vector<double> imuvel_z;
    std::vector<double> imu_t;

    double imu_t0; // coefficient for imu_t return to zero
    double gtruth_time_delay = 0;
    double gtruth_time_l;
    bool imusubTri = false;
    bool uavvelsubTri = false;
    bool gtruthsubTri = false;
    bool calibration = false;
    bool lastreceived = true;
    bool carodomsubTri;
    int seq = 0;
    int bias_c = 0;
    int fit_size;
    // int car_odom_remap;

    ros::Publisher vision_getPub;
    ros::Publisher vision_rawPub;
    ros::Subscriber visionsub; 
    Eigen::Quaterniond uav_orientation;  
    Eigen::Vector3d uav_position;        
    std_msgs::Float64 vision_statu;
    int source = 0;
    int vision_source = 0;
    Eigen::Quaterniond vision_ori;
    Eigen::Vector3d vision_position;
    int recieve=0;
    bool zero = false;
    // Eigen::Vector3d uav_pose_zero;
    // Eigen::Quaterniond uav_ori_;
    // Eigen::Vector3d uav_ori_0;
    // Eigen::Vector3d uav_ori_zero;
    // Eigen::Quaterniond to_vision;
    /* car ekf param */
    int flag = 1;//跳转flag数据,(无视觉1,刚进入视觉2,在视觉中0)
    std::vector<double> error_detect_list_x;
    std::vector<double> error_detect_list_y;
    std::vector<double> error_detect_list_z;
    std::vector<double> list_time;
    std::vector<double>  weight_list; 

    void visionCallback(const apriltag_ros::AprilTagDetectionArray &transform){ 
        // vision_time = static_cast<double>(transform.header.stamp.Sec)+static_cast<double>(transform.header.stamp.NSec)/1e9;
        vision_position.x() = transform.detections[0].pose.pose.pose.position.x;
        vision_position.y() = transform.detections[0].pose.pose.pose.position.y;
        vision_position.z() = transform.detections[0].pose.pose.pose.position.z;//-

        vision_ori.w() = transform.detections[0].pose.pose.pose.orientation.w;
        vision_ori.x() = transform.detections[0].pose.pose.pose.orientation.x;//-
        vision_ori.y() = transform.detections[0].pose.pose.pose.orientation.y;//-
        vision_ori.z() = transform.detections[0].pose.pose.pose.orientation.z;

        
    }

    void irCallback(const geometry_msgs::PoseStamped::ConstPtr &irmsg)
    {
        vision_position.x() = irmsg->pose.position.x;
        vision_position.y() = irmsg->pose.position.y;
        vision_position.z() = irmsg->pose.position.z;

        vision_ori.w() = irmsg->pose.orientation.w;
        vision_ori.x() = irmsg->pose.orientation.x;
        vision_ori.y() = irmsg->pose.orientation.y;
        vision_ori.z() = irmsg->pose.orientation.z;
        
    }

    void mctruthCallback(const geometry_msgs::PoseStamped::ConstPtr &gtruthMsg)
    {
        gtruth.header = gtruthMsg->header;
        gtruth.pose.pose = gtruthMsg->pose;
        gtruthsubTri = true;
        gtruth.pose.pose.position.x;
    }

    void gpstruthCallback(const nav_msgs::Odometry::ConstPtr &gtruthMsg)
    {
        if(!uavvelsubTri)
        {
            return;
        }
        gtruth.child_frame_id = gtruthMsg->child_frame_id;
        gtruth.header = gtruthMsg->header;
        gtruth.pose = gtruthMsg->pose;
        gtruth.twist.twist = gtruth_enu_vel.twist;
        gtruthsubTri = true;
    }

    void LLTtoUTM(const double& latitude, const double& longitude, geometry_msgs::Vector3& point)
    {
        geographic_msgs::GeoPoint geo_pt;
        geo_pt.latitude = latitude;
        geo_pt.longitude = longitude;
        geo_pt.altitude = 0;
        geodesy::UTMPoint utm_pt(geo_pt);
        point.x = utm_pt.easting;
        point.y = utm_pt.northing;
    }

    void carodomCallback(const nav_msgs::Odometry::ConstPtr &carodomMsg)
    {
        double car_time_delay = carodomMsg->header.stamp.toSec() - car_odom.time.toSec();
        if((abs(carodomMsg->header.stamp.toSec() - car_odom.time.toSec()) >= 0.04)){
            ROS_WARN("[odom_remap]:car odom data update rate is too low!");
            std::cout<<"car_data_delay= "<<car_time_delay<<std::endl; //test
        }

        geometry_msgs::Vector3 utm;
        LLTtoUTM(carodomMsg->pose.pose.position.x, carodomMsg->pose.pose.position.y, utm);
        car_odom.time = carodomMsg->header.stamp;
        car_odom.px = utm.x;
        car_odom.py = utm.y;
        car_odom.pz = carodomMsg->pose.pose.position.z;
        car_odom.roll = 0.0; // debug
        car_odom.pitch = 0.0; // debug
        car_odom.yaw = carodomMsg->twist.twist.angular.z;
        car_odom.vx = carodomMsg->twist.twist.linear.x;
        car_odom.vy = carodomMsg->twist.twist.linear.y;
        car_odom.vz = carodomMsg->twist.twist.linear.z;
        // car_odom is in ENU
        // car_odom_rawPub.publish(car_odom); //debug

        carodomsubTri = true;
    }

    void uavvelCallback(const geometry_msgs::TwistStamped::ConstPtr &gtruthvelMsg)
    {
        gtruth_enu_vel = *gtruthvelMsg;
        uavvelsubTri = true;
    }

    void simtruthCallback(const gazebo_msgs::ModelStates::ConstPtr &modelMsg)
    {
        for(int i = 0; i < modelMsg->name.size(); i++)
        {
            if(modelMsg->name[i] == "iris_0")
            {
                gtruth.header.stamp = ros::Time::now();
                gtruth.pose.pose = modelMsg->pose[i];
                gtruth.twist.twist = modelMsg->twist[i];
                gtruthsubTri = true;
            }
        }
    }
    void sim_car_odom_Callback(const nav_msgs::Odometry::ConstPtr &carMsg)
    {
        car_odom_sim = *carMsg;
        carodomsubTri = true;
    }

    // void car_odom_Callback(const ros::TimerEvent& time_event)
    // {
    //     static char buffer[512];
    //     int ret = 0;
    //     // std::cout<<"car_odom_thread"<<std::endl;
    //     if (port2._is_open)
    //     {
    //         // std::cout<<"port_is_open"<<std::endl;
    //         memset(buffer, 0, 512);
    //         ret = port2.read(buffer, 512);
    //         if (ret > 0)
    //         {
    //             // mtx.lock();
    //             // std::cout << "success: buffer=\t" << ret << std::endl;
    //             static mavlink_message_t mavlink_msg;
    //             static mavlink_status_t status;
    //             for (int i = 0; i < ret; ++i) {
    //                 if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &mavlink_msg, &status)){
    //                     switch (mavlink_msg.msgid) {
    //                         case 204:
    //                         {
    //                             mavlink_car_status_t car;
    //                             mavlink_msg_car_status_decode(&mavlink_msg, &car);
    //                             static uint32_t tick = 0;
    //                             car_odom.pitch = car.pitch;
    //                             car_odom.yaw = car.yaw;
    //                             car_odom.roll = car.roll;
    //                             car_odom.px = car.px;
    //                             car_odom.py =car.py;
    //                             car_odom.pz = car.pz;
    //                             car_odom.vx = car.vx;
    //                             car_odom.vy= car.vy;
    //                             car_odom.vz = car.vz;
    //                             car_odom.status = car.status;
    //                             car_odom_rawPub.publish(car_odom);
    //                             break;
    //                         }
    //                     }
    //                 }
    //             }
    //             carodomsubTri = true;
    //             // mtx.unlock();
    //         }
    //     }
    // }

    void gtruth_const_bias_cal(const double &gtruth_t)
    {
        double current_t = imu.header.stamp.toSec();

        double flash_dur;
        if(bias_c > 0)
        {
            flash_dur = abs( abs(gtruth_t - current_t) - gtruth_time_delay / bias_c );
        }
        else
        {
            flash_dur = abs( abs(gtruth_t - current_t) - gtruth_time_delay);
            gtruth_time_delay = 0; // delete first gtruth_time_delay for total time bias cal
        }

        if( flash_dur < 0.5)
        {
            bias_c += 1;

            gtruth_time_delay = gtruth_time_delay + abs(gtruth_t - current_t);
            

            gtruth_pos_bias.x = gtruth_pos_bias.x + gtruth.pose.pose.position.x;
            gtruth_pos_bias.y = gtruth_pos_bias.y + gtruth.pose.pose.position.y;
            gtruth_pos_bias.z = gtruth_pos_bias.z + gtruth.pose.pose.position.z;

            tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
            tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_rpy.x, gtruth_rpy.y, gtruth_rpy.z);

            gtruth_rpy_bias.x = gtruth_rpy_bias.x + gtruth_rpy.x;
            gtruth_rpy_bias.y = gtruth_rpy_bias.y + gtruth_rpy.y;
            gtruth_rpy_bias.z = gtruth_rpy_bias.z + gtruth_rpy.z;

            // std::cout<<"bias_c = "<<bias_c<<std::endl;
            // std::cout<<"delta_r_tol = "<<gtruth_rpy_bias.x<<" delta_p_tol = "<<gtruth_rpy_bias.y<<" delta_y_tol = "<<gtruth_rpy_bias.z<<std::endl;
            // std::cout<<"delta_t_tol = "<<gtruth_time_delay<<" delta_x_tol = "<<gtruth_pos_bias.x<<" delta_y_tol = "<<gtruth_pos_bias.y<<" delta_z_tol = "<<gtruth_pos_bias.z<<std::endl;
        }
        else
        {
            gtruth_time_delay = abs(gtruth_t - current_t);
            seq = 0;
            bias_c = 0;
            std::cout<<"time delay haven't stable"<<std::endl;
        }
    }

    void gtruth_const_bias_cal()
    {
        // Eigen::Quaterniond car_qua_bias;
        bias_c += 1;

        gtruth_pos_bias.x = gtruth_pos_bias.x + gtruth.pose.pose.position.x;
        gtruth_pos_bias.y = gtruth_pos_bias.y + gtruth.pose.pose.position.y;
        gtruth_pos_bias.z = gtruth_pos_bias.z + gtruth.pose.pose.position.z;

        // tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
        // tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_rpy.x, gtruth_rpy.y, gtruth_rpy.z);

        // gtruth_rpy_bias.x = gtruth_rpy_bias.x + gtruth_rpy.x;
        // gtruth_rpy_bias.y = gtruth_rpy_bias.y + gtruth_rpy.y;
        // gtruth_rpy_bias.z = gtruth_rpy_bias.z + gtruth_rpy.z;

        if(issimulation)
        {
            CU_pos_init_differ.x = car_odom_sim.pose.pose.position.x;
            CU_pos_init_differ.y = car_odom_sim.pose.pose.position.y;
            CU_pos_init_differ.z = car_odom_sim.pose.pose.position.z;
            car_qua_bias.w() = car_odom_sim.pose.pose.orientation.w;
            car_qua_bias.x() = car_odom_sim.pose.pose.orientation.x;
            car_qua_bias.y() = car_odom_sim.pose.pose.orientation.y;
            car_qua_bias.z() = car_odom_sim.pose.pose.orientation.z;

            gtruth_qua_bias.w() += gtruth.pose.pose.orientation.w;
            gtruth_qua_bias.x() += gtruth.pose.pose.orientation.x;
            gtruth_qua_bias.y() += gtruth.pose.pose.orientation.y;
            gtruth_qua_bias.z() += gtruth.pose.pose.orientation.z; //debug
            calibration = true;
        }
        else
        {
            geometry_msgs::Vector3 car_rpy_bias;
            CU_pos_init_differ.x += car_odom.px;
            CU_pos_init_differ.y += car_odom.py;
            CU_pos_init_differ.z += car_odom.pz;
            car_rpy_bias.x = car_odom.roll;
            car_rpy_bias.y = car_odom.pitch;
            car_rpy_bias.z = car_odom.yaw;
            Eigen::AngleAxisd roll(Eigen::AngleAxisd(car_rpy_bias.x,Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitch(Eigen::AngleAxisd(car_rpy_bias.y,Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yaw(Eigen::AngleAxisd(car_rpy_bias.z,Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond cur_qua_bias = roll * pitch * yaw;

            car_qua_bias.w() += cur_qua_bias.w();
            car_qua_bias.x() += cur_qua_bias.x();
            car_qua_bias.y() += cur_qua_bias.y();
            car_qua_bias.z() += cur_qua_bias.z();
            // std::cout<<"CU_pos_init_differ = "<<CU_pos_init_differ.x<<" "<<CU_pos_init_differ.y<<CU_pos_init_differ.z<<std::endl;
            gtruth_qua_bias.w() += cur_qua_bias.w();
            gtruth_qua_bias.x() += cur_qua_bias.x();
            gtruth_qua_bias.y() += cur_qua_bias.y();
            gtruth_qua_bias.z() += cur_qua_bias.z(); //debug
            calibration = true;
        }
        
        // gtruth_qua_bias.w() += gtruth.pose.pose.orientation.w;
        // gtruth_qua_bias.x() += gtruth.pose.pose.orientation.x;
        // gtruth_qua_bias.y() += gtruth.pose.pose.orientation.y;
        // gtruth_qua_bias.z() += gtruth.pose.pose.orientation.z; //debug

        // std::cout<<"bias_c = "<<bias_c<<std::endl;
        // std::cout<<"delta_r_tol = "<<gtruth_rpy_bias.x<<" delta_p_tol = "<<gtruth_rpy_bias.y<<" delta_y_tol = "<<gtruth_rpy_bias.z<<std::endl;
        // std::cout<<"delta_t_tol = "<<gtruth_time_delay<<" delta_x_tol = "<<gtruth_pos_bias.x<<" delta_y_tol = "<<gtruth_pos_bias.y<<" delta_z_tol = "<<gtruth_pos_bias.z<<std::endl;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        imu = *imuMsg;
        imusubTri = true;
    }

    void imuvel_cal(geometry_msgs::Vector3 &vel)
    {
        int data_c = imu_t.size();
        int order = 2;

        if(imu_t.size() > (fit_size - 1) )
        {
            imu_t.erase(imu_t.begin());
            imuvel_x.erase(imuvel_x.begin());
            imuvel_y.erase(imuvel_y.begin());
            imuvel_z.erase(imuvel_z.begin());
        }

        if(imu_t.size() < 1)
        {
            imu_t0 = imu.header.stamp.toSec();
        }

        imu_t.emplace_back(imu.header.stamp.toSec() - imu_t0);

        if(data_c > order)
        {
            double imu_dur;
            
            imu_dur = imu_t[data_c - 1] - imu_t[data_c - 2];
            vel.x = imuvel_x[data_c - 1] + imu.linear_acceleration.x * imu_dur;
            vel.y = imuvel_y[data_c - 1] + imu.linear_acceleration.y * imu_dur;
            vel.z = imuvel_z[data_c - 1] + imu.linear_acceleration.z * imu_dur;

            // imuvelrawPub.publish(vel); //test
            
            imuvel_x.emplace_back(vel.x);
            imuvel_y.emplace_back(vel.y);
            imuvel_z.emplace_back(vel.z);

            fit.polyfit(imu_t, imuvel_x, order, false);
            double fit_x = fit.getY(imu_t[data_c - 1]);
            vel.x = imuvel_x[data_c - 1] - fit.getY(imu_t[data_c - 1]);

            fit.polyfit(imu_t, imuvel_y, order, false);
            double fit_y = fit.getY(imu_t[data_c - 1]);
            vel.y = imuvel_y[data_c - 1] - fit.getY(imu_t[data_c - 1]);

            fit.polyfit(imu_t, imuvel_z, order, false);
            double fit_z = fit.getY(imu_t[data_c - 1]);
            vel.z = imuvel_z[data_c - 1] - fit.getY(imu_t[data_c - 1]);
        }
        else
        {
            imuvel_x.resize(order+1, 0);
            imuvel_y.resize(order+1, 0);
            imuvel_z.resize(order+1, 0);
        }     
    }

    void motion_cal(Eigen::Quaterniond& qua)
    {
        Eigen::Quaterniond gtruth_orientation;
        gtruth_orientation.w() = gtruth.pose.pose.orientation.w;
        gtruth_orientation.x() = gtruth.pose.pose.orientation.x;
        gtruth_orientation.y() = gtruth.pose.pose.orientation.y;
        gtruth_orientation.z() = gtruth.pose.pose.orientation.z;

        qua = gtruth_qua_bias.inverse() * gtruth_orientation;
    }

    void motion_cal(Eigen::Quaterniond& qua, Eigen::Vector3d& pos, Eigen::Vector3d& lin_vel, Eigen::Vector3d& ang_vel)
    {
        Eigen::Quaterniond gtruth_orientation;
        Eigen::Vector3d p(gtruth.pose.pose.position.x, gtruth.pose.pose.position.y, gtruth.pose.pose.position.z);
        Eigen::Vector3d p_bias(gtruth_pos_bias.x, gtruth_pos_bias.y, gtruth_pos_bias.z);
        Eigen::Vector3d l_v(gtruth.twist.twist.linear.x, gtruth.twist.twist.linear.y, gtruth.twist.twist.linear.z);
        Eigen::Vector3d a_v(gtruth.twist.twist.angular.x, gtruth.twist.twist.angular.y, gtruth.twist.twist.angular.z);
        // Eigen::Vector3d l_v(0,0,0);
        // Eigen::Vector3d a_v(0,0,0);

        gtruth_orientation.w() = gtruth.pose.pose.orientation.w;
        gtruth_orientation.x() = gtruth.pose.pose.orientation.x;
        gtruth_orientation.y() = gtruth.pose.pose.orientation.y;
        gtruth_orientation.z() = gtruth.pose.pose.orientation.z;

        qua = gtruth_qua_bias.inverse() * gtruth_orientation;
        pos = gtruth_qua_bias.inverse() * (p - p_bias);
        lin_vel = gtruth_qua_bias.inverse() * l_v;
        // ang_vel = gtruth_qua_bias.inverse() * a_v;
    }

    void car_ekf_handler(int& flag, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& ekf_error)
    {
        if(flag == 0){//视觉数据进行         
            //1.估计值   
            Ekf::filter_calibrate(pos(0), pos(1), pos(2), 
                                  vel(0), vel(1), vel(2));

            pos[0] = Ekf::x_x[0];
            pos[1] = Ekf::x_y[0];
            pos[2] = Ekf::x_z[0];

            //2.段时间误差值输出
            //2.1)储存25个观测误差值
            error_detect_list_x = Ekf::list_cb(Ekf::err_x[0]);
            error_detect_list_y = Ekf::list_cb(Ekf::err_y[0]);
            error_detect_list_z = Ekf::list_cb(Ekf::err_z[0]);

            //2.2)生成权重
            double stamp=0;
            for(int i = 0; i < Ekf::_MAX_SEG; i++){
                list_time.push_back(stamp);
                stamp = stamp + 0.005; 
            }
              //tanh weight
            for(int i = 0; i < Ekf::_MAX_SEG; i++){
                double tanh_input = list_time[Ekf::_MAX_SEG - 1] - list_time[i];
                if(!tanh_input){
                    weight_list.push_back(1);
                }
                else{
                    tanh_input = 1.0 / tanh_input;
                    weight_list.push_back(tanh(0.2 * tanh_input));
                }
            }
            //2.3)求和
            for(int i = 0; i < Ekf::_MAX_SEG; i++){
                    ekf_error[0] = ekf_error[0] + weight_list[i]*error_detect_list_x[i];//x轴
                    ekf_error[1] = ekf_error[1] + weight_list[i]*error_detect_list_y[i];//y轴
                    ekf_error[2] = ekf_error[2] + weight_list[i]*error_detect_list_z[i];//z轴
            }
           
        }
        else if(flag == 1){//刚进入视觉
            //初始化
            Ekf::x_x << pos(0),vel(0);    
            Ekf::x_y << pos(1),vel(1);        
            Ekf::x_z << pos(2),vel(2);  
            Ekf::P_x << 1000, 0,  0,1000;  Ekf::P_y << 1000, 0,  0,1000;  Ekf::P_z << 1000, 0,  0,1000;

            flag = 0;

            Ekf::x_x_old = Ekf::x_x;
            Ekf::x_y_old = Ekf::x_y;
            Ekf::x_z_old = Ekf::x_z;
        }
    }

    void car_motion_cal(Eigen::Quaterniond& qua, Eigen::Vector3d& pos, Eigen::Vector3d& lin_vel, Eigen::Vector3d& ekf_error)
    {
    	int sour_last = source;
        Eigen::Vector3d l_v(car_odom.vx, car_odom.vy, car_odom.vz);
        lin_vel = car_qua_bias.inverse() * l_v; 
        lin_vel[2] = 0.0;
        // lin_vel = gtruth_qua_bias.inverse() * l_v; 
        // lin_vel = l_v;
        if(vision_position.x() == 0 && vision_position.y() == 0 && vision_position.z() == 0 && vision_ori.w() == 0 && vision_ori.x() ==0 && vision_ori.y() == 0 && vision_ori.z()==0)
        {
            Eigen::Quaterniond car_orientation;
            Eigen::Vector3d p(car_odom.px, car_odom.py, car_odom.pz);
            Eigen::Vector3d p_bias(CU_pos_init_differ.x, CU_pos_init_differ.y, CU_pos_init_differ.z);
            // Eigen::Vector3d l_v(0,0,0);
            // Eigen::Vector3d a_v(0,0,0);

            Eigen::AngleAxisd roll(Eigen::AngleAxisd(car_odom.roll,Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitch(Eigen::AngleAxisd(car_odom.pitch,Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yaw(Eigen::AngleAxisd(car_odom.yaw,Eigen::Vector3d::UnitZ()));

            car_orientation = roll * pitch * yaw;
            qua = car_qua_bias.inverse() * car_orientation;
            
            //qua.w() = 1;
            //qua.x() = 0;
            //qua.y() = 0;
            //qua.z() = 0; //debug

            pos = car_qua_bias.inverse() * (p - p_bias);
            // pos[2] = pos[2] - 0.05; // move gps center to camera
            pos[2] = 0.0;
            
            source = 0;
            flag = 1;
        }
        else if(vision_source == 0)
        {
            
            // cal_a.w()=0.707;
            // cal_a.x()=0;
            // cal_a.y()=0;
            // cal_a.z()=0.707;
                        
            // cal_b.w()=0 ;
            // cal_b.x()=1;
            // cal_b.y()=0;
            // cal_b.z()=0;

            // //vision_ori_cal = cal_a*cal_b;
            // vision_ori_cal = cal_b;
            // vision_ori_cal.normalize();
            // vision_pose_cal.x()=0.14083589;
            // vision_pose_cal.y()= -0.006509123;
            // vision_pose_cal.z()=0.14074158;        
            // //Eigen::Vector3d uav_v_position = cal_a*cal_a.inverse()*vision_ori_cal *(vision_position + vision_pose_cal );
            // //Eigen::Quaterniond uav_v_orientation = cal_a*vision_ori_cal * vision_ori;
            // uav_v_position =  cal_a.inverse() * (vision_position + vision_pose_cal);
            // uav_v_orientation = vision_ori_cal.inverse() * vision_ori;
            // //vision_local_orientation.normalize();
            // uav_v_position_117.x() = uav_v_position.x()-0.03;//-0.13;//bianxiao
            // uav_v_position_117.y() = -uav_v_position.y()-0.143;//-0.145;//bianxiao
            // uav_v_position_117.z() = -uav_v_position.z()+0.04;;
            // uav_v_orientation_117.w() = -uav_v_orientation.w();
            // uav_v_orientation_117.x() = uav_v_orientation.x();
            // uav_v_orientation_117.y() = -uav_v_orientation.y();
            // uav_v_orientation_117.z() = uav_v_orientation.z();
            
            // pos = uav_position - uav_orientation * uav_v_position_117;   //uav_orientation改成实时的无人机位置信息
            // qua = uav_orientation * uav_v_orientation_117.inverse();               //uav_position同上
            Eigen::Vector3d vision_pose_cal;
            Eigen::Quaterniond vision_ori_cal;
            Eigen::Vector3d angular;
            Eigen::Vector3d vision_ori_right;
            Eigen::Vector3d camera_position;
            Eigen::Quaterniond camera_ori;
            Eigen::Vector3d april_local_position;
            Eigen::Quaterniond april_local_ori;
            vision_pose_cal.x() = - vision_position.y();
            vision_pose_cal.y() = - vision_position.x();
            vision_pose_cal.z() = - vision_position.z();
            Eigen::Quaterniond cal(0,1,0,0);
            Eigen::Quaterniond cal_a = cal.inverse() * vision_ori;
            Eigen::Vector3d euler_angles = cal_a.toRotationMatrix().eulerAngles(2, 1, 0);
            vision_ori_right =  euler_angles;
            // if (euler_angles[0]>1.57){
            //     vision_ori_right[0] =  euler_angles[0] - M_PI;
            // }
            // else if(euler_angles[0]< -1.57){
            //     vision_ori_right[0] =  - euler_angles[0] + M_PI;
            // }else{
            //     ROS_INFO("Over 90 degree");
            // }
            
            

            angular[2] =  -vision_ori_right[1];
            angular[1] =  -vision_ori_right[2];
            angular[0] =  vision_ori_right[0];
            Eigen::Vector3d to_vision = uav_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(-to_vision(2), Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(-to_vision(1), Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(angular(0), Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond vision_quaternion;
            vision_quaternion = yawAngle * pitchAngle * rollAngle;

            camera_position = uav_position +  uav_orientation * Eigen::Vector3d(0.04,0,-0.14) + Eigen::Vector3d(0,0,0.265); 
            camera_ori = uav_orientation;  
            april_local_position = camera_ori * vision_pose_cal + camera_position;
            april_local_ori = camera_ori * camera_ori.inverse();
            pos = april_local_position;
            qua = april_local_ori;
            /* debug msg */
            geometry_msgs::Pose vision_raw;
            vision_raw.orientation.w = qua.w();
            vision_raw.orientation.x = qua.x();
            vision_raw.orientation.y = qua.y();
            vision_raw.orientation.z = qua.z();
            vision_raw.position.x = pos[0];
            vision_raw.position.y = pos[1];
            vision_raw.position.z = pos[2];
            vision_rawPub.publish(vision_raw);

            car_ekf_handler(flag, pos, lin_vel, ekf_error);
            //qua.z() = qua.z();
            qua.x() = 0;
            qua.y() = 0;
            qua.normalize();
            
            source = 1;
        }
        else if(vision_source == 1)
        {
            qua = uav_orientation * vision_ori;
            pos = uav_position - uav_orientation * vision_position;
            pos[2] -= 0.23;
            source = 1;
        }
        // lin_vel = gtruth_qua_bias.inverse() * l_v; 
        vision_statu.data = source;
        
        if(sour_last != source)
        {
            if(source == 0)
            {
            	ROS_INFO("[odom_remap]: Change odom source to gps!");
            }
            else
            {
               ROS_INFO("[odom_remap]: Change odom source to vision!");
            }
        }
    }

    void car_motion_cal_sim(Eigen::Quaterniond& qua, Eigen::Vector3d& pos)
    {
        Eigen::Vector3d p(car_odom_sim.pose.pose.position.x, car_odom_sim.pose.pose.position.y, car_odom_sim.pose.pose.position.z);
        Eigen::Vector3d p_bias(CU_pos_init_differ.x, CU_pos_init_differ.y, CU_pos_init_differ.z);
        Eigen::Quaterniond q;

        q.w() = car_odom_sim.pose.pose.orientation.w;
        q.x() = car_odom_sim.pose.pose.orientation.x;
        q.y() = car_odom_sim.pose.pose.orientation.y;
        q.z() = car_odom_sim.pose.pose.orientation.z;
        // Eigen::Vector3d l_v(0,0,0);
        // Eigen::Vector3d a_v(0,0,0);

        qua = gtruth_qua_bias.inverse() * q;
        pos = gtruth_qua_bias.inverse() * (p - p_bias);
    }

    void mc_odom_pub(const ros::TimerEvent& time_event)
    {
        // std::cout<<"---------------"<<std::endl;
        if(imusubTri && gtruthsubTri)
        {
            seq += 1;
            double gtruth_t = gtruth.header.stamp.toSec();

            if(seq < 300)
            {
                gtruth_const_bias_cal(gtruth_t);    
            }
            else if(seq == 300)
            {
                gtruth_time_delay = gtruth_time_delay / bias_c;
                gtruth_pos_bias.x = gtruth_pos_bias.x / bias_c;
                gtruth_pos_bias.y = gtruth_pos_bias.y / bias_c;
                gtruth_pos_bias.z = gtruth_pos_bias.z / bias_c;
                gtruth_rpy_bias.x = gtruth_rpy_bias.x / bias_c;
                gtruth_rpy_bias.y = gtruth_rpy_bias.y / bias_c;
                gtruth_rpy_bias.z = gtruth_rpy_bias.z / bias_c;

                gtruth_qua_bias = Eigen::AngleAxisd(gtruth_rpy_bias.z, Eigen::Vector3d::UnitZ()) * 
                    Eigen::AngleAxisd(gtruth_rpy_bias.y, Eigen::Vector3d::UnitY()) * 
                    Eigen::AngleAxisd(gtruth_rpy_bias.x, Eigen::Vector3d::UnitX());

                ROS_INFO("[odom_remap]:Odom const bias cal succeed, ready to flight!");

                std::cout<<"delta_r = "<<gtruth_rpy_bias.x * 180 / 3.14159265<<" delta_p = "<<gtruth_rpy_bias.y * 180 / 3.14159265<<" delta_y = "<<gtruth_rpy_bias.z * 180 / 3.14159265<<std::endl;
                // std::cout<<"delta_t = "<<gtruth_time_delay<<" delta_x = "<<gtruth_pos_bias.x<<" delta_y = "<<gtruth_pos_bias.y<<" delta_z = "<<gtruth_pos_bias.z<<std::endl;
            }
            else
            {
                //  std::cout<<"truth_data_delta= "<<abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec())<<std::endl;
                // std::cout<<"dur_judge = "<<abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec())<<std::endl;
                if( (abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec()) < 0.05))
                {
                    geometry_msgs::Vector3 vel;
                    nav_msgs::OdometryPtr odomMsg(new nav_msgs::Odometry);
                    Eigen::Quaterniond gtruth_qua;

                    imuvel_cal(vel);
                    motion_cal(gtruth_qua);

                    odomMsg->header.stamp = ros::Time().fromSec(gtruth_t - gtruth_time_delay);
                    odomMsg->pose.pose.position.x = gtruth.pose.pose.position.y - gtruth_pos_bias.y;
                    odomMsg->pose.pose.position.y = -(gtruth.pose.pose.position.x - gtruth_pos_bias.x);
                    odomMsg->pose.pose.position.z = gtruth.pose.pose.position.z - gtruth_pos_bias.z;
                    odomMsg->pose.pose.orientation.w = gtruth_qua.w();
                    odomMsg->pose.pose.orientation.x = gtruth_qua.x();
                    odomMsg->pose.pose.orientation.y = gtruth_qua.y();
                    odomMsg->pose.pose.orientation.z = gtruth_qua.z();
                    odomMsg->twist.twist.linear.x = vel.x;
                    odomMsg->twist.twist.linear.y = vel.y;
                    odomMsg->twist.twist.linear.z = vel.z;
                    odomMsg->twist.twist.angular.x = 0;
                    odomMsg->twist.twist.angular.y = 0;
                    odomMsg->twist.twist.angular.z = 0;

                    odomPub.publish(odomMsg);
                }
                else
                {
                    ROS_WARN("[odom_remap]:Truth data delay too high!");
                    std::cout<<"truth_data_delay= "<<abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec())<<std::endl;
                }
            }
        }
        else
        {
            ROS_ERROR("[odom_remap]:No odom or imu data, please check rostopic.");
            while(!imusubTri || !gtruthsubTri)
            {
                ros::Duration(0.2).sleep();
            }
        }
        // gtruth_time_l = gtruth.header.stamp.toSec();
    }

    void gps_odom_pub(const ros::TimerEvent& time_event)
    {
        if(gtruthsubTri && carodomsubTri)
        {
            seq += 1;

            if(seq < 300)
            {
                gtruth_const_bias_cal();    
            }
            else if(seq == 300)
            {
                gtruth_pos_bias.x = gtruth_pos_bias.x / bias_c;
                gtruth_pos_bias.y = gtruth_pos_bias.y / bias_c;
                gtruth_pos_bias.z = gtruth_pos_bias.z / bias_c;
                CU_pos_init_differ.x /= bias_c;
                CU_pos_init_differ.y /= bias_c;
                CU_pos_init_differ.z /= bias_c;
                gtruth_qua_bias.w() = gtruth_qua_bias.w() / bias_c;
                gtruth_qua_bias.x() = gtruth_qua_bias.x() / bias_c;
                gtruth_qua_bias.y() = gtruth_qua_bias.y() / bias_c;
                gtruth_qua_bias.z() = gtruth_qua_bias.z() / bias_c;
                car_qua_bias.w() /= bias_c;
                car_qua_bias.x() /= bias_c;
                car_qua_bias.y() /= bias_c;
                car_qua_bias.z() /= bias_c;

                ROS_INFO("[odom_remap]:Odom const bias cal succeed, ready to flight!");

                std::cout<<"delta_x = "<<gtruth_pos_bias.x<<" delta_y = "<<gtruth_pos_bias.y<<" delta_z = "<<gtruth_pos_bias.z<<std::endl;
                Eigen::Matrix3d rx = gtruth_qua_bias.toRotationMatrix();
                Eigen::Vector3d eular_bias = rx.eulerAngles(2,1,0);
                std::cout<<"delta_r = "<<eular_bias[0] * 180 / 3.14159265<<" delta_p = "<<eular_bias[1] * 180 / 3.14159265<<" delta_y = "<<eular_bias[2] * 180 / 3.14159265<<std::endl;
            }
            else
            {
                /* uav publish */
                if( (abs(gtruth.header.stamp.toSec() - gtruth_time_l) < 0.04))
                {
                    nav_msgs::OdometryPtr odomMsg(new nav_msgs::Odometry);
                    Eigen::Quaterniond gtruth_qua;
                    Eigen::Vector3d position;
                    Eigen::Vector3d lin_vel;
                    Eigen::Vector3d ang_vel;

                    motion_cal(gtruth_qua, position, lin_vel, ang_vel);

                    odomMsg->header = gtruth.header;
                    odomMsg->child_frame_id = gtruth.child_frame_id;
                    odomMsg->pose.pose.position.x = position[0];
                    odomMsg->pose.pose.position.y = position[1];
                    odomMsg->pose.pose.position.z = position[2];
                    odomMsg->pose.covariance = gtruth.pose.covariance;
                    odomMsg->pose.pose.orientation.w = gtruth_qua.w();
                    odomMsg->pose.pose.orientation.x = gtruth_qua.x();
                    odomMsg->pose.pose.orientation.y = gtruth_qua.y();
                    odomMsg->pose.pose.orientation.z = gtruth_qua.z();
                    odomMsg->twist.twist.linear.x = lin_vel[0];
                    odomMsg->twist.twist.linear.y = lin_vel[1];
                    odomMsg->twist.twist.linear.z = lin_vel[2];
                    odomMsg->twist.twist.angular = gtruth.twist.twist.angular;
                    // odomMsg->twist.twist.angular.x = ang_vel[0];
                    // odomMsg->twist.twist.angular.y = ang_vel[1];
                    // odomMsg->twist.twist.angular.z = ang_vel[2];

                    uav_orientation = gtruth_qua;
                    uav_position = position;

                    odomPub.publish(odomMsg);
                        
                        /* car odom publish */
                        if(calibration)
                        {
                            nav_msgs::Odometry car_odomMsg;
                            Eigen::Quaterniond car_qua;
                            Eigen::Vector3d car_pos;
                            Eigen::Vector3d car_vel;
                            Eigen::Vector3d ekf_error(0.0,0.0,0.0);

                            if(!issimulation)
                            {
                                car_motion_cal(car_qua, car_pos, car_vel, ekf_error);

                                car_odomMsg.header = gtruth.header;
                                car_odomMsg.child_frame_id = gtruth.child_frame_id;
                                car_odomMsg.pose.pose.position.x = car_pos[0];
                                car_odomMsg.pose.pose.position.y = car_pos[1];
                                car_odomMsg.pose.pose.position.z = car_pos[2];
                                car_odomMsg.pose.covariance[0] = ekf_error[0];
                                car_odomMsg.pose.covariance[1] = ekf_error[1];
                                car_odomMsg.pose.covariance[2] = ekf_error[2];
                                car_odomMsg.pose.pose.orientation.w = car_qua.w();
                                car_odomMsg.pose.pose.orientation.x = car_qua.x();
                                car_odomMsg.pose.pose.orientation.y = car_qua.y();
                                car_odomMsg.pose.pose.orientation.z = car_qua.z();
                                car_odomMsg.twist.twist.linear.x = car_vel[0];
                                car_odomMsg.twist.twist.linear.y = car_vel[1];
                                car_odomMsg.twist.twist.linear.z = car_vel[2];
                                // car_odomMsg.twist.twist.linear.z = 0;
                                vision_getPub.publish(vision_statu);
                            }
                            else
                            {
                                car_motion_cal_sim(car_qua, car_pos);

                                car_odomMsg.header = gtruth.header;
                                car_odomMsg.child_frame_id = gtruth.child_frame_id;
                                car_odomMsg.pose.pose.position.x = car_pos[0];
                                car_odomMsg.pose.pose.position.y = car_pos[1];
                                car_odomMsg.pose.pose.position.z = car_pos[2];
                                car_odomMsg.pose.covariance[0] = ekf_error[0];
                                car_odomMsg.pose.covariance[1] = ekf_error[1];
                                car_odomMsg.pose.covariance[2] = ekf_error[2];
                                car_odomMsg.pose.pose.orientation.w = car_qua.w();
                                car_odomMsg.pose.pose.orientation.x = car_qua.x();
                                car_odomMsg.pose.pose.orientation.y = car_qua.y();
                                car_odomMsg.pose.pose.orientation.z = car_qua.z();
                                car_odomMsg.twist.twist.linear = car_odom_sim.twist.twist.linear;
                            }
                            car_odomPub.publish(car_odomMsg);
                        }
                        else
                        {
                            ROS_ERROR("[odom_remap]:car odom calibration haven't finished");
                        }
                        // mtx.unlock();
                    // }
                }
                else
                {
                    if((abs(gtruth.header.stamp.toSec() - gtruth_time_l) >= 0.04)){
                        ROS_WARN("[odom_remap]:odom data update rate is too low!");
                        std::cout<<"truth_data_delay= "<<abs(gtruth.header.stamp.toSec() - gtruth_time_l)<<std::endl; //test
                    }

                    // if((abs(car_odom.time - car_time_l) >= 0.04)){
                    //     ROS_WARN("[odom_remap]:car odom data update rate is too low!");
                    //     std::cout<<"car_data_delay= "<<abs(car_odom.time - car_time_l)<<std::endl; //test
                    // }
                }
            }
        }
        else
        {
            // if(!gtruthsubTri)
            // {
            //     ROS_ERROR("[odom_remap]:No uav odom data, please check rostopic.");
            // }

            // if(!carodomsubTri)
            // {
            //     ROS_ERROR("[odom_remap]:No car odom data, please check rostopic.");
            // }

            while(!gtruthsubTri || !carodomsubTri)
            {
                if(!gtruthsubTri)
                {
                    ROS_ERROR("[odom_remap]:No uav odom data, please check rostopic.");
                }

                if(!carodomsubTri)
                {
                    ROS_ERROR("[odom_remap]:No car odom data, please check rostopic.");
                }
                ros::Duration(1.0).sleep();
            }   
        }
        gtruth_time_l = gtruth.header.stamp.toSec();
        // car_time_l = car_odom.time;
    }

    // void car_odom_server_init()
    // {
    //     SerialPort::OpenOptions uartOptions;
    //     uartOptions = SerialPort::defaultOptions;
    //     uartOptions.baudRate = SerialPort::BR38400;
    //     port2.open("/dev/ttyUSB0",uartOptions);
    //     if (port2.isOpen())
    //     {
    //         ROS_WARN("[odom_remap]:Port2 open success!");
    //     }
    //     else
    //     {
    //         ROS_ERROR("[odom_remap]:Port2 open failed!");
    //     }
    // }

    void ekf_param_init()
    {
        /* ekf param init */
        double dt = 0.005;
        double ep =  0.7; //位置标准差
        double ev =  7; //速度标准差
        //矩阵初始化
        Ekf::F << 1, dt, 0, 1;
        Ekf::H << 1.0, 0.0,  0.0, 1.0;
        Ekf::R <<(ep*ep), 0,  0, (ev*ev);
        //Q初始化
        Eigen::MatrixXd E_V(2,2);
        E_V << 0 ,0 ,0 , 0.1*0.1;
        Ekf::Q = (Ekf::F) * E_V* (Ekf::F.transpose());
        //状态变量初始化
        Ekf::x_x_old << 0,0;     Ekf::x_y_old << 0,0;    Ekf::x_z_old << 0,0;
    }

    void init(ros::NodeHandle& nh)
    {
        int odom_source;
        nh.getParam("gtruthTopic", gtruthTopic);
        nh.param("fit_size", fit_size, 100);
        nh.param("odom_source", odom_source, 0); // 0 for mocap, 1 for gps
        nh.param("simulation", issimulation, false); 
        // nh.param("car_odom_remap", car_odom_remap, 0); // 1 for remap car odom
        nh.param("vision_source", vision_source, 0); // 0 for aprilTag, 1 for infrared

        gtruth_pos_bias.x = 0;
        gtruth_pos_bias.y = 0;
        gtruth_pos_bias.z = 0;
        gtruth_rpy_bias.x = 0;
        gtruth_rpy_bias.y = 0;
        gtruth_rpy_bias.z = 0;
        uav_global.resize(3);

        // imuSub = nh.subscribe("/mavros/imu/data", 10, &odomRemap::imuCallback, this);

        odomPub = nh.advertise<nav_msgs::Odometry>("/odom/remap", 1);
        car_odomPub = nh.advertise<nav_msgs::Odometry>("/odom/remap/car", 1);
        // imuvelrawPub = nh.advertise<geometry_msgs::Vector3>("/mavros/imu/data/linear_velocity_raw",10);//test
        switch (odom_source) {
            case 0:
                gtruthSub = nh.subscribe(gtruthTopic, 1, &odomRemap::mctruthCallback, this,
                                   ros::TransportHints().tcpNoDelay());

                odom_timer = nh.createTimer(ros::Duration(0.005), &odomRemap::mc_odom_pub, this);
                ROS_INFO("[odom_remap]:Using Mocap for odom calculate.");
                break;
            case 1:
                if(!issimulation)
                {
                    gtruthSub = nh.subscribe(gtruthTopic, 1, &odomRemap::gpstruthCallback, this,
                                ros::TransportHints().tcpNoDelay());

                    gtruthvelSub = nh.subscribe("/mavros/local_position/velocity_local", 1, &odomRemap::uavvelCallback, this,
                                ros::TransportHints().tcpNoDelay());

                    // uav_globalposSub = nh.subscribe("/mavros/global_position/global", 1, &odomRemap::uavglobalCallback, this,
                    //                 ros::TransportHints().tcpNoDelay());

                    car_odom_rawPub = nh.advertise<car_odom_server::car_status>("/odom/remap/car/raw", 1);
                    vision_rawPub = nh.advertise<geometry_msgs::Pose>("odom/remap/vision_raw", 1);
                    
                    if(vision_source == 0)
                        visionsub = nh.subscribe("/tag_detections", 1, &odomRemap::visionCallback, this);  // april_tag
                    else if(vision_source == 1)
                        visionsub = nh.subscribe("/ir_pose_topic", 1, &odomRemap::irCallback, this); // infrared

                    vision_getPub = nh.advertise<std_msgs::Float64>("/vision_received", 1); 
                    // car_odom_server_init();
                    ekf_param_init();
                    car_odomSub = nh.subscribe("/chcnav/car_odom", 1, &odomRemap::carodomCallback, this);
                    // car_odom_timer = nh.createTimer(ros::Duration(0.01), &odomRemap::car_odom_Callback, this);
                }
                else
                {
                    gtruthSub = nh.subscribe(gtruthTopic, 1, &odomRemap::simtruthCallback, this,
                                ros::TransportHints().tcpNoDelay());

                    car_odomSub = nh.subscribe("/smart/odom", 1, &odomRemap::sim_car_odom_Callback, this,
                                        ros::TransportHints().tcpNoDelay());
                }

                // debugSub = nh.subscribe("/mavros/local_position/pose", 10, &odomRemap::localposeCallback, this,
                //                     ros::TransportHints().tcpNoDelay()); // debug
                odom_timer = nh.createTimer(ros::Duration(0.005), &odomRemap::gps_odom_pub, this);
                ROS_INFO("[odom_remap]:Using GPS for odom calculate.");
                break;
        }
    }

public:
    void onInit(void)
    {
        ros::NodeHandle nh(getMTPrivateNodeHandle()); //线程并行回调
        initThread_ = std::thread(std::bind(&odomRemap::init, this, nh)); //在单独的线程中运行Nodelet::init()      
    }
};
} // namespace odomRemap
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(odomRemap::odomRemap, nodelet::Nodelet);
