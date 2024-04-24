#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <car_odom_server/car_status.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ekf_test/ekf.hpp>

Eigen::Vector3d car_odom_pos;
Eigen::Vector3d car_odom_vel;
Eigen::Vector3f vision_position;
Eigen::Quaternionf vision_ori;
ros::Publisher ekf_pub;

//flag判断
bool odom_sub_tri = false;
int flag=1 ;//跳转flag数据,(无视觉1,刚进入视觉2,在视觉中0)
std::vector<double> last_detect_list(50);
//误差输出 
std::vector<double> error_detect_list_x;
std::vector<double> error_detect_list_y;
std::vector<double> error_detect_list_z;
std::vector<double> list_time;
std::vector<double>  weight_list; 
Eigen::Vector3d error_out ;//px py pz 
//估计值输出 
Eigen::Vector3d pos_out;


/*小车里程计话题，v*/
void car_odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_sub_tri = true;

    car_odom_pos(0) = msg->pose.pose.position.x;
    car_odom_pos(1) = msg->pose.pose.position.y;
    car_odom_pos(2) = msg->pose.pose.position.z;

    car_odom_vel(0) = msg->twist.twist.linear.x;
    car_odom_vel(1) = msg->twist.twist.linear.y;
    car_odom_vel(2) = msg->twist.twist.linear.z;

    
}


void handler()
{
    /************************************拟合值*********************************/
    if(odom_sub_tri == true)//接收到观测值
    {
    //STEP1:判断状态(无视觉1,刚进入视觉2,在视觉中0)
        //存储当前时刻前50帧数据(观测数据更新时间0.05s)
        int _SEG = 50;//0.06s/0.005s
        last_detect_list.erase(last_detect_list.begin());
        last_detect_list.push_back(car_odom_pos(2));        

        //前50帧数据为0+检测到视觉数据=>flag置2,才更新初值
        if( car_odom_pos(2) != 0 ){
            double m =0;
            //50帧皆为0才判为表明刚进入视觉=>初始化状态变量和协方差矩阵
            for(int i = 0; i < _SEG-1; i++){
                m = m + last_detect_list[i];
            }
            if(m == 0)  flag = 2;  
            else  flag = 0; 
        } 

        //前50帧数据为0=>无视觉数据=>flag置0  
        else if(car_odom_pos(2) == 0 ){
            double m =0;
            for(int i = 40; i < _SEG-1; i++){
                m = m + last_detect_list[i];
            }
            if(m == 0)  flag = 1;  
            else  flag = 0; 
        }
        
       
    //STEP2:处理不同状态(无视觉1,刚进入视觉2,在视觉中0)
        if(flag == 0){//视觉数据进行         
            //1.估计值   
            Ekf::filter_calibrate(car_odom_pos(0), car_odom_pos(1), car_odom_pos(2), 
                                  car_odom_vel(0), car_odom_vel(1), car_odom_vel(2));

            pos_out[0] = Ekf::x_x[0];
            pos_out[1] = Ekf::x_y[0];
            pos_out[2] = Ekf::x_z[0];

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
            error_out <<0,0,0;
            for(int i = 0; i < Ekf::_MAX_SEG; i++){
                    error_out[0] = error_out[0] + weight_list[i]*error_detect_list_x[i];//x轴
                    error_out[1] = error_out[1] + weight_list[i]*error_detect_list_y[i];//y轴
                    error_out[2] = error_out[2] + weight_list[i]*error_detect_list_z[i];//z轴
            }
           
        }
        else if(flag == 1){//非视觉,pz=0,输出小车里程计话题接收的数据
            pos_out[0] = car_odom_pos(0);
            pos_out[1] = car_odom_pos(1);
            pos_out[2] = 0;

            error_out <<0,0,0;

        }
        else if(flag == 2){//刚进入视觉
            //初始化
            Ekf::x_x << car_odom_pos(0),car_odom_vel(0);    
            Ekf::x_y << car_odom_pos(1),car_odom_vel(1);        
            Ekf::x_z << car_odom_pos(2),car_odom_vel(2);  
            Ekf::P_x << 1000, 0,  0,1000;  Ekf::P_y << 1000, 0,  0,1000;  Ekf::P_z << 1000, 0,  0,1000;

            flag = 0;

            Ekf::x_x_old = Ekf::x_x;
            Ekf::x_y_old = Ekf::x_y;
            Ekf::x_z_old = Ekf::x_z;
        }

        odom_sub_tri = false;
    }
    

    

    /************************************输出*********************************/
        geometry_msgs::PoseWithCovariance ekf_pos;
        ekf_pos.pose.position.x = pos_out[0];//x轴KF最佳估计值
        ekf_pos.pose.position.y = pos_out[1];
        ekf_pos.pose.position.z = pos_out[2];     
    
        ekf_pos.covariance[0] = error_out[0];
        ekf_pos.covariance[1] = error_out[1];
        ekf_pos.covariance[2] = error_out[2];

        ekf_pub.publish(ekf_pos);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ekf_test");
    ros::NodeHandle nh("~");

    ekf_pub = nh.advertise<geometry_msgs::PoseWithCovariance>("/pose_ekf", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom/remap/car", 1, &car_odom_Callback, ros::TransportHints().tcpNoDelay()); // 小车里程计话题，local坐标系
    

    
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
    pos_out <<0,0,0; 


    while (ros::ok())
    {
        ros::spinOnce();
        handler();
        ros::Duration(0.005).sleep();
    }

    return 0;
}




