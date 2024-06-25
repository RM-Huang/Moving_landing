#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

using std::cout;
using std::endl;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace Ekf {
    
    //状态矩阵(位置，速度)
    Eigen::Vector2d x_x; 
    Eigen::Vector2d x_x_;
    
    Eigen::Vector2d x_y; 
    Eigen::Vector2d x_y_;

    Eigen::Vector2d x_z; 
    Eigen::Vector2d x_z_;
 
    //初始化不确定性协方差矩阵，位置(0,0)的不确定性为1000，速度的不确定性为1000
    Eigen::MatrixXd P_x(2,2);
    Eigen::MatrixXd P_x_(2,2);
    Eigen::MatrixXd P_y(2,2);
    Eigen::MatrixXd P_y_(2,2);
    Eigen::MatrixXd P_z(2,2);
    Eigen::MatrixXd P_z_(2,2);

    //状态转移矩阵
    Eigen::MatrixXd F(2,2);
    // 测量矩阵
    Eigen::MatrixXd H(2,2); 
    //测量协方差矩阵
    Eigen::MatrixXd R(2,2);
    Eigen::Matrix2d I = MatrixXd::Identity(2,2);
    //过程协方差矩阵
    Eigen::MatrixXd Q(2,2);

    //变量定义
    Eigen::Vector2d err_x;
    Eigen::Vector2d err_y;
    Eigen::Vector2d err_z;
    Eigen::Vector2d x_x_old;
    Eigen::Vector2d x_y_old;
    Eigen::Vector2d x_z_old;

    //误差输出-设定帧数
    int _MAX_SEG = 50;// x/0.005
    std::vector<double> error_detect_list(_MAX_SEG);  


    void filter_calibrate(double &car_odom_pos_x, double &car_odom_pos_y,double &car_odom_pos_z,
                double &car_odom_vel_x, double &car_odom_vel_y,double &car_odom_vel_z) {
 
        /********************* x轴 *************************/
        /* 1.预测 */
        // STEP1
        x_x_ = F * x_x ;
      
        // STEP2
        MatrixXd Ft = F.transpose();
        P_x_ = F * P_x * Ft + Q;

        /* 2.校正 */
        // STEP3 Kk=(PkHt)/(HPkHt+R) 
        MatrixXd Ht = H.transpose();
        MatrixXd S = H * P_x_ * Ht + R;
        MatrixXd Si = S.inverse();
        MatrixXd K_x = P_x_ * Ht * Si;

        // STEP4 
        Eigen::Vector2d z_x(car_odom_pos_x,car_odom_vel_x);
        VectorXd y_x = z_x - H * x_x_;
        x_x = x_x_ + (K_x * y_x);

        err_x << 0,0;
        err_x[0] =fabs(x_x[0] - x_x_old[0]) ;
        x_x_old = x_x; //储存估计值

        // STEP5 更新协方差矩阵  
        P_x = (I - K_x * H) * P_x_;


        /********************* y轴 *************************/
        /* 1.预测 */
        // STEP1
        x_y_ = F * x_y ;
      
        // STEP2
        Ft = F.transpose();
        P_y_ = F * P_y * Ft + Q;

        /* 2.校正 */
        // STEP3 
        Ht = H.transpose();
        S = H * P_y_ * Ht + R;
        Si = S.inverse();
        MatrixXd K_y = P_y_ * Ht * Si;

        // STEP4 
        Eigen::Vector2d z_y(car_odom_pos_y,car_odom_vel_y);
        VectorXd y_y = z_y - H * x_y_;
        x_y = x_y_ + (K_y * y_y);

        err_y << 0,0;
        err_y[0] = fabs(x_y[0] - x_y_old[0]);
        x_y_old = x_y; //储存估计值

        // STEP5 更新协方差矩阵  
        P_y = (I - K_y * H) * P_y_;
    
        
        /********************* z轴 *************************/
        /* 1.预测 */
        // STEP1
        x_z_ = F * x_z ;
      
        // STEP2
        Ft = F.transpose();
        P_z_ = F * P_z * Ft + Q;
        // cout << "P_z_=" << endl  << P_z_ << endl;


        /* 2.校正 */
        // STEP3 
        Ht = H.transpose();
        S = H * P_z_ * Ht + R;
        Si = S.inverse();
        MatrixXd K_z = P_z_ * Ht * Si;
        // cout << "K_z=" << endl  << K_z << endl;

        // STEP4 
        Eigen::Vector2d z_z(car_odom_pos_z,car_odom_vel_z);
        VectorXd y_z = z_z - H * x_z_;
        x_z = x_z_ + (K_z * y_z);

        err_z << 0,0;
        err_z[0] = fabs(x_z[0] - x_z_old[0]);
        x_z_old = x_z; //储存估计值

        // STEP5 更新协方差矩阵  
        P_z = (I - K_z * H) * P_z_;
        

    }


    std::vector<double>list_cb(double &state_error){
      
        error_detect_list.erase(error_detect_list.begin());
        error_detect_list.push_back(state_error);        
        return error_detect_list;
        
    }

}