#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <osqp.h>
#include <ekf_test/qp_solver.hpp>
#include <iostream>
#include <vector>
#include <cmath>

namespace Ekf {
class CTRV{
    private:
    QP_Solver::Update_vec qp_solver;    

    public:
     //状态矩阵(px,py,pz,v_hor,v_ver,theta,delta_the)
    Eigen::VectorXd x = Eigen::VectorXd::Zero(7);
 
    //初始化不确定性协方差矩阵
    Eigen::Matrix<double,7,7> P;

    //H
    Eigen::Matrix<double,6,7> H;

    //F
    Eigen::Matrix<double,7,7> F;

    //测量协方差矩阵
    Eigen::Matrix<double,6,6> R;
    Eigen::Matrix<double,7,7> I;
    //过程协方差矩阵
    Eigen::Matrix<double,7,7> Q;
    Eigen::Matrix<double,3,3> E;

    //最小二乘法求加速度
    Eigen::MatrixXd P_lq;
    Eigen::MatrixXd A_lq;
    Eigen::VectorXd l_lq;
    Eigen::VectorXd u_lq;
    int var_num; // indicating the dimensions of acc
    int cons_num; // indicating the dimensions of acc constrain

    //估计的加速度和速度差分队列，不参与ekf过程
    Eigen::VectorXd acc;
    std::vector<Eigen::VectorXd> acc_raw_list;
    Eigen::VectorXd a_sum;

    //状态量
    // double& p_x, p_y, p_z;
    // double& v_hor, v_ver;
    // double& theta, delta_the;
    double& p_x = x(0);
    double& p_y = x(1);
    double& p_z = x(2);
    double& v_hor = x(3);
    double& v_ver = x(4);
    double& theta = x(5);
    double& delta_the = x(6);

    //设定帧数, predict_list存放x队列
    int _MAX_SEG = 50;// x/0.005
    std::vector<Eigen::VectorXd> predict_list;  

    int init(int max_seg, double t_, const double& e_ah_, const double& e_av_, const double& e_ddtheta_, const Eigen::VectorXd& e_measure_);

    int reset(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const double &theta);

    void updateF();
    Eigen::MatrixXd updateQ();
    Eigen::VectorXd predictX();

    void estimate_err();

    void estimate_acc(Eigen::VectorXd& v_last);

    void update(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const double &the);
};

class LinearCV{
    public:
    //状态矩阵(位置，速度)
    Eigen::Vector2d x_x; 
    Eigen::Vector2d x_x_;
    
    Eigen::Vector2d x_y; 
    Eigen::Vector2d x_y_;

    Eigen::Vector2d x_z; 
    Eigen::Vector2d x_z_;
 
    //初始化不确定性协方差矩阵，位置(0,0)的不确定性为1000，速度的不确定性为1000
    Eigen::MatrixXd P_x = Eigen::MatrixXd::Zero(2,2);
    Eigen::MatrixXd P_x_ = Eigen::MatrixXd::Zero(2,2);
    Eigen::MatrixXd P_y = Eigen::MatrixXd::Zero(2,2);
    Eigen::MatrixXd P_y_ = Eigen::MatrixXd::Zero(2,2);
    Eigen::MatrixXd P_z = Eigen::MatrixXd::Zero(2,2);
    Eigen::MatrixXd P_z_ = Eigen::MatrixXd::Zero(2,2);

    //状态转移矩阵
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(2,2);
    // 测量矩阵
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,2); 
    //测量协方差矩阵
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2,2);
    Eigen::Matrix2d I = Eigen::MatrixXd::Identity(2,2);
    //过程协方差矩阵
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2,2);

    //变量定义
    Eigen::Vector2d err_x;
    Eigen::Vector2d err_y;
    Eigen::Vector2d err_z;
    Eigen::Vector2d x_x_old;
    Eigen::Vector2d x_y_old;
    Eigen::Vector2d x_z_old;

    //误差输出-设定帧数
    int _MAX_SEG = 50;// x/0.005
    std::vector<double> error_detect_list;  

    void init(int max_seg, double t_);

    void filter_calibrate(double &car_odom_pos_x, double &car_odom_pos_y,double &car_odom_pos_z,
                double &car_odom_vel_x, double &car_odom_vel_y,double &car_odom_vel_z);

    std::vector<double>list_cb(double &state_error);
};

}