#include <ekf_test/ekf.hpp>

namespace Ekf{
    static double dt = 0.005; // 时间间隔

    Eigen::MatrixXd CTRV::updateQ(){ // Q:7*7
        // Q = G * E * G^T
        Eigen::Matrix<double,7,3> G = Eigen::Matrix<double,7,3>::Zero();
        G(0,0) = dt * dt * cos(theta) / 2;
        G(1,0) = dt * dt * sin(theta) / 2;
        G(2,1) = dt * dt / 2;
        G(3,0) = dt;
        G(4,1) = dt;
        G(5,2) = dt * dt / 2;
        G(6,2) = dt;

        return G * E * G.transpose();
    }

    void CTRV::updateF(){
        // const double& p_x = x(0), p_y = x(1), p_z = x(2);
        // const double& v_hor = x(3), v_ver = x(4);
        // const double& theta = x(5), delta_the = x(6);
        
        F = Eigen::Matrix<double,7,7>::Identity();
        if(delta_the == 0){
            F(0,3) = cos(theta) * dt;
            F(0,5) = -v_hor * sin(theta) * dt;
            F(1,3) = sin(theta) * dt;
            F(1,5) = v_hor * cos(theta) * dt;
            F(2,4) = dt;
            F(5,6) = dt;
        }else{
            F(0,3) = (sin(delta_the * dt + theta) - sin(theta)) / delta_the;
            F(0,5) = (v_hor * (cos(dt * delta_the + theta) - cos(theta))) / delta_the;
            F(0,6) = (dt * cos(dt * delta_the + theta) - (sin(dt * delta_the + theta) - sin(theta)) / delta_the) * v_hor / delta_the;
            F(1,3) = (v_hor * (sin(dt * delta_the + theta) - sin(theta))) / delta_the;
            F(1,5) = (- cos(delta_the * dt + theta) + cos(theta)) / delta_the;
            F(1,6) = (dt * sin(dt * delta_the + theta) - (- cos(dt * delta_the + theta) + cos(theta)) / delta_the) * v_hor / delta_the;
            F(2,4) = dt;
            F(5,6) = dt;
        }
    }

    Eigen::VectorXd CTRV::predictX(){
        Eigen::VectorXd x_ = Eigen::VectorXd::Zero(7);
        
        if(delta_the == 0){
            x_(0) = v_hor * cos(theta) * dt + p_x;
            x_(1) = v_hor * sin(theta) * dt + p_y;
            x_(2) = v_ver * dt + p_z;
            x_(3) = v_hor;
            x_(4) = v_ver;
            x_(5) = theta;
            x_(6) = delta_the;
        }else{
            x_(0) = (sin(delta_the * dt + theta) - sin(theta)) * v_hor / delta_the + p_x;
            x_(1) = (- cos(delta_the * dt + theta) + cos(theta)) * v_hor / delta_the + p_y;
            x_(2) = v_ver * dt + p_z;
            x_(3) = v_hor;
            x_(4) = v_ver;
            x_(5) = delta_the * dt + theta;
            x_(6) = delta_the;
        }
        return x_;
    }

    void CTRV::estimate_err(){

    }

    void CTRV::estimate_acc(Eigen::VectorXd& v_last){
        v_last(0) = (v_hor - v_last(0)) / 0.005;
        v_last(1) = (v_ver - v_last(1)) / 0.005;
        a_sum = a_sum - acc_raw_list[0];
        a_sum = a_sum + v_last;
        acc_raw_list.erase(acc_raw_list.begin());
        acc_raw_list.push_back(v_last);
        int acc_est_flag = qp_solver.Count_x(-a_sum, l_lq, u_lq);
        if(!acc_est_flag){
            for(int i = 0; i < var_num; i++){
                acc[i] = qp_solver.solver->solution->x[i];
            }
        }
    }

    void CTRV::update(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const double &the){
        // predict
        Eigen::VectorXd x_pred = predictX();
        updateF();
        Eigen::MatrixXd Q = updateQ();
        Eigen::MatrixXd P_pred = F * P * F.transpose() + Q; // 7*7

        // update
        Eigen::MatrixXd K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse(); // 7*6
        Eigen::VectorXd z(6);
        z(0) = pos(0);
        z(1) = pos(1);
        z(2) = pos(2);
        z(3) = sqrt(vel(0) * vel(0) + vel(1) * vel(1));
        z(4) = vel(2);
        z(5) = the;
        z = z - H * x_pred;

        if(abs(z(5)) > M_PI){
            double delta_the = abs(z(5)) - M_PI;
            if(z(5) > 0){
                z(5) = -delta_the;
            }else{
                z(5) = delta_the;
            }
        } 

        Eigen::VectorXd v_last(var_num);
        v_last << v_hor, v_ver;

        x = x_pred + K * z;
        if(theta > M_PI){
            theta -= 2 * M_PI;
        }else if(theta < -M_PI){
            theta += 2 * M_PI;
        }
        P = (I - K * H) * P_pred;

        predict_list.erase(predict_list.begin());
        predict_list.push_back(x);

        // estimate_err();
        estimate_acc(v_last);

        //debug
        printf("px:%6.3f, py:%6.3f, pz:%6.3f, vx:%6.3f, vy:%6.3f, vz:%6.3f\r",p_x,p_y,p_z,v_hor*cos(theta),v_hor*sin(theta),v_ver);
        // Eigen::Vector3d ekf_err = ekf_err_list.back();
        // printf("err_px:%6.3f, err_py:%6.3f, err_pz:%6.3f, vx:%6.3f, vy:%6.3f, vz:%6.3f",ekf_err(0),ekf_err(1),ekf_err(2),v_hor*cos(theta),v_hor*sin(theta),v_ver);
        fflush(stdout);
    }

    int CTRV::reset(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const double &theta){
        x = Eigen::VectorXd::Zero(7);
        P = 1000 * Eigen::MatrixXd::Identity(7,7);
        F = Eigen::Matrix<double,7,7>::Identity();

        predict_list.clear();
        predict_list.resize(_MAX_SEG);

        acc_raw_list.clear();
        acc_raw_list.resize(_MAX_SEG, Eigen::VectorXd::Zero(var_num));
        a_sum = Eigen::VectorXd::Zero(var_num);
        int lq_flag = qp_solver.init(var_num, cons_num, P_lq, A_lq, -a_sum, l_lq, u_lq);

        x << pos(0), pos(1), pos(2), sqrt(pow(vel(0),2) + pow(vel(1),2)), vel(2), theta, 0;

        predict_list.push_back(x);

        return lq_flag;
        // acc_raw_list.push_back((Eigen::Vector3d){10,10,10}); // set a huge num
    }

    int CTRV::init(int max_seg, double t_, const double& e_ah_, const double& e_av_, const double& e_ddtheta_, const Eigen::VectorXd& e_measure_){
        dt = t_;
        _MAX_SEG = max_seg;

        /* ekf param init */
        H = Eigen::MatrixXd::Zero(6,7);
        H.block<6,6>(0,0) = Eigen::Matrix<double,6,6>::Identity();

        I = Eigen::MatrixXd::Identity(7,7);
        R = Eigen::MatrixXd::Identity(6,6);
        E = Eigen::MatrixXd::Identity(3,3);
        E(0,0) = e_ah_ * e_ah_;
        E(1,1) = e_av_ * e_av_;
        E(2,2) = e_ddtheta_ * e_ddtheta_;

        for(int i = 0; i < e_measure_.size(); i++){
            R(i,i) = e_measure_(i) * e_measure_(i);
        }

        var_num = 2;
        cons_num = 2;
        acc = Eigen::VectorXd::Zero(var_num);
        P_lq = _MAX_SEG * Eigen::MatrixXd::Identity(var_num, var_num);
        A_lq = Eigen::MatrixXd::Identity(var_num, cons_num);
        l_lq = -5.0 * Eigen::VectorXd::Ones(var_num);
        u_lq = 5.0 * Eigen::VectorXd::Ones(var_num);

        return reset((Eigen::Vector3d){0,0,0}, (Eigen::Vector3d){0,0,0}, 0);
    }

    void LinearCV::init(int max_seg, double t_){
        dt = t_;
        _MAX_SEG = max_seg;
        error_detect_list.resize(_MAX_SEG);

        /* kf param init */
        double ep =  0.7; //位置标准差
        double ev =  7; //速度标准差
        //矩阵初始化
        F << 1, dt, 0, 1;
        H << 1.0, 0.0,  0.0, 1.0;
        R <<(ep*ep), 0,  0, (ev*ev);
        //Q初始化
        Eigen::MatrixXd E_V(2,2);
        E_V << 0 ,0 ,0 , 0.1*0.1;
        Q = (F) * E_V* (F.transpose());
        //状态变量初始化
        x_x_old << 0,0;
        x_y_old << 0,0;
        x_z_old << 0,0;
    }

    void LinearCV::filter_calibrate(double &car_odom_pos_x, double &car_odom_pos_y,double &car_odom_pos_z,
                double &car_odom_vel_x, double &car_odom_vel_y,double &car_odom_vel_z) {
 
        /********************* x轴 *************************/
        /* 1.预测 */
        // STEP1
        x_x_ = F * x_x ;
      
        // STEP2
        Eigen::MatrixXd Ft = F.transpose();
        P_x_ = F * P_x * Ft + Q;

        /* 2.校正 */
        // STEP3 Kk=(PkHt)/(HPkHt+R) 
        Eigen::MatrixXd Ht = H.transpose();
        Eigen::MatrixXd S = H * P_x_ * Ht + R;
        Eigen::MatrixXd Si = S.inverse();
        Eigen::MatrixXd K_x = P_x_ * Ht * Si;

        // STEP4 
        Eigen::Vector2d z_x(car_odom_pos_x,car_odom_vel_x);
        Eigen::VectorXd y_x = z_x - H * x_x_;
        x_x = x_x_ + (K_x * y_x);

        err_x << 0,0;
        // err_x[0] =fabs(x_x[0] - x_x_old[0]) ;
        err_x[0] = fabs((x_x[0] - x_x_old[0]) - x_x[1] * F(0,1));
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
        Eigen::MatrixXd K_y = P_y_ * Ht * Si;

        // STEP4 
        Eigen::Vector2d z_y(car_odom_pos_y,car_odom_vel_y);
        Eigen::VectorXd y_y = z_y - H * x_y_;
        x_y = x_y_ + (K_y * y_y);

        err_y << 0,0;
        // err_y[0] = fabs(x_y[0] - x_y_old[0]);
        err_y[0] = fabs((x_y[0] - x_y_old[0]) - x_y[1] * F(0,1));
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
        Eigen::MatrixXd K_z = P_z_ * Ht * Si;
        // cout << "K_z=" << endl  << K_z << endl;

        // STEP4 
        Eigen::Vector2d z_z(car_odom_pos_z,car_odom_vel_z);
        Eigen::VectorXd y_z = z_z - H * x_z_;
        x_z = x_z_ + (K_z * y_z);

        err_z << 0,0;
        // err_z[0] = fabs(x_z[0] - x_z_old[0]);
        err_z[0] = fabs((x_z[0] - x_z_old[0]) - x_z[1] * F(0,1));
        x_z_old = x_z; //储存估计值

        // STEP5 更新协方差矩阵  
        P_z = (I - K_z * H) * P_z_;
    }

    std::vector<double> LinearCV::list_cb(double &state_error){   
        error_detect_list.erase(error_detect_list.begin());
        error_detect_list.push_back(state_error);        
        return error_detect_list;    
    }
}