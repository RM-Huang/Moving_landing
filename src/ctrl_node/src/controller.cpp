#include "controller.hpp"

namespace Controller{
    
    /*************** Velocity_Controller *****************/
    void Velocity_Control::init(ctrl_node::Parameter_t &param){
        param_ = param;
        input.vel_last.setZero();
        input.yaw_last = 0;
    }

    double Velocity_Control::get_vel_err(const Desired_State_t &des, const ctrl_node::Odom_Data_t &odom){
        // Eigen::Vector3d des_v = Kp.asDiagonal() * (des.p - odom.p);
        return 0;
    }

    quadrotor_msgs::Px4ctrlDebug Velocity_Control::calculateControl(const Desired_State_t &des, 
                                                                    const ctrl_node::Odom_Data_t &odom, 
                                                                    Controller_Output_t &u){
        // 
        Eigen::Vector3d Kp(param_.gain.vel.Kvp0, param_.gain.vel.Kvp1, param_.gain.vel.Kvp2);
        Eigen::Vector3d Kd(param_.gain.vel.Kvd0, param_.gain.vel.Kvd1, param_.gain.vel.Kvd2);
        Eigen::Vector3d vel_max(param_.kine_cons.vel_hor_max,param_.kine_cons.vel_hor_max,param_.kine_cons.vel_ver_max);
        Eigen::Vector3d acc_max(param_.kine_cons.acc_hor_max, param_.kine_cons.acc_hor_max, param_.kine_cons.acc_ver_max);
        
        //PD control
        Eigen::Vector3d pos_err(des.p - odom.p);
        u.velocity = Kp.asDiagonal() * pos_err + Kd.asDiagonal()*(pos_err - odom_last_.p);
        odom_last_.p = pos_err;

        //limit vel
        u.velocity = u.velocity.cwiseMax(-vel_max).cwiseMin(vel_max);

        //limit acc
        Eigen::Vector3d vel_err(u.velocity-input.vel_last);
        vel_err = vel_err.cwiseMax(-acc_max).cwiseMin(acc_max);
        u.velocity = input.vel_last + vel_err;
        input.vel_last = u.velocity;
        
        //limit yaw
        // double odomYaw = q2yaw(odom.q);
        double odomYaw = q2yaw(odom.q);
        double omega_err(des.yaw - input.yaw_last);
        omega_err = omega_err > param_.kine_cons.omega_yaw_max?param_.kine_cons.omega_yaw_max:omega_err;
        omega_err = omega_err < -param_.kine_cons.omega_yaw_max?-param_.kine_cons.omega_yaw_max:omega_err;
        u.yaw =  input.yaw_last + omega_err;
        input.yaw_last = u.yaw;

        //debug
        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);
        
        debug_msg_.des_v_x = (des.p - odom.p)(0);
        debug_msg_.des_v_y = (des.p - odom.p)(1);
        debug_msg_.des_v_z = (des.p - odom.p)(2);

        debug_msg_.cmd_v_x = u.velocity(0);
        debug_msg_.cmd_v_y = u.velocity(1);
        debug_msg_.cmd_v_z = u.velocity(2);
        
        debug_msg_.des_yaw = u.yaw;

        return debug_msg_;
    }

    /*************** Position_Controller *****************/
    void Position_Control::init(ctrl_node::Parameter_t &param){
        param_ = param;
    }

    quadrotor_msgs::Px4ctrlDebug Position_Control::calculateControl(const Desired_State_t &des, 
                                                                    const ctrl_node::Odom_Data_t &odom, 
                                                                    Controller_Output_t &u){
    // 
        quadrotor_msgs::Px4ctrlDebug data;
        return data;
    }

    /*************** Attitude_Controller *****************/
    void Attitude_Angular_Control::init(ctrl_node::Parameter_t &param){
        param_ = param;
        resetThrustMapping();
    }

    quadrotor_msgs::Px4ctrlDebug Attitude_Angular_Control::calculateControl(const Desired_State_t &des,
                                                                const ctrl_node::Odom_Data_t &odom,
                                                                const ctrl_node::Imu_Data_t &imu, 
                                                                Controller_Output_t &u){
        /* WRITE YOUR CODE HERE */
            //compute disired acceleration
            Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
            Eigen::Vector3d Kp,Kv,Trou;
            Kp << param_.gain.att_pid.Kp0, param_.gain.att_pid.Kp1, param_.gain.att_pid.Kp2;
            Kv << param_.gain.att_pid.Kv0, param_.gain.att_pid.Kv1, param_.gain.att_pid.Kv2;

            des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
            
            // des_acc = des.a + Kp.asDiagonal() * (des.p - odom.p);
            des_acc += Eigen::Vector3d(0,0,param_.gra);

            u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
            double roll,pitch,yaw,yaw_imu;
            double yaw_odom = q2yaw(odom.q);
            double sin = std::sin(yaw_odom);
            double cos = std::cos(yaw_odom);
            roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
            pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
            yaw_imu = q2yaw(imu.q);
            // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
            //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
            //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
            Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
            u.q = imu.q * odom.q.inverse() * q;

        /* WRITE YOUR CODE HERE */

        //used for debug
        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);
        
        debug_msg_.des_v_x = des.v(0);
        debug_msg_.des_v_y = des.v(1);
        debug_msg_.des_v_z = des.v(2);
        
        debug_msg_.des_a_x = des_acc(0);
        debug_msg_.des_a_y = des_acc(1);
        debug_msg_.des_a_z = des_acc(2);
        
        debug_msg_.des_q_x = u.q.x();
        debug_msg_.des_q_y = u.q.y();
        debug_msg_.des_q_z = u.q.z();
        debug_msg_.des_q_w = u.q.w();
        
        debug_msg_.des_thr = u.thrust;
        
        // Used for thrust-accel mapping estimation
        timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
        while (timed_thrust_.size() > 100)
        {
            timed_thrust_.pop();
        }
        return debug_msg_;
    }

    quadrotor_msgs::Px4ctrlDebug Attitude_Angular_Control::calculateControlCMD(const Desired_State_t &des,
                                                                    const ctrl_node::Odom_Data_t &odom,
                                                                    const ctrl_node::Imu_Data_t &imu, 
                                                                    Controller_Output_t &u,
                                                                    ros::Time &t){   
        /* Obtain initial state */ 
            if(is_first_in_control){
                last_time = t;
                init_t = t;
                init_state = odom.v;
            }
        /* WRITE YOUR CODE HERE */
            //compute disired acceleration
            Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
            // 位置环
            Eigen::Vector3d Kp, Kv;
            Eigen::Vector3d Trou; // UDE参数
            Eigen::Vector3d Trou_min, Trou_max; // TVUDE参数
            double t_min, t_max;// TVUDE参数
            Eigen::Vector3d Omg_eso_pos;// ESO参数
            Eigen::Vector3d Beta_x, Beta_y, Beta_z; // ESO参数

            // 姿态环
            // Eigen::Vector3d Katt;
            // Eigen::Vector3d des_omg(0.0, 0.0, 0.0);
            
            Kp << param_.gain.att_pid.Kp0, param_.gain.att_pid.Kp1, param_.gain.att_pid.Kp2;
            Kv << param_.gain.att_pid.Kv0, param_.gain.att_pid.Kv1, param_.gain.att_pid.Kv2;
            Trou << param_.gain.att_ude.Trou0, param_.gain.att_ude.Trou1, param_.gain.att_ude.Trou2;
            Trou_min << param_.gain.att_tvude.Trou_min0, param_.gain.att_tvude.Trou_min1, param_.gain.att_tvude.Trou_min2;
            Trou_max << param_.gain.att_tvude.Trou_max0, param_.gain.att_tvude.Trou_max1, param_.gain.att_tvude.Trou_max2;
            Omg_eso_pos << param_.gain.att_eso.Omg_eso_pos0, param_.gain.att_eso.Omg_eso_pos1, param_.gain.att_eso.Omg_eso_pos2;

            Beta_x << 2 * Omg_eso_pos(0), 2 * Omg_eso_pos(0) * Omg_eso_pos(0), Omg_eso_pos(0) * Omg_eso_pos(0) * Omg_eso_pos(0);
            Beta_y << 2 * Omg_eso_pos(1), 2 * Omg_eso_pos(1) * Omg_eso_pos(1), Omg_eso_pos(1) * Omg_eso_pos(1) * Omg_eso_pos(1);
            Beta_z << 2 * Omg_eso_pos(2), 2 * Omg_eso_pos(2) * Omg_eso_pos(2), Omg_eso_pos(2) * Omg_eso_pos(2) * Omg_eso_pos(2);
            
            t_min = param_.gain.att_tvude.t_min;
            t_max = param_.gain.att_tvude.t_max;

            // Katt << param_.gain.Katt0, param_.gain.Katt1, param_.gain.Katt2; 


            //常值干扰
            Eigen::Vector3d constd(0.35, 0.35, 0.35);
            des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);

            Eigen::Vector3d d_acc = Eigen::Vector3d(0.0, 0.0, 0.0);

            //0:ude 1:TVUDE 2:ESO
            if(param_.estimator_type == 1){
                if(is_first_in_control){
                    u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_control = false;
                }else{
                
                    Eigen::Vector3d T = gettimevaryingT(t.toSec()-init_t.toSec(), t_min, t_max, Trou_min, Trou_max);
                    Eigen::Vector3d T_dot = gettimevaryingTdot(t.toSec()-init_t.toSec(), t_min, t_max, Trou_min, Trou_max);
                    std::cout<<"T is "<<T.transpose()<<std::endl;
                    std::cout<<"T_dot is "<<T_dot.transpose()<<std::endl;
                    // 1
                    d_acc(0) = odom.v(0) / T(0);
                    d_acc(1) = odom.v(1) / T(1);
                    d_acc(2) = odom.v(2) / T(2);
                    // 2
                    Eigen::Vector3d T_init = gettimevaryingT(0, t_min, t_max, Trou_min, Trou_max);
                    d_acc(0) -= init_state(0) / T_init(0);
                    d_acc(1) -= init_state(1) / T_init(1);
                    d_acc(2) -= init_state(2) / T_init(2);
                    // 3
                    //TODO 应该是1/T的导数而不是1/(T的导数)
                    u0_integral_pos(0) +=  (odom.v(0) * T_dot(0) + des_acc(0) / T(0)) * (t.toSec() - last_time.toSec());
                    u0_integral_pos(1) +=  (odom.v(1) * T_dot(1) + des_acc(1) / T(1)) * (t.toSec() - last_time.toSec());
                    u0_integral_pos(2) +=  (odom.v(2) * T_dot(2) + des_acc(2) / T(2)) * (t.toSec() - last_time.toSec());
                    d_acc -= u0_integral_pos; 
                    last_time = t;
                }
            }
            else if(param_.estimator_type == 0)
            {
                // std::cout<<"use ude"<<std::endl;
                if(is_first_in_control){
                    u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_control = false;
                }else{
                    u0_integral_pos += des_acc * (t.toSec() - last_time.toSec());
                    d_acc = odom.v - init_state - u0_integral_pos;
                    // d_acc = odom.v - u0_integral;
                    d_acc(0) = d_acc(0) / Trou(0);
                    d_acc(1) = d_acc(1) / Trou(1);
                    d_acc(2) = d_acc(2) / Trou(2); 
                    last_time = t;
                }
            }
            else if(param_.estimator_type == 2)
            {
                std::cout<<"use eso"<<std::endl;
                if(is_first_in_control){
                    //初始化估计状态
                    pos_x_est = Eigen::Vector3d(0.0, 0.0, 0.0);
                    pos_y_est = Eigen::Vector3d(0.0, 0.0, 0.0);
                    pos_z_est = Eigen::Vector3d(0.0, 0.0, 0.0);
                    pos_x_est_dot = Eigen::Vector3d(0.0, 0.0, 0.0);
                    pos_y_est_dot = Eigen::Vector3d(0.0, 0.0, 0.0);
                    pos_z_est_dot = Eigen::Vector3d(0.0, 0.0, 0.0);

                    is_first_in_control = false;
                }
                else{
                    double step = t.toSec() - last_time.toSec();
                    d_acc(0) = getdisturbfromESO(Beta_x, pos_x_est, pos_x_est_dot, step, des_acc(0), odom.p(0));
                    d_acc(1) = getdisturbfromESO(Beta_y, pos_y_est, pos_y_est_dot, step, des_acc(1), odom.p(1));
                    d_acc(2) = getdisturbfromESO(Beta_z, pos_z_est, pos_z_est_dot, step, des_acc(2), odom.p(2));
                    last_time = t;
                }
            }

            des_acc -= d_acc;
            
            // des_acc = des.a + Kp.asDiagonal() * (des.p - odom.p);
            des_acc += Eigen::Vector3d(0,0,param_.gra);

            u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
            double roll,pitch,yaw,yaw_imu;
            double yaw_odom = q2yaw(odom.q);
            double sin = std::sin(yaw_odom);
            double cos = std::cos(yaw_odom);
            roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
            pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
            // yaw = fromQuaternion2yaw(des.q);
            yaw_imu = q2yaw(imu.q);
            // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
            //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
            //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
            Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
            u.q = imu.q * odom.q.inverse() * q;


        /* WRITE YOUR CODE HERE */

        //used for debug
        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);
        
        debug_msg_.des_v_x = des.v(0);
        debug_msg_.des_v_y = des.v(1);
        debug_msg_.des_v_z = des.v(2);
        
        debug_msg_.des_a_x = des_acc(0);
        debug_msg_.des_a_y = des_acc(1);
        debug_msg_.des_a_z = des_acc(2);
        
        debug_msg_.des_q_x = u.q.x();
        debug_msg_.des_q_y = u.q.y();
        debug_msg_.des_q_z = u.q.z();
        debug_msg_.des_q_w = u.q.w();
        
        debug_msg_.des_thr = u.thrust;
        
        // Used for thrust-accel mapping estimation
        timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
        while (timed_thrust_.size() > 100)
        {
            timed_thrust_.pop();
        }
        return debug_msg_;
    }

    Eigen::Vector3d Attitude_Angular_Control::gettimevaryingT(double const t,
                                                    double const t_min,
                                                    double const t_max,
                                                    Eigen::Vector3d const Trou_min,
                                                    Eigen::Vector3d const Trou_max){
        Eigen::Vector3d T;
        if(t <= t_min){
            T = Trou_max;
            return T;
        }
        if(t > t_max){
            T = Trou_min;
            return T;
        }

        T = (Trou_max - Trou_min) / 2 * cos(M_PI * (t - t_min) / (t_max - t_min)) + (Trou_max + Trou_min) / 2;
        return T;
    }

    Eigen::Vector3d Attitude_Angular_Control::gettimevaryingTdot(double const t,
                                                    double const t_min,
                                                    double const t_max,
                                                    Eigen::Vector3d const Trou_min,
                                                    Eigen::Vector3d const Trou_max)
    {
        Eigen::Vector3d T,c;
        if(t <= t_min || t > t_max){
            T = Eigen::Vector3d(0,0,0);
            return T;
        }
        T = (Trou_max - Trou_min) / 2 * sin(M_PI * (t - t_min) / (t_max - t_min)) * M_PI / (t_max - t_min);
        c = (Trou_max - Trou_min) / 2 * cos(M_PI * (t - t_min) / (t_max - t_min)) + (Trou_max + Trou_min) / 2;
        T(0) /= pow(c(0),2);
        T(1) /= pow(c(1),2);
        T(2) /= pow(c(2),2);
        return T;
    }

    double Attitude_Angular_Control::getdisturbfromESO(Eigen::Vector3d parameter,
                                            Eigen::Vector3d &x_est,
                                            Eigen::Vector3d &x_est_dot,
                                            double step,
                                            double u_0,
                                            double x){
        x_est_dot(2) = - parameter(2) * (x_est(0) - x);
        x_est(2) += x_est_dot(2) * step;
        x_est_dot(1) = x_est(2) + u_0 - parameter(1) * (x_est(0) - x);
        x_est(1) += x_est_dot(1) * step;
        x_est_dot(0) = x_est(1) - parameter(0) * (x_est(0) - x);
        x_est(0) += x_est_dot(0) * step;
        return x_est(2);
    }

    /*
        compute throttle percentage 
    */
    double Attitude_Angular_Control::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc){
        double throttle_percentage(0.0);
        
        /* compute throttle, thr2acc has been estimated before */
        throttle_percentage = des_acc(2) / thr2acc_;

        return throttle_percentage;
    }

    bool Attitude_Angular_Control::estimateThrustModel(const Eigen::Vector3d &est_a, const ctrl_node::Parameter_t &param){
        ros::Time t_now = ros::Time::now();
        while (timed_thrust_.size() >= 1)
        {
            // Choose data before 35~45ms ago
            std::pair<ros::Time, double> t_t = timed_thrust_.front();
            double time_passed = (t_now - t_t.first).toSec();
            if (time_passed > 0.045) // 45ms
            {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
            }
            if (time_passed < 0.035) // 35ms
            {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
            }

            /***********************************************************/
            /* Recursive least squares algorithm with vanishing memory */
            /***********************************************************/
            double thr = t_t.second;
            timed_thrust_.pop();
            
            /***********************************/
            /* Model: est_a(2) = thr1acc_ * thr */
            /***********************************/
            double gamma = 1 / (rho2_ + thr * P_ * thr);
            double K = gamma * P_ * thr;
            thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
            P_ = (1 - K * thr) * P_ / rho2_;
            //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
            //fflush(stdout);

            debug_msg_.thr2acc = thr2acc_;
            return true;
        }
        return false;
    }

    void Attitude_Angular_Control::resetThrustMapping(){
        thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
        P_ = 1e6;
    }
}