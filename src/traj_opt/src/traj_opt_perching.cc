#include <traj_opt/traj_opt.h>

#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt {

static Eigen::Vector3d car_p_, car_v_;
static Eigen::Quaterniond uav_q_, car_q_;
Eigen::Quaterniond land_q;
static Eigen::Vector3d tail_q_v_;
static Eigen::Vector3d g_(0, 0, -9.8);
// static Eigen::Vector3d land_v_;
static Eigen::Vector3d v_t_x_, v_t_y_;
static Trajectory init_traj_;
static double init_tail_f_;
static double traj_tail_alt = 1.0;
static Eigen::Vector2d init_vt_;
static bool initial_guess_ = false;

// static Bezierpredict* bezier_ptr;
static TrajOpt::plan_s* plan_state_;

static double thrust_middle_, thrust_half_;

static double tictoc_innerloop_;
static double tictoc_integral_;

static double v_plus_;

static int iter_times_;

static bool predict_suc = false;

static bool q2v(const Eigen::Quaterniond& q,
                Eigen::Vector3d& v) {
  Eigen::MatrixXd R = q.toRotationMatrix();
  v = R.col(2);
  return true;
}

static Eigen::Vector3d f_N(const Eigen::Vector3d& x) {
  return x.normalized();
}

static Eigen::MatrixXd f_DN(const Eigen::Vector3d& x) {
  double x_norm_2 = x.squaredNorm();
  return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2); // 公式9b
}

static Eigen::MatrixXd f_D2N(const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
  double x_norm_2 = x.squaredNorm();
  double x_norm_3 = x_norm_2 * x.norm();
  Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
  return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
}

// SECTION  variables transformation and gradient transmission
static double smoothedL1(const double& x,
                         double& grad) {
  static double mu = 0.01;
  if (x < 0.0) {
    return 0.0;
  } else if (x > mu) {
    grad = 1.0;
    return x - 0.5 * mu;
  } else {
    const double xdmu = x / mu;
    const double sqrxdmu = xdmu * xdmu;
    const double mumxd2 = mu - 0.5 * x;
    grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
    return mumxd2 * sqrxdmu * xdmu;
  }
}
static double smoothed01(const double& x,
                         double& grad) {
  static double mu = 0.01;
  static double mu4 = mu * mu * mu * mu;
  static double mu4_1 = 1.0 / mu4;
  if (x < -mu) {
    grad = 0;
    return 0;
  } else if (x < 0) {
    double y = x + mu;
    double y2 = y * y;
    grad = y2 * (mu - 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu - x) * mu4_1;
  } else if (x < mu) {
    double y = x - mu;
    double y2 = y * y;
    grad = y2 * (mu + 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu + x) * mu4_1 + 1;
  } else {
    grad = 0;
    return 1;
  }
}

static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}
static inline double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}

static double forward_thrust(const double& f) {
  return thrust_half_ * sin(f) + thrust_middle_;
  // return f;
}
static void addLayerThrust(const double& f,
                           const double& grad_thrust,
                           double& grad_f) {
  grad_f = thrust_half_ * cos(f) * grad_thrust; // 公式22
  // grad_f = grad_thrust;
}
static void forwardTailV(const double& t,
                         const Eigen::Vector3d& car_tail_v_,
                         const Eigen::Ref<const Eigen::Vector2d>& xy,
                         Eigen::Ref<Eigen::Vector3d> tailV) {
  Eigen::Vector3d land_v_ = car_tail_v_ - tail_q_v_ * v_plus_;
  tailV = land_v_ + xy.x() * v_t_x_ + xy.y() * v_t_y_; // 公式19,land_v_ = car_v - tail_q_v_ * v_plus_
}

static void forwardTailQua(const double& t)
{
  // Eigen::Quaterniond land_q = getPredictYQua(vel, t);
  q2v(land_q, tail_q_v_);
  v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 0, 1));
  if (v_t_x_.squaredNorm() == 0) {
    v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 1, 0));
  }
  v_t_x_.normalize();
  v_t_y_ = tail_q_v_.cross(v_t_x_);
  v_t_y_.normalize(); //这一段定义了以tail_q_v_为z轴的坐标系
}

static Eigen::Quaterniond getPredictYQua(const Eigen::Vector3d& vel, const double& t) // vel here is relative to the absolute coordinate
{
  Eigen::Quaterniond orientation;
  if(*plan_state_ == TrajOpt::plan_s::LAND || !predict_suc)
  { 
    orientation = car_q_;
    // Eigen::Matrix3d rx = car_q_.toRotationMatrix();
    // Eigen::Vector3d car_eular = rx.eulerAngles(0,1,2);
    // r = (Eigen::AngleAxisd(car_eular[0],Eigen::Vector3d::UnitX()));
    // p = (Eigen::AngleAxisd(car_eular[1],Eigen::Vector3d::UnitY()));
  }
  else
  {
    double yaw;
    if(vel[0] >= 0 and vel[1] >= 0)
      yaw = std::atan(vel[1] / vel[0]);
    else if(vel[0] >= 0 and vel[1]< 0)
      yaw = 2 * 3.14159265 - std::atan(- vel[1] / vel[0]);
    else if(vel[0] < 0 and vel[1] >= 0)
      yaw = 3.14159265 - std::atan(vel[1] / - vel[0]);
    else if(vel[0] < 0 and vel[1] < 0)
      yaw = std::atan(- vel[1] / - vel[0]) + 3.14159265;
      
    Eigen::AngleAxisd r;
    Eigen::AngleAxisd p;

    Eigen::Matrix3d rx = uav_q_.toRotationMatrix();
    Eigen::Vector3d uav_eular = rx.eulerAngles(0,1,2);
    r = (Eigen::AngleAxisd(uav_eular[0],Eigen::Vector3d::UnitX()));
    p = (Eigen::AngleAxisd(uav_eular[1],Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd y(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
    orientation = r * p * y;
  }

  // std::cout<<"predict_qua = "<<orientation.w()<<" "<<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<std::endl;
  
  return orientation;
}

static void getPVA(const double& t, Eigen::Vector3d& car_p, Eigen::Vector3d& car_v, Eigen::Vector3d& car_a)
{
  // if(predict_suc)
  // {
  //   car_p = bezier_ptr->getPosFromBezier(t,0);
  //   car_v = bezier_ptr->getVelFromBezier(t,0);
  //   car_a = bezier_ptr->getAccFromBezier(t,0);
  // }
  // else
  // {
  car_p = car_p_ + t * car_v_;
  car_p[2] = traj_tail_alt;
  car_v = car_v_;
  car_a = Eigen::Vector3d(0,0,0);
  // }

  // if(*plan_state_ == TrajOpt::plan_s::FOLLOW || *plan_state_ == TrajOpt::plan_s::HOVER)
  // {
  //   car_p[2] = traj_tail_alt;
  // }
}

static void getTailPVAQ(const double& t, Eigen::Vector3d& car_p, Eigen::Vector3d& car_v, Eigen::Vector3d& car_a)
{
  getPVA(t, car_p, car_v, car_a);

  land_q = getPredictYQua(car_v, t);
}

// !SECTION variables transformation and gradient transmission

// SECTION object function
static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  // std::cout << "damn" << std::endl;
  iter_times_++;
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  const double& t = x[0];
  double& gradt = grad[0];
  Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
  Eigen::Map<Eigen::MatrixXd> gradP(grad + obj.dim_t_, 3, obj.dim_p_);
  const double& tail_f = x[obj.dim_t_ + obj.dim_p_ * 3];
  double& grad_f = grad[obj.dim_t_ + obj.dim_p_ * 3];
  Eigen::Map<const Eigen::Vector2d> vt(x + obj.dim_t_ + 3 * obj.dim_p_ + 1);
  Eigen::Map<Eigen::Vector2d> grad_vt(grad + obj.dim_t_ + 3 * obj.dim_p_ + 1);

  double dT = expC2(t);
  double T = obj.N_ * dT;
  Eigen::Vector3d tailV, grad_tailV, tail_p_, tail_v_, car_a_;
  // land_v_ = bezier_ptr->getVelFromBezier(t,0) - tail_q_v_ * v_plus_;
  getTailPVAQ(T, tail_p_, tail_v_, car_a_);
  forwardTailQua(T);
  forwardTailV(T, tail_v_, vt, tailV);

  // std::cout<<"T = "<<T<<" ";
  // std::cout<<"car_v_tail = "<<tail_v_.transpose()<<std::endl;

  Eigen::MatrixXd tailS(3, 4);
  // tailS.col(0) = car_p_ + car_v_ * obj.N_ * dT + tail_q_v_ * obj.robot_l_; // cons 4d
  tailS.col(0) = tail_p_ + tail_q_v_ * obj.robot_l_;
  tailS.col(1) = tailV;
  // tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_; // 公式22
  // tailS.col(1) = vt;
  tailS.col(2).setZero();
  tailS.col(3).setZero();

  auto tic = std::chrono::steady_clock::now();
  obj.mincoOpt_.generate(obj.initS_, tailS, P, dT); // minco生成轨迹

  double cost = obj.mincoOpt_.getTrajSnapCost();
  obj.mincoOpt_.calGrads_CT();

  auto toc = std::chrono::steady_clock::now();
  tictoc_innerloop_ += (toc - tic).count();
  // double cost_with_only_energy = cost;
  // std::cout << "cost of energy: " << cost_with_only_energy << std::endl;

  tic = std::chrono::steady_clock::now();
  obj.addTimeIntPenalty(cost);
  toc = std::chrono::steady_clock::now();
  tictoc_integral_ += (toc - tic).count();

  tic = std::chrono::steady_clock::now();
  obj.mincoOpt_.calGrads_PT();
  toc = std::chrono::steady_clock::now();
  tictoc_innerloop_ += (toc - tic).count();
  // std::cout << "cost of penalty: " << cost - cost_with_only_energy << std::endl;

  obj.mincoOpt_.gdT += obj.mincoOpt_.gdTail.col(0).dot(obj.N_ * car_v_);
  // obj.mincoOpt_.gdT += obj.mincoOpt_.gdTail.col(0).dot(obj.N_ * car_v_ + obj.N_ * car_a_ * dT);
  // obj.mincoOpt_.gdT += obj.mincoOpt_.gdTail.col(1).dot(obj.N_ * car_a_);
  grad_tailV = obj.mincoOpt_.gdTail.col(1);
  // double grad_thrust = obj.mincoOpt_.gdTail.col(2).dot(tail_q_v_);
  // addLayerThrust(tail_f, grad_thrust, grad_f);

  // if(obj.rhoTf_ > -1)
  // {
  //   cost += obj.rhoTf_ * abs(sin(tail_f));
  //   // std::cout<< "Tf_cost = "<<obj.rhoTf_ * abs(sin(tail_f))<<std::endl;
  // }

  if (obj.rhoVt_ > -1) {
    grad_vt.x() = grad_tailV.dot(v_t_x_);
    grad_vt.y() = grad_tailV.dot(v_t_y_);
    double vt_sqr = vt.squaredNorm();
    cost += obj.rhoVt_ * vt_sqr;
    grad_vt += obj.rhoVt_ * 2 * vt;
  }

  obj.mincoOpt_.gdT += obj.rhoT_;
  cost += obj.rhoT_ * dT;
  gradt = obj.mincoOpt_.gdT * gdT2t(t);

  gradP = obj.mincoOpt_.gdP;

  // std::cout<<"cost = "<<cost<<" grad_t = "<<gradt<<" grad_vt = "<<grad_vt.transpose()<<std::endl;;
  return cost;
}

// !SECTION object function
static inline int earlyExit(void* ptrObj,
                            const double* x,
                            const double* grad,
                            const double fx,
                            const double xnorm,
                            const double gnorm,
                            const double step,
                            int n,
                            int k,
                            int ls) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  if (obj.pause_debug_) {
    const double& t = x[0];
    Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
    const double& tail_f = x[obj.dim_t_ + obj.dim_p_ * 3];
    Eigen::Map<const Eigen::Vector2d> vt(x + obj.dim_t_ + 3 * obj.dim_p_ + 1);

    double dT = expC2(t);
    double T = obj.N_ * dT;
    Eigen::Vector3d tailV, tail_p_, tail_v_, car_a_;
    getTailPVAQ(T, tail_p_, tail_v_, car_a_);
    forwardTailQua(T);
    forwardTailV(T, tail_v_, vt, tailV);

    Eigen::MatrixXd tailS(3, 4);
    tailS.col(0) = tail_p_ + tail_q_v_ * obj.robot_l_;
    tailS.col(1) = tailV;
    // tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_;
    tailS.col(2).setZero();
    tailS.col(3).setZero();

    obj.mincoOpt_.generate(obj.initS_, tailS, P, dT);
    auto traj = obj.mincoOpt_.getTraj();
    obj.visPtr_->visualize_traj(traj, "debug_traj");
    std::vector<Eigen::Vector3d> int_waypts;
    for (const auto& piece : traj) {
      const auto& dur = piece.getDuration();
      for (int i = 0; i < obj.K_; ++i) {
        double t = dur * i / obj.K_;
        int_waypts.push_back(piece.getPos(t));
      }
    }
    obj.visPtr_->visualize_pointcloud(int_waypts, "int_waypts");

    // NOTE pause
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // return k > 1e3;
  return 0;
}

static void bvp(const double& t,
                const Eigen::MatrixXd i_state,
                const Eigen::MatrixXd f_state,
                CoefficientMat& coeffMat) {
  double t1 = t;
  double t2 = t1 * t1;
  double t3 = t2 * t1;
  double t4 = t2 * t2;
  double t5 = t3 * t2;
  double t6 = t3 * t3;
  double t7 = t4 * t3;
  CoefficientMat boundCond; // CoefficientMat为3行8列
  boundCond.leftCols(4) = i_state;
  boundCond.rightCols(4) = f_state;

  coeffMat.col(0) = (boundCond.col(7) / 6.0 + boundCond.col(3) / 6.0) * t3 +
                    (-2.0 * boundCond.col(6) + 2.0 * boundCond.col(2)) * t2 +
                    (10.0 * boundCond.col(5) + 10.0 * boundCond.col(1)) * t1 +
                    (-20.0 * boundCond.col(4) + 20.0 * boundCond.col(0));
  coeffMat.col(1) = (-0.5 * boundCond.col(7) - boundCond.col(3) / 1.5) * t3 +
                    (6.5 * boundCond.col(6) - 7.5 * boundCond.col(2)) * t2 +
                    (-34.0 * boundCond.col(5) - 36.0 * boundCond.col(1)) * t1 +
                    (70.0 * boundCond.col(4) - 70.0 * boundCond.col(0));
  coeffMat.col(2) = (0.5 * boundCond.col(7) + boundCond.col(3)) * t3 +
                    (-7.0 * boundCond.col(6) + 10.0 * boundCond.col(2)) * t2 +
                    (39.0 * boundCond.col(5) + 45.0 * boundCond.col(1)) * t1 +
                    (-84.0 * boundCond.col(4) + 84.0 * boundCond.col(0));
  coeffMat.col(3) = (-boundCond.col(7) / 6.0 - boundCond.col(3) / 1.5) * t3 +
                    (2.5 * boundCond.col(6) - 5.0 * boundCond.col(2)) * t2 +
                    (-15.0 * boundCond.col(5) - 20.0 * boundCond.col(1)) * t1 +
                    (35.0 * boundCond.col(4) - 35.0 * boundCond.col(0));
  coeffMat.col(4) = boundCond.col(3) / 6.0;
  coeffMat.col(5) = boundCond.col(2) / 2.0;
  coeffMat.col(6) = boundCond.col(1);
  coeffMat.col(7) = boundCond.col(0);

  coeffMat.col(0) = coeffMat.col(0) / t7;
  coeffMat.col(1) = coeffMat.col(1) / t6;
  coeffMat.col(2) = coeffMat.col(2) / t5;
  coeffMat.col(3) = coeffMat.col(3) / t4;
}

static double getMaxOmega(Trajectory& traj) {
  double dt = 0.01;
  double max_omega = 0;
  for (double t = 0; t < traj.getTotalDuration(); t += dt) {
    Eigen::Vector3d a = traj.getAcc(t);
    Eigen::Vector3d j = traj.getJer(t);
    Eigen::Vector3d thrust = a - g_;
    Eigen::Vector3d zb_dot = f_DN(thrust) * j; // zb为推力方向向量，公式9b
    double omega12 = zb_dot.norm(); // 角速度前两个分量平方和
    if (omega12 > max_omega) {
      max_omega = omega12;
    }
  }
  return max_omega;
}

static double getMaxVel(Trajectory& traj){
  double dt = 0.01;
  double max_vel = 0;
  for(double t = 0; t < traj.getTotalDuration(); t += dt){
    Eigen::Vector3d v = traj.getVel(t);
    double vel = v.norm();
    if(vel > max_vel){
      max_vel = vel;
    }
  }
  return max_vel;
}

static double getMaxVelZ(Trajectory& traj){
  double dt = 0.01;
  double max_vel_z = 0;
  for(double t = 0; t < traj.getTotalDuration(); t += dt){
    Eigen::Vector3d v = traj.getVel(t);
    double vel_z = abs(v[2]);
    if(vel_z > max_vel_z){
      max_vel_z = vel_z;
    }
  }
  return max_vel_z;
}

bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
                            const Eigen::Vector3d& car_p,
                            const Eigen::Vector3d& car_v,
                            const Eigen::Quaterniond& car_q,
                            const Eigen::Quaterniond& uav_q,
                            const bool& predict_flag,
                            const int& N,
                            Trajectory& traj,
                            plan_s *plan_state)
                            // const double& t_replan) 
{
  predict_suc = predict_flag;
  plan_state_ = plan_state;
  uav_q_ = uav_q;
  car_q_ = car_q;
  // if(predict_suc)
  // {
  //   bezier_ptr = bezier_predict;
  // }
  N_ = N;
  dim_t_ = 1;
  dim_p_ = N_ - 1;
  x_ = new double[dim_t_ + 3 * dim_p_ + 1 + 2];  // 1: tail thrust; 2: tail vt
  double& t = x_[0];
  Eigen::Map<Eigen::MatrixXd> P(x_ + dim_t_, 3, dim_p_); // P为x_的映射矩阵，3行dim_p_列，从第dim_t_组开始取
  double& tail_f = x_[dim_t_ + 3 * dim_p_];
  Eigen::Map<Eigen::Vector2d> vt(x_ + dim_t_ + 3 * dim_p_ + 1);
  car_p_ = car_p;
  car_v_ = car_v;

  // NOTE set boundary conditions
  initS_ = iniState;

  if(*plan_state == FOLLOW || *plan_state_ == HOVER)
  {
    traj_tail_alt = 0.8;
    // bvp_f.col(1) = car_v_ / 2;
  }
  else if(*plan_state == LAND)
  {
    traj_tail_alt = car_p_[2];
  }
  // traj_tail_alt = car_p_[2];
  // else if(*plan_state == LAND && initS_.col(0)[2] - car_p_[2] > 0.5)
  // {
  //   traj_tail_alt = initS_.col(0)[2] - 0.5;
  // }
  // else if(*plan_state == LAND && initS_.col(0)[2] - car_p_[2] <= 0.5)
  // {
  //   traj_tail_alt = car_p_[2];
  // }
  std::cout<< "traj_tail_alt = " << traj_opt::traj_tail_alt <<std::endl;
  // std::cout << "land_q: "
  //           << land_q.w() << ","
  //           << land_q.x() << ","
  //           << land_q.y() << ","
  //           << land_q.z() << "," << std::endl;
  // q2v(land_q, tail_q_v_); // 得到平台机体坐标系z轴在原坐标系中的投影向量
  // // tail_q_v_ << 0,0,1; // 泛函修改，将末端姿态改为定值
  thrust_middle_ = (thrust_max_ + thrust_min_) / 2; // 中位
  thrust_half_ = (thrust_max_ - thrust_min_) / 2; // 半增值

  vt.setConstant(0.0);

  // set initial guess with obvp minimum jerk + rhoT
  mincoOpt_.reset(N_);

  tail_f = 0;

  Eigen::MatrixXd bvp_i = initS_; // 无人机初始状态
  Eigen::MatrixXd bvp_f(3, 4); // 1、降落点位置，2、降落点速度，3、降落点加速度，4、jerk
  bvp_f.col(0) = car_p_;
  bvp_f.col(1) = car_v_;
  bvp_f.col(0)[2] = traj_tail_alt;
  // bvp_f.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_; // 公式22
  bvp_f.col(2).setZero();
  bvp_f.col(3).setZero();
  double T_min = (bvp_f.col(0) - bvp_i.col(0)).norm() / vmax_;
  double T_bvp = T_min; // 得到初始相对距离最小时间
  Eigen::Vector3d tail_p_, tail_v_, car_a_;
  CoefficientMat coeffMat;
  double max_omega = 0;
  // std::cout<<"T , omega = "<<std::endl;
  do {
    if(*plan_state_ == TrajOpt::plan_s::LAND)
      T_bvp += 0.1; // 1.0
    else
      T_bvp += 1.0;
    
    if(T_bvp > 50 * T_min)
    {
      std::cout<<"minumsnap T cost too high"<<" T = "<<T_bvp<<" max_omega = "<<max_omega<<std::endl;
      return false;
    }
    getPVA(T_bvp, tail_p_, tail_v_, car_a_);
    bvp_f.col(0) = tail_p_;
    bvp_f.col(1) = tail_v_;
    bvp(T_bvp, bvp_i, bvp_f, coeffMat); // 得到多项式系数
    std::vector<double> durs{T_bvp};
    std::vector<CoefficientMat> coeffs{coeffMat};
    // std::cout<<"T_bvp = "<<T_bvp<<std::endl;
    Trajectory traj(durs, coeffs); // 保存粗轨迹
    max_omega = getMaxOmega(traj);
    // std::cout<<T_bvp<<" "<<max_omega<<" , ";
  } while (max_omega > 1.5 * omega_max_);
  // std:;cout<<std::endl;
  std::cout<<"T_bvp = "<<T_bvp<<std::endl;

  // auto tic_snap = std::chrono::steady_clock::now();
  // double T_head = T_min;
  // double T_tail = 80 * T_min;
  // double T_tmp = T_head;
  // bool found_res = false;
  // CoefficientMat coeffMat_tmp;
  // do{
  //   getPVA(T_tmp, tail_p_, tail_v_, car_a_);
  //   bvp_f.col(0) = tail_p_;
  //   bvp_f.col(1) = tail_v_;
  //   bvp(T_tmp, bvp_i, bvp_f, coeffMat_tmp); // 得到多项式系数
  //   std::vector<double> durs{T_tmp};
  //   std::vector<CoefficientMat> coeffs{coeffMat_tmp};
  //   // std::cout<<"T_bvp = "<<T_bvp<<std::endl;
  //   Trajectory traj(durs, coeffs); // 保存粗轨迹
  //   max_omega = getMaxOmega(traj);
  //   if(max_omega > 1.5)
  //   {
  //     T_head = T_tmp;
  //   }
  //   else
  //   {
  //     found_res = true;
  //     T_tail = T_tmp;
  //     coeffMat = coeffMat_tmp;
  //   }
  //   T_tmp = (T_head + T_tail) / 2;
  // }while(T_tail - T_head > 0.1);
  // if(!found_res)
  // {
  //   std::cout<<"minumsnap T cost too high"<<" t_tail = "<<T_tmp<<" t_min = "<< T_min<<std::endl;
  //   return false;
  // }
  // T_bvp = T_tail;
  // std::cout<<"T_bvp = "<<T_bvp<<std::endl;
  // auto toc_snap = std::chrono::steady_clock::now();
  // std::cout << "snap costs: " << (toc_snap - tic_snap).count() * 1e-6 << "ms" << std::endl;
  Eigen::VectorXd tt(8);
  tt(7) = 1.0;
  for (int i = 1; i < N_; ++i) {
    double tt0 = (i * 1.0 / N_) * T_bvp;
    for (int j = 6; j >= 0; j -= 1) {
      tt(j) = tt(j + 1) * tt0;
    }
    P.col(i - 1) = coeffMat * tt; // 根据粗轨迹获得N-1个中间点
    // std::cout<<"tt:"<<tt.transpose()<<std::endl;
    // std::cout<<"P:"<<std::endl;
    // std::cout<<P<<std::endl;
  }

  t = logC2(T_bvp / N_); // 为了将T>0约束等式化方便计算cost
  // }
  // std::cout << "initial guess >>> t: " << t << std::endl;
  // std::cout << "initial guess >>> tail_f: " << tail_f << std::endl;
  // std::cout << "initial guess >>> vt: " << vt.transpose() << std::endl;

  // NOTE optimization
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 32;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.min_step = 1e-16;
  lbfgs_params.delta = 1e-4;
  lbfgs_params.line_search_type = 0;
  double minObjective;

  int opt_ret = 0;

  auto tic = std::chrono::steady_clock::now();
  tictoc_innerloop_ = 0;
  tictoc_integral_ = 0;

  iter_times_ = 0;
  opt_ret = lbfgs::lbfgs_optimize(dim_t_ + 3 * dim_p_ + 1 + 2, x_, &minObjective,
                                  &objectiveFunc, nullptr,
                                  &earlyExit, this, &lbfgs_params);

  auto toc = std::chrono::steady_clock::now();

  std::cout << "\033[32m>ret: " << opt_ret << "\033[0m" << std::endl;

  std::cout << "innerloop costs: " << tictoc_innerloop_ * 1e-6 << "ms" << std::endl;
  std::cout << "integral costs: " << tictoc_integral_ * 1e-6 << "ms" << std::endl;
  std::cout << "optmization costs: " << (toc - tic).count() * 1e-6 << "ms" << std::endl;
  // std::cout << "\033[32m>iter times: " << iter_times_ << "\033[0m" << std::endl;
  if (pause_debug_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  if (opt_ret < 0) {
    delete[] x_;
    return false;
  }
  double dT = expC2(t);
  double T = N_ * dT;
  Eigen::Vector3d tailV;
  getTailPVAQ(T, tail_p_, tail_v_, car_a_);
  forwardTailQua(T);
  forwardTailV(T, tail_v_, vt, tailV);
  Eigen::MatrixXd tailS(3, 4);
  // tailS.col(0) = car_p_ + car_v_ * T + tail_q_v_ * robot_l_;
  tailS.col(0) = tail_p_ + tail_q_v_ * robot_l_;
  tailS.col(1) = tailV;
  // tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_;
  tailS.col(2).setZero();
  tailS.col(3).setZero();
  // std::cout << "tail thrust: " << forward_thrust(tail_f) << std::endl;
  std::cout << "tailS : " << std::endl;
  std::cout << tailS << std::endl;
  mincoOpt_.generate(initS_, tailS, P, dT);
  traj = mincoOpt_.getTraj(); //调用MINCO轨迹类生成轨迹

  // std::cout << "tailV: " << tailV.transpose() << std::endl;
  std::cout << "maxOmega: " << getMaxOmega(traj) << std::endl;
  std::cout << "maxThrust: " << traj.getMaxThrust() << std::endl;
  std::cout << "maxVel: " << getMaxVel(traj) << std::endl;
  std::cout << "maxVel_Z:" << getMaxVelZ(traj) << std::endl;

  init_traj_ = traj;
  init_tail_f_ = tail_f;
  init_vt_ = vt;
  initial_guess_ = true;
  delete[] x_;
  return true;
}

// bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
//                             const Eigen::Vector3d& car_p,
//                             const Eigen::Vector3d& car_v,
//                             const Eigen::Quaterniond& land_q,
//                             Bezierpredict *bezier_predict,
//                             const int& N,
//                             Trajectory& traj,
//                             const plan_s& plan_state)
//                             // const double& t_replan) 
// {
//   bezier_ptr = bezier_predict;
//   std::cout<<"bezier_ptr = "<<bezier_ptr<<std::endl;
//   N_ = N;
//   dim_t_ = 1;
//   dim_p_ = N_ - 1;
//   x_ = new double[dim_t_ + 3 * dim_p_ + 1 + 2];  // 1: tail thrust; 2: tail vt
//   double& t = x_[0];
//   Eigen::Map<Eigen::MatrixXd> P(x_ + dim_t_, 3, dim_p_); // P为x_的映射矩阵，3行dim_p_列，从第dim_t_组开始取
//   double& tail_f = x_[dim_t_ + 3 * dim_p_];
//   Eigen::Map<Eigen::Vector2d> vt(x_ + dim_t_ + 3 * dim_p_ + 1);
//   car_p_ = car_p;
//   car_v_ = car_v;
//   if(plan_state == FOLLOW)
//   {
//     car_p_[2] = traj_tail_alt;
//   }
//   // std::cout << "land_q: "
//   //           << land_q.w() << ","
//   //           << land_q.x() << ","
//   //           << land_q.y() << ","
//   //           << land_q.z() << "," << std::endl;
//   q2v(land_q, tail_q_v_); // 得到平台机体坐标系z轴在原坐标系中的投影向量
//   // tail_q_v_ << 0,0,1; // 泛函修改，将末端姿态改为定值
//   thrust_middle_ = (thrust_max_ + thrust_min_) / 2; // 中位
//   thrust_half_ = (thrust_max_ - thrust_min_) / 2; // 半增值

//   land_v_ = car_v - tail_q_v_ * v_plus_; // 在平台运动速度基础上减去降落平面方向上的微小速度
//   // std::cout << "tail_q_v_: " << tail_q_v_.transpose() << std::endl;

//   v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 0, 1));
//   if (v_t_x_.squaredNorm() == 0) {
//     v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 1, 0));
//   }
//   v_t_x_.normalize();
//   v_t_y_ = tail_q_v_.cross(v_t_x_);
//   v_t_y_.normalize(); //这一段定义了以tail_q_v_为z轴的坐标系

//   vt.setConstant(0.0);

//   // NOTE set boundary conditions
//   initS_ = iniState;

//   // set initial guess with obvp minimum jerk + rhoT
//   mincoOpt_.reset(N_);

//   tail_f = 0;

//   // bool opt_once = initial_guess_ && t_replan > 0 && t_replan < init_traj_.getTotalDuration(); // t_replan = -1
//   // if (opt_once) {
//   //   double init_T = init_traj_.getTotalDuration() - t_replan;
//   //   t = logC2(init_T / N_);
//   //   for (int i = 1; i < N_; ++i) {
//   //     double tt0 = (i * 1.0 / N_) * init_T;
//   //     P.col(i - 1) = init_traj_.getPos(tt0 + t_replan);
//   //   }
//   //   tail_f = init_tail_f_;
//   //   vt = init_vt_;
//   // } else {
//   Eigen::MatrixXd bvp_i = initS_; // 无人机初始状态
//   Eigen::MatrixXd bvp_f(3, 4); // 1、降落点位置，2、降落点速度，3、降落点加速度，4、jerk
//   bvp_f.col(0) = car_p_;
//   bvp_f.col(1) = car_v_;
//   // bvp_f.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_; // 公式22
//   bvp_f.col(2).setZero();
//   bvp_f.col(3).setZero();
//   double T_bvp = (bvp_f.col(0) - bvp_i.col(0)).norm() / vmax_; // 得到初始相对距离最小时间
//   CoefficientMat coeffMat;
//   double max_omega = 0;
//   do {
//     T_bvp += 1.0;
//     // bvp_f.col(0) = car_p_ + car_v_ * T_bvp; // 获得n时刻后平台位置
//     bvp_f.col(0) = bezier_ptr->getPosFromBezier(T_bvp,0);
//     bvp_f.col(1) = bezier_ptr->getVelFromBezier(T_bvp,0);
//     bvp(T_bvp, bvp_i, bvp_f, coeffMat); // 得到多项式系数
//     std::vector<double> durs{T_bvp};
//     std::vector<CoefficientMat> coeffs{coeffMat};
//     // std::cout<<"T_bvp = "<<T_bvp<<std::endl;
//     Trajectory traj(durs, coeffs); // 保存粗轨迹
//     max_omega = getMaxOmega(traj);
//   } while (max_omega > 1.5 * omega_max_);
//   Eigen::VectorXd tt(8);
//   tt(7) = 1.0;
//   for (int i = 1; i < N_; ++i) {
//     double tt0 = (i * 1.0 / N_) * T_bvp;
//     for (int j = 6; j >= 0; j -= 1) {
//       tt(j) = tt(j + 1) * tt0;
//     }
//     P.col(i - 1) = coeffMat * tt; // 根据粗轨迹获得N-1个中间点
//     // std::cout<<"tt:"<<tt.transpose()<<std::endl;
//     // std::cout<<"P:"<<std::endl;
//     // std::cout<<P<<std::endl;
//   }
//   t = logC2(T_bvp / N_); // 为了将T>0约束等式化方便计算cost
//   // }
//   // std::cout << "initial guess >>> t: " << t << std::endl;
//   // std::cout << "initial guess >>> tail_f: " << tail_f << std::endl;
//   // std::cout << "initial guess >>> vt: " << vt.transpose() << std::endl;

//   // NOTE optimization
//   lbfgs::lbfgs_parameter_t lbfgs_params;
//   lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
//   lbfgs_params.mem_size = 32;
//   lbfgs_params.past = 3;
//   lbfgs_params.g_epsilon = 0.0;
//   lbfgs_params.min_step = 1e-16;
//   lbfgs_params.delta = 1e-4;
//   lbfgs_params.line_search_type = 0;
//   double minObjective;

//   int opt_ret = 0;

//   auto tic = std::chrono::steady_clock::now();
//   tictoc_innerloop_ = 0;
//   tictoc_integral_ = 0;

//   iter_times_ = 0;
//   opt_ret = lbfgs::lbfgs_optimize(dim_t_ + 3 * dim_p_ + 1 + 2, x_, &minObjective,
//                                   &objectiveFunc, nullptr,
//                                   &earlyExit, this, &lbfgs_params);

//   auto toc = std::chrono::steady_clock::now();

//   std::cout << "\033[32m>ret: " << opt_ret << "\033[0m" << std::endl;

//   // std::cout << "innerloop costs: " << tictoc_innerloop_ * 1e-6 << "ms" << std::endl;
//   // std::cout << "integral costs: " << tictoc_integral_ * 1e-6 << "ms" << std::endl;
//   std::cout << "optmization costs: " << (toc - tic).count() * 1e-6 << "ms" << std::endl;
//   // std::cout << "\033[32m>iter times: " << iter_times_ << "\033[0m" << std::endl;
//   if (pause_debug_) {
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   }
//   if (opt_ret < 0) {
//     delete[] x_;
//     return false;
//   }
//   double dT = expC2(t);
//   double T = N_ * dT;
//   Eigen::Vector3d tailV;
//   forwardTailV(vt, tailV);
//   Eigen::MatrixXd tailS(3, 4);
//   tailS.col(0) = car_p_ + car_v_ * T + tail_q_v_ * robot_l_;
//   tailS.col(1) = tailV;
//   // tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_;
//   tailS.col(2).setZero();
//   tailS.col(3).setZero();
//   // std::cout << "tail thrust: " << forward_thrust(tail_f) << std::endl;
//   std::cout << "tailS : " << std::endl;
//   std::cout << tailS << std::endl;
//   mincoOpt_.generate(initS_, tailS, P, dT);
//   traj = mincoOpt_.getTraj(); //调用MINCO轨迹类生成轨迹

//   // std::cout << "tailV: " << tailV.transpose() << std::endl;
//   std::cout << "maxOmega: " << getMaxOmega(traj) << std::endl;
//   std::cout << "maxThrust: " << traj.getMaxThrust() << std::endl;
//   std::cout << "maxVel: " << getMaxVel(traj) << std::endl;

//   init_traj_ = traj;
//   init_tail_f_ = tail_f;
//   init_vt_ = vt;
//   initial_guess_ = true;
//   delete[] x_;
//   return true;
// }

void TrajOpt::addTimeIntPenalty(double& cost) {
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_tmp2, grad_tmp3, grad_p, grad_v, grad_a, grad_j, grad_car_p;
  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
  double s1, s2, s3, s4, s5, s6, s7;
  double step, alpha;
  Eigen::Matrix<double, 8, 3> gradViola_c;
  double gradViola_t;
  double omg;

  int innerLoop = K_ + 1;
  step = mincoOpt_.t(1) / K_;

  s1 = 0.0;

  for (int j = 0; j < innerLoop; ++j) {
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    s6 = s4 * s2;
    s7 = s4 * s3;
    beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
    beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
    alpha = 1.0 / K_ * j;
    omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;
    for (int i = 0; i < N_; ++i) {
      Eigen::Vector3d car_p, car_v, car_a_;
      double dur2now = (i + alpha) * mincoOpt_.t(1);
      // getPVA(dur2now, car_p, car_v, car_a_);
      car_p = car_p_ + car_v_ * dur2now;

      const auto& c = mincoOpt_.c.block<8, 3>(i * 8, 0); // 第i段多项式系数

      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;
      snp = c.transpose() * beta4;

      grad_p.setZero();
      grad_v.setZero();
      grad_a.setZero();
      grad_j.setZero();
      grad_tmp3.setZero();
      grad_car_p.setZero();
      cost_inner = 0.0;

      if (grad_cost_floor(pos, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_top(pos, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_v(vel, grad_tmp, cost_tmp)) {
        grad_v += grad_tmp;
        cost_inner += cost_tmp;
      }

      if(grad_cost_v_z(vel, grad_tmp, cost_tmp)){
        grad_v += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_thrust(acc, grad_tmp, cost_tmp)) {
        grad_a += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_omega(acc, jer, grad_tmp, grad_tmp2, cost_tmp)) {
        grad_a += grad_tmp;
        grad_j += grad_tmp2;
        cost_inner += cost_tmp;
      }

      // if (grad_cost_omega_yaw(acc, jer, grad_tmp, grad_tmp2, cost_tmp)) {
      //   grad_a += grad_tmp;
      //   grad_j += grad_tmp2;
      //   cost_inner += cost_tmp;
      // }

      // if(*plan_state_ == TrajOpt::plan_s::LAND)
      // {
        // if(grad_cost_visible_domain(pos, car_p, grad_tmp, grad_tmp2, cost_tmp)){
        // grad_p += grad_tmp;
        // grad_car_p += grad_tmp2;
        // cost_inner += cost_tmp;
        // }
        if(grad_cost_visible_domain(pos, acc, car_p,
                                   grad_tmp, grad_tmp2, grad_tmp3,
                                   cost_tmp)){
          grad_p += grad_tmp;
          grad_a += grad_tmp2;
          grad_car_p += grad_tmp3;
          cost_inner += cost_tmp;
          // grad_car_t += grad_tmp3.dot(car_v);
        }
        // std::cout<<" , ";
      // }

      // Eigen::Vector3d car_p = car_p_ + car_v_ * dur2now; // 预测，predict
      if (grad_cost_perching_collision(pos, acc, car_p,
                                       grad_tmp, grad_tmp2, grad_tmp3,
                                       cost_tmp)) { // ??
        grad_p += grad_tmp;
        grad_a += grad_tmp2;
        grad_car_p += grad_tmp3;
        cost_inner += cost_tmp;
        // grad_car_t += grad_tmp3.dot(car_v);
      }

      double grad_car_t = grad_car_p.transpose() * car_v_;

      gradViola_c = beta0 * grad_p.transpose();
      gradViola_t = grad_p.transpose() * vel;
      gradViola_c += beta1 * grad_v.transpose();
      gradViola_t += grad_v.transpose() * acc;
      gradViola_c += beta2 * grad_a.transpose();
      gradViola_t += grad_a.transpose() * jer;
      gradViola_c += beta3 * grad_j.transpose();
      gradViola_t += grad_j.transpose() * snp;
      gradViola_t += grad_car_t;

      mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViola_c;
      mincoOpt_.gdT += omg * (cost_inner / K_ + alpha * step * gradViola_t);
      mincoOpt_.gdT += i * omg * step * grad_car_t;
      cost += omg * step * cost_inner;
    }
    s1 += step;
  }
}

TrajOpt::TrajOpt(ros::NodeHandle& nh) {
  // nh.getParam("N", N_);
  nh.getParam("K", K_);
  // load dynamic paramters
  nh.getParam("vmax", vmax_);
  nh.getParam("amax", amax_);
  nh.getParam("thrust_max", thrust_max_);
  nh.getParam("thrust_min", thrust_min_);
  nh.getParam("omega_max", omega_max_);
  nh.getParam("omega_yaw_max", omega_yaw_max_);
  nh.getParam("v_plus", v_plus_);
  nh.getParam("robot_l", robot_l_);
  nh.getParam("robot_r", robot_r_);
  nh.getParam("platform_r", platform_r_);
  nh.getParam("rhoT", rhoT_);
  nh.getParam("rhoVt", rhoVt_);
  // nh.getParam("rhoTf", rhoTf_);
  nh.getParam("rhoPf", rhoPf_); // force the lowest height of uav
  nh.getParam("rhoPt", rhoPt_);
  nh.getParam("rhoV", rhoV_);
  nh.getParam("rhoVZ", rhoVZ_); // add vz pen
  nh.getParam("rhoVland", rhoVland_); // add vland pen
  nh.getParam("rhoVisibleDomain", rhoVisibleDomain_);
  nh.getParam("landpBound", landpBound_);
  // nh.getParam("rhoA", rhoA_);
  nh.getParam("rhoThrust", rhoThrust_);
  nh.getParam("rhoOmega", rhoOmega_);
  nh.getParam("rhoPerchingCollision", rhoPerchingCollision_);
  nh.getParam("pause_debug", pause_debug_);
  visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
}

// bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
//                           Eigen::Vector3d& gradv,
//                           double& costv) {
//   double vpen = v.squaredNorm() - vmax_ * vmax_;
//   if (vpen > 0) {
//     double grad = 0;
//     costv = smoothedL1(vpen, grad);
//     gradv = rhoV_ * grad * 2 * v;
//     costv *= rhoV_;
//     return true;
//   }
//   return false;
// }

bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  double vxypen = v.x() * v.x() + v.y() * v.y() - vmax_ * vmax_;
  if (vxypen > 0) {
    double grad = 0;
    costv = smoothedL1(vxypen, grad);
    gradv = rhoV_ * grad * 2 * Eigen::Vector3d(v.x(), v.y(), 0);
    costv *= rhoV_;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_v_z(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradvz,
                          double& costvz) {
  double vzpen = v.z() * v.z();
  if (vzpen > 0) {
    double grad = 0;
    costvz = smoothedL1(vzpen, grad);
    gradvz = rhoVZ_ * grad * 2 * Eigen::Vector3d(0,0,v.z());
    costvz *= rhoVZ_;
    return true;
  }
  return false;
}

// bool TrajOpt::grad_cost_v_z(const Eigen::Vector3d& v,
//                           Eigen::Vector3d& gradvz,
//                           double& costvz) {
//   double grad = 0;
//   costvz = rhoVZ_ * 4 * v.z() * v.z();
//   gradvz = rhoVZ_ * 8 * Eigen::Vector3d(0,0,v.z());
//   return true;
// }

bool TrajOpt::grad_cost_thrust(const Eigen::Vector3d& a,
                               Eigen::Vector3d& grada,
                               double& costa) {
  bool ret = false;
  grada.setZero();
  costa = 0;
  Eigen::Vector3d thrust_f = a - g_;
  double max_pen = thrust_f.squaredNorm() - thrust_max_ * thrust_max_;
  if (max_pen > 0) {
    double grad = 0;
    costa = rhoThrust_ * smoothedL1(max_pen, grad);
    grada = rhoThrust_ * 2 * grad * thrust_f;
    ret = true;
  }

  double min_pen = thrust_min_ * thrust_min_ - thrust_f.squaredNorm();
  if (min_pen > 0) {
    double grad = 0;
    costa = rhoThrust_ * smoothedL1(min_pen, grad);
    grada = -rhoThrust_ * 2 * grad * thrust_f;
    ret = true;
  }

  return ret;
}

// bool TrajOpt::grad_cost_visible_domain(const Eigen::Vector3d& pos,
//                                            const Eigen::Vector3d& acc,
//                                            const Eigen::Vector3d& car_p,
//                                            Eigen::Vector3d& gradp,
//                                            Eigen::Vector3d& grada,
//                                            Eigen::Vector3d& grad_car_p,
//                                            double& cost){
//   Eigen::Vector3d pc = - car_p + pos;
//   double dist_sqr = pc.squaredNorm();
//   double safe_r = platform_r_;
//   double safe_r_sqr = safe_r * safe_r;
//   double pen_dist = - safe_r_sqr + dist_sqr;
//   //pen_dist /= safe_r_sqr;
//   double grad_dist = 0;
//   double var01 = smoothed01(pen_dist, grad_dist);

//   if (var01 == 0) {
//     return false;
//   }
  
//   Eigen::Vector3d thrust_f = acc - g_;
//   Eigen::Vector3d zb = f_N(thrust_f);
//   // Eigen::Vector3d zb(0,0,1);
//   Eigen::Vector3d zc(0,0,1);
//   // Eigen::Vector3d pc = - car_p + pos;
//   Eigen::Vector3d pc_norm = pc.normalized(); // normalize pc

//   double dtheta = cos(M_PI / 4);
//   double costheta = pc_norm.dot(zc);
//   double cosphi = zb.dot(zc);
//   // double theta = acos(costheta);
//   double grad = 0;

//   double pen = (dtheta - costheta);
//   cost = smoothedL1(pen, grad);

//   // gradp.x() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (zb.x() * pc.norm() - zb.x() * pow(pc.x(),2) / pc.norm()) / pow(pc.norm(),2);
//   // gradp.y() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (zb.y() * pc.norm() - zb.y() * pow(pc.y(),2) / pc.norm()) / pow(pc.norm(),2);
//   // gradp.z() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (zb.z() * pc.norm() - zb.z() * pow(pc.z(),2) / pc.norm()) / pow(pc.norm(),2);
//   // gradp = gradp * var01 + cost * grad_dist * 2 * pc;

//   // gradp = - grad * 2 * theta * (1 / sqrt(1 - costheta * costheta)) * f_DN(pc).transpose() * zb;
//   // gradp = - 2 * (1 - costheta) * f_DN(pc).transpose() * zb;
//   gradp = - grad * f_DN(pc).transpose() * zb;

//   // grad_car_p.x() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (- zb.x() * pc.norm() + zb.x() * pow(pc.x(),2) / pc.norm()) / pow(pc.norm(),2);
//   // grad_car_p.y() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (- zb.y() * pc.norm() + zb.y() * pow(pc.y(),2) / pc.norm()) / pow(pc.norm(),2);
//   // grad_car_p.z() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (- zb.z() * pc.norm() + zb.z() * pow(pc.z(),2) / pc.norm()) / pow(pc.norm(),2);
//   // grad_car_p = grad_car_p * var01 - cost * grad_dist * 2 * pc;
//   // gradp = 2 * (1 - costheta) * f_DN(pc).transpose() * zb;
//   grad_car_p = grad * f_DN(pc).transpose() * zb;
//   // grad_car_p.setZero();

//   cost += (1 - cosphi) * (1 - cosphi);
//   // grada.x() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * f_DN(thrust_f)
//   // grada.y() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (pc.y() * thrust_f.norm() - pc.y() * pow(thrust_f.y(),2) / thrust_f.norm()) / (pc.norm() * pow(thrust_f.norm(),2));
//   // grada.z() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (pc.z() * thrust_f.norm() - pc.z() * pow(thrust_f.z(),2) / thrust_f.norm()) / (pc.norm() * pow(thrust_f.norm(),2));
//   grada = - 2 * (1 - cosphi) * f_DN(thrust_f).transpose() * zc;
//   // grada = - 2 * (1 - costheta) * f_DN(thrust_f).transpose() * pc_norm;
//   // grada *= var01;
//   // grada.setZero();
  
//   cost *= var01;
//   gradp = grad_dist * 2 * pc * cost + var01 * gradp;
//   grad_car_p = - grad_dist * 2 * pc * cost + var01 * grad_car_p;
//   grada *= var01;
//   cost *= rhoVisibleDomain_;
//   gradp *= rhoVisibleDomain_;
//   grad_car_p *= rhoVisibleDomain_;
//   grada *= rhoVisibleDomain_;

//   return true;
// }

bool TrajOpt::grad_cost_visible_domain(const Eigen::Vector3d& pos,
                                           const Eigen::Vector3d& acc,
                                           const Eigen::Vector3d& car_p,
                                           Eigen::Vector3d& gradp,
                                           Eigen::Vector3d& grada,
                                           Eigen::Vector3d& grad_car_p,
                                           double& cost){
  Eigen::Vector3d pc = - car_p + pos;
  double dist_sqr = pc.squaredNorm();
  // double safe_r = platform_r_;
  double safe_r_sqr = platform_r_ * platform_r_;
  double pen_dist = - safe_r_sqr + dist_sqr;
  //pen_dist /= safe_r_sqr;
  double grad_dist = 0;
  double var01 = smoothed01(pen_dist, grad_dist);

  if (var01 == 0) {
    return false;
  }
  
  Eigen::Vector3d thrust_f = acc - g_;
  Eigen::Vector3d zb = f_N(thrust_f);
  // Eigen::Vector3d zb(0,0,1);
  Eigen::Vector3d zc(0,0,1);
  // Eigen::Vector3d pc = - car_p + pos;
  Eigen::Vector3d pc_norm = pc.normalized(); // normalize pc

  double costheta = pc_norm.dot(zc);
  double cosphi = zb.dot(zc);
  // double theta = acos(costheta);
  double grad = 0;
  cost = (1 - costheta) * (1 - costheta) + (1 - cosphi) * (1 - cosphi);
  // cost = smoothedL1(pen, grad);
  if(cost > 0){
    // gradp.x() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (zb.x() * pc.norm() - zb.x() * pow(pc.x(),2) / pc.norm()) / pow(pc.norm(),2);
    // gradp.y() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (zb.y() * pc.norm() - zb.y() * pow(pc.y(),2) / pc.norm()) / pow(pc.norm(),2);
    // gradp.z() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (zb.z() * pc.norm() - zb.z() * pow(pc.z(),2) / pc.norm()) / pow(pc.norm(),2);
    // gradp = gradp * var01 + cost * grad_dist * 2 * pc;

    // gradp = - grad * 2 * theta * (1 / sqrt(1 - costheta * costheta)) * f_DN(pc).transpose() * zb;
    // gradp = - 2 * (1 - costheta) * f_DN(pc).transpose() * zb;
    gradp = - 2 * (1 - costheta) * f_DN(pc).transpose() * zc;

    // grad_car_p.x() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (- zb.x() * pc.norm() + zb.x() * pow(pc.x(),2) / pc.norm()) / pow(pc.norm(),2);
    // grad_car_p.y() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (- zb.y() * pc.norm() + zb.y() * pow(pc.y(),2) / pc.norm()) / pow(pc.norm(),2);
    // grad_car_p.z() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (- zb.z() * pc.norm() + zb.z() * pow(pc.z(),2) / pc.norm()) / pow(pc.norm(),2);
    // grad_car_p = grad_car_p * var01 - cost * grad_dist * 2 * pc;
    // gradp = 2 * (1 - costheta) * f_DN(pc).transpose() * zb;
    grad_car_p = 2 * (1 - costheta) * f_DN(pc).transpose() * zc;
    // grad_car_p.setZero();

    // grada.x() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * f_DN(thrust_f)
    // grada.y() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (pc.y() * thrust_f.norm() - pc.y() * pow(thrust_f.y(),2) / thrust_f.norm()) / (pc.norm() * pow(thrust_f.norm(),2));
    // grada.z() = - 2 * grad * theta * (1 / sqrt(1 - pow(costheta,2))) * (pc.z() * thrust_f.norm() - pc.z() * pow(thrust_f.z(),2) / thrust_f.norm()) / (pc.norm() * pow(thrust_f.norm(),2));
    grada = - 2 * (1 - cosphi) * f_DN(thrust_f).transpose() * zc;
    // grada = - 2 * (1 - costheta) * f_DN(thrust_f).transpose() * pc_norm;
    // grada *= var01;
    // grada.setZero();
    
    cost *= var01;
    gradp = grad_dist * 2 * pc * cost + var01 * gradp;
    grad_car_p = - grad_dist * 2 * pc * cost + var01 * grad_car_p;
    grada *= var01;
    // cost += var01;
    cost *= rhoVisibleDomain_;
    gradp *= rhoVisibleDomain_;
    grad_car_p *= rhoVisibleDomain_;
    grada *= rhoVisibleDomain_;
    return true;
  }
  return false;
}

// using hopf fibration:
// [a,b,c] = thrust.normalized()
// \omega_1 = sin(\phi) \dot{a] - cos(\phi) \dot{b} - (a sin(\phi) - b cos(\phi)) (\dot{c}/(1+c))
// \omega_2 = cos(\phi) \dot{a] - sin(\phi) \dot{b} - (a cos(\phi) - b sin(\phi)) (\dot{c}/(1+c))
// \omega_3 = (b \dot{a} - a \dot(b)) / (1+c)
// || \omega_12 ||^2 = \omega_1^2 + \omega_2^2 = \dot{a}^2 + \dot{b}^2 + \dot{c}^2

bool TrajOpt::grad_cost_omega(const Eigen::Vector3d& a,
                              const Eigen::Vector3d& j,
                              Eigen::Vector3d& grada,
                              Eigen::Vector3d& gradj,
                              double& cost) {
  Eigen::Vector3d thrust_f = a - g_;
  Eigen::Vector3d zb_dot = f_DN(thrust_f) * j;
  double omega_12_sq = zb_dot.squaredNorm();
  double pen = omega_12_sq - omega_max_ * omega_max_;
  if (pen > 0) {
    double grad = 0;
    cost = smoothedL1(pen, grad);

    Eigen::Vector3d grad_zb_dot = 2 * zb_dot;
    // std::cout << "grad_zb_dot: " << grad_zb_dot.transpose() << std::endl;
    gradj = f_DN(thrust_f).transpose() * grad_zb_dot;
    grada = f_D2N(thrust_f, j).transpose() * grad_zb_dot;

    cost *= rhoOmega_;
    grad *= rhoOmega_;
    grada *= grad;
    gradj *= grad;

    return true;
  }
  return false;
}

// bool TrajOpt::grad_cost_omega_yaw(const Eigen::Vector3d& a,
//                                   const Eigen::Vector3d& j,
//                                   Eigen::Vector3d& grada,
//                                   Eigen::Vector3d& gradj,
//                                   double& cost) {
//   // TODO
//   return false;
// }

bool TrajOpt::grad_cost_floor(const Eigen::Vector3d& p,
                              Eigen::Vector3d& gradp,
                              double& costp) {
  static double z_floor = 0.4;
  double pen = z_floor - p.z(); // 公式12为[z_f^2 - p.z^2]
  if (pen > 0) {
    double grad = 0;
    costp = smoothedL1(pen, grad);
    costp *= rhoPf_;
    gradp.setZero();
    gradp.z() = -rhoPf_ * grad;
    return true;
  } else {
    return false;
  }
}

bool TrajOpt::grad_cost_top(const Eigen::Vector3d& p,
                              Eigen::Vector3d& gradp,
                              double& costp) {
  static double z_top = car_p_.z() + 3.0; // max height set to a fixed values related to the initial state
  double pen = p.z() - z_top; // 公式12为[z_f^2 - p.z^2]
  if (pen > 0) {
    double grad = 0;
    costp = smoothedL1(pen, grad);
    costp *= rhoPt_;
    gradp.setZero();
    gradp.z() = rhoPt_ * grad;
    return true;
  } else {
    return false;
  }
}

// plate: \Epsilon = \left{ x = RBu + c | \norm(u) \leq r \right}
// x \in R_{3\times1}, u \in R_{2\times1}, B \in R_{3\times2}
// c: center of the plate; p: center of the drone bottom
//  c = p - l * z_b
// plane: a^T x \leq b
//        a^T(RBu + c) \leq b
//        a^T(RBu + p - l * z_b) \leq b
//        u^T(B^T R^T a) + a^Tp - a^T*l*z_b - b \leq 0
//        r \norm(B^T R^T a) + a^Tp - a^T*l*z_b - b \leq 0
// B^T R^T = [1-2y^2,    2xy, -2yw;
//               2xy, 1-2x^2,  2xw]
// B^T R^T = [1-a^2/(1+c),   -ab/(1+c), -a;
//              -ab/(1+c), 1-b^2/(1+c), -b]
bool TrajOpt::grad_cost_perching_collision(const Eigen::Vector3d& pos,
                                           const Eigen::Vector3d& acc,
                                           const Eigen::Vector3d& car_p,
                                           Eigen::Vector3d& gradp,
                                           Eigen::Vector3d& grada,
                                           Eigen::Vector3d& grad_car_p,
                                           double& cost) {
  static double eps = 1e-6;

  double dist_sqr = (pos - car_p).squaredNorm();
  double safe_r = platform_r_ + robot_r_;
  double safe_r_sqr = safe_r * safe_r;
  double pen_dist = safe_r_sqr - dist_sqr;
  pen_dist /= safe_r_sqr;
  double grad_dist = 0;
  double var01 = smoothed01(pen_dist, grad_dist);
  if (var01 == 0) {
    return false;
  }
  Eigen::Vector3d gradp_dist = grad_dist * 2 * (car_p - pos);
  Eigen::Vector3d grad_carp_dist = -gradp_dist;

  Eigen::Vector3d a_i = -tail_q_v_;
  double b_i = a_i.dot(car_p);

  Eigen::Vector3d thrust_f = acc - g_;
  Eigen::Vector3d zb = f_N(thrust_f);

  Eigen::MatrixXd BTRT(2, 3);
  double a = zb.x();
  double b = zb.y();
  double c = zb.z();

  double c_1 = 1.0 / (1 + c);

  BTRT(0, 0) = 1 - a * a * c_1;
  BTRT(0, 1) = -a * b * c_1;
  BTRT(0, 2) = -a;
  BTRT(1, 0) = -a * b * c_1;
  BTRT(1, 1) = 1 - b * b * c_1;
  BTRT(1, 2) = -b;  // BTRT denote the uav rotation without yaw

  Eigen::Vector2d v2 = BTRT * a_i;
  double v2_norm = sqrt(v2.squaredNorm() + eps);
  double pen = a_i.dot(pos) - (robot_l_ - 0.005) * a_i.dot(zb) - b_i + robot_r_ * v2_norm;

  if (pen > 0) {
    double grad = 0;
    cost = smoothedL1(pen, grad);
    // gradients: pos, car_p, v2
    gradp = a_i;
    grad_car_p = -a_i;
    Eigen::Vector2d grad_v2 = robot_r_ * v2 / v2_norm;

    Eigen::MatrixXd pM_pa(2, 3), pM_pb(2, 3), pM_pc(2, 3);
    double c2_1 = c_1 * c_1;

    pM_pa(0, 0) = -2 * a * c_1;
    pM_pa(0, 1) = -b * c_1;
    pM_pa(0, 2) = -1;
    pM_pa(1, 0) = -b * c_1;
    pM_pa(1, 1) = 0;
    pM_pa(1, 2) = 0;

    pM_pb(0, 0) = 0;
    pM_pb(0, 1) = -a * c_1;
    pM_pb(0, 2) = 0;
    pM_pb(1, 0) = -a * c_1;
    pM_pb(1, 1) = -2 * b * c_1;
    pM_pb(1, 2) = -1;

    pM_pc(0, 0) = a * a * c2_1;
    pM_pc(0, 1) = a * b * c2_1;
    pM_pc(0, 2) = 0;
    pM_pc(1, 0) = a * b * c2_1;
    pM_pc(1, 1) = b * b * c2_1;
    pM_pc(1, 2) = 0;

    Eigen::MatrixXd pv2_pzb(2, 3);
    pv2_pzb.col(0) = pM_pa * a_i;
    pv2_pzb.col(1) = pM_pb * a_i;
    pv2_pzb.col(2) = pM_pc * a_i;

    Eigen::Vector3d grad_zb = pv2_pzb.transpose() * grad_v2 - robot_l_ * a_i;

    grada = f_DN(thrust_f).transpose() * grad_zb;

    grad *= var01;
    gradp_dist *= cost;
    grad_carp_dist *= cost;
    cost *= var01;
    gradp = grad * gradp + gradp_dist;
    grada *= grad;
    grad_car_p = grad * grad_car_p + grad_carp_dist;

    cost *= rhoPerchingCollision_;
    gradp *= rhoPerchingCollision_;
    grada *= rhoPerchingCollision_;
    grad_car_p *= rhoPerchingCollision_;

    // std::cout << "var01: " << var01 << std::endl;

    return true;
  }
  return false;
}

bool TrajOpt::check_collilsion(const Eigen::Vector3d& pos,
                               const Eigen::Vector3d& acc,
                               const Eigen::Vector3d& car_p) {
  if ((pos - car_p).norm() > platform_r_) {
    return false;
  }
  static double eps = 1e-6;

  Eigen::Vector3d a_i = -tail_q_v_;
  double b_i = a_i.dot(car_p); //得到降落时速度在平台位置向量上的分量

  Eigen::Vector3d thrust_f = acc - g_;
  Eigen::Vector3d zb = f_N(thrust_f); //推力的单位向量

  Eigen::MatrixXd BTRT(2, 3);
  double a = zb.x();
  double b = zb.y();
  double c = zb.z();

  double c_1 = 1.0 / (1 + c);

  BTRT(0, 0) = 1 - a * a * c_1;
  BTRT(0, 1) = -a * b * c_1;
  BTRT(0, 2) = -a;
  BTRT(1, 0) = -a * b * c_1;
  BTRT(1, 1) = 1 - b * b * c_1;
  BTRT(1, 2) = -b;

  Eigen::Vector2d v2 = BTRT * a_i;
  double v2_norm = sqrt(v2.squaredNorm() + eps);
  double pen = a_i.dot(pos) - (robot_l_ - 0.005) * a_i.dot(zb) - b_i + robot_r_ * v2_norm;

  return pen > 0;
}

bool TrajOpt::feasibleCheck(Trajectory& traj) {
  double dt = 0.01;
  for (double t = 0; t < traj.getTotalDuration(); t += dt) {
    Eigen::Vector3d p = traj.getPos(t);
    Eigen::Vector3d a = traj.getAcc(t);
    Eigen::Vector3d j = traj.getJer(t);
    Eigen::Vector3d thrust = a - g_;
    Eigen::Vector3d zb_dot = f_DN(thrust) * j;
    double omega12 = zb_dot.norm(); // 角速度前两个分量平方和
    if (omega12 > omega_max_ + 0.2) {
      return false;
    }
    if (p.z() < 0.1) {
      return false;
    }
  }
  return true;
}

}  // namespace traj_opt