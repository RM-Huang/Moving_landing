#include <ros/ros.h>
#include <coptcpp_pch.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <omp.h>
#include <cmath>

namespace estimate
{
    class Solver{
        private:
        bool CONTINUES_ESTIMATE_ = 0; // 是否使用迭代估计
        int N_ = 10; // 观测状态数
        double weight_decrese_rate = 1.0; // 权重下降比例

        double T_total = 0; // 总时间偏移
        std::vector<Eigen::MatrixXd> At;

        Envr copt_env;
        Model model;
        void init_Psd_quastion();
        
        Eigen::MatrixXd get_At_matrix(const Eigen::Vector3d& p_uu, const Eigen::Vector3d& p_cc, const Eigen::Quaterniond& q_uu,
                                    const Eigen::Vector3d& v_cc, const Eigen::Vector3d& b, const int n);

        void update_At_matrix(const int n, Eigen::Ref(Eigen::MatrixXd) At);

        void update_Q_matrix(Eigen::Ref(Eigen::MatrixXd) Q);

        public:
        Solver(bool if_iter);
        ~Solver(){};

        void reset();

        bool optimize(const Eigen::Vector3d& p_uu, const Eigen::Vector3d& p_cc, const Eigen::Quaterniond& q_uu,
                        const Eigen::Vector3d& v_cc, const Eigen::Vector3d& b);

        
    }
} // namespace estimate