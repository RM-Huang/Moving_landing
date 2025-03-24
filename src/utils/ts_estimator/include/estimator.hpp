#include <ros/ros.h>
#include <coptcpp_pch.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct>
#include <omp.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <string>

namespace estimate
{
    class Solver{
        private:
        bool CONTINUES_ESTIMATE_ = 0; // 是否使用迭代估计
        int N_ = 10; // 观测状态数
        double weight_decrese_rate = 1.0; // 权重下降比例
        
        double T_total = 0; // 总时间偏移
        std::vector<Eigen::MatrixXd> At;
        std::vector<Eigen::MatrixXd> At_bar;

        Envr copt_env;
        Model model = copt_env.CreateModel("sdp_q");
        PsdVar Z = model.AddPsdVar(20, "Z");;
        // void init_sdp_quastion();

        void init_constrains();

        void get_nonZero_vals(const Eigen::MatrixXd& Mat, std::vector<int>& rows, std::vector<int>& cols, std::vector<double>& vals);

        void print_DenseMatrix_asSym(const Eigen::MatrixXd& Mat, std::string name);

        int rank_count(const Eigen::MatrixXd& Mat, const std::string& name);

        Eigen::MatrixXd get_inverse_Matrix(const Eigen::MatrixXd& mat);

        Eigen::MatrixXd psdVector_2_MatrixXd(const std::vector<double>& vec, const int dim);

        Eigen::VectorXd revert_z_from_Z(const std::vector<double>& vec, const int dim, int& rank);

        // SymMatrix EigenMatrix_2_SymMatrix(const Eigen::MatrixXd& Mat, int size);

        Eigen::Vector3d time_shift(const Eigen::Vector3d& p, const Eigen::Vector3d& v);
        
        Eigen::MatrixXd get_At_matrix(const Eigen::Vector3d& p_uu, const Eigen::Vector3d& p_cc, const Eigen::Quaterniond& q_uu,
                                    const Eigen::Vector3d& v_cc, const Eigen::Vector3d& b, const int n);

        Eigen::MatrixXd get_At_bar_matrix(const Eigen::Vector3d& p_uv, const Eigen::Vector3d& p_cc, const Eigen::Vector3d& v_cc);

        void update_At_vector(const int n, Eigen::MatrixXd& At);

        void update_Q_matrix(Eigen::MatrixXd& Q);

        void update_Q_bar_matrix(Eigen::MatrixXd& Q);

        public:
        Solver(){};
        ~Solver(){};

        int init(const bool if_iter, const int N, const double R);

        int optimize_vision(const Eigen::Vector3d& p_uv, const Eigen::Vector3d& p_cc, const Eigen::Vector3d& v_cc, Eigen::VectorXd& x);

        int optimize(const Eigen::Vector3d& p_uu, const Eigen::Vector3d& p_cc, const Eigen::Quaterniond& q_uu,
                        const Eigen::Vector3d& v_cc, const Eigen::Vector3d& b, Eigen::VectorXd& x);

        
    };
} // namespace estimate