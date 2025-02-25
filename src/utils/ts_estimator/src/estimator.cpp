#include "estimator.hpp"

namespace estimate
{
    Solver::Solver(bool if_iter){
        CONTINUES_ESTIMATE_ = if_iter;

        // COPT init
        model = copt_env.CreateModel("psd_q");
    }

    void Solver::init_Psd_quastion(){
        // 设置半定变量
        PsdVar Z = model.AddPsdVar(20, "Z");

        // 设置半定约束
        model.AddPsdConstr()
    }

    Solver::reset(){
        T_total = 0;
        At.clear();

        model.ResetAll();
    }

    void Solver::update_At_matrix(const int n, Eigen::Ref(Eigen::MatrixXd) Atk){
        Atk.col(n) = Atk.col(n + 1);
        Atk.col(n + 1) = Eigen::Vector3d::Zero();
    }

    Eigen::MatrixXd Solver::get_At_matrix(const Eigen::Vector3d& p_uu, const Eigen::Vector3d& p_cc, const Eigen::Quaterniond& q_uu,
                                    const Eigen::Vector3d& v_cc, const Eigen::Vector3d& b, const int n){ // n start from 0
        Eigen::MatrixXd A_tk(3, 22 + N_) = Eigen::MatrixXd::Zero(3, 22 + N_);
        Eigen::Block<Eigen::MatrixXd> A_x = A_tk.block(0, 0, 3, 19);
        Eigen::Block<Eigen::MatrixXd> A_s = A_tk.block(0, 19, 3, (3 + N_));
        Eigen::MatrixXd I = EIgen::MatrixXd::Identity(3,3);

        A_x.block(0, 0, 3, 9) = Eigen::KroneckerProduct(p_cc.transpose(), -1 * I);
        A_x.block(0, 9, 3, 9) = Eigen::KroneckerProduct(v_cc.transpose(), -1 * I);
        A_x.col(18) = p_uu;

        A_s.block(0, 0, 3, 3) = -I;
        A_s.col(3 + n) = q_uu.toRotationMatrix() * b;

        return A_tk;
    }

    void Solver::update_Q_matrix(Eigen::Ref(Eigen::MatrixXd) Q){
        int size = At.size();
        #pragma omp parallel for num_threads(5) reduction(+:sum)
        for(int i = 0; i < size; i++){
            update_At_matrix(i, At[i]);
            Q += pow(weight_decrese_rate, (N_ - 1 - i)) * At[i].transpose() * At[i];
        }
    }

    Solver::optimize(const Eigen::Vector3d& p_uu, const Eigen::Vector3d& p_cc, const Eigen::Vector3d& q_uu, 
                        const Eigen::Vector3d& v_cc, const Eigen::Vector3d& b){
        Eigen::MatrixXd A_latest = get_At_matrix(p_uu, p_cc, q_uu, v_cc, b, (N_ - 1));
        Eigen::MatrixXd Q = A_latest.transpose() * A_latest;

        if(At.size() >= N_){
            At.erase(At.begin());
        }
        update_Q_matrix(Q);
        At.emplace_back(A_latest);

        Eigen::Block<Eigen::MatrixXd> Q_a = Q.block(0, 0, 19, 19);
        Eigen::Block<Eigen::MatrixXd> Q_b = Q.block(0, 19, 19, (3 + N_));
        Eigen::Block<Eigen::MatrixXd> Q_c = Q.block(19, 19, (3 + N_), (3 + N_));

        Eigen::MatrixXd Q_0 = Eigen::MatrixXd::Zero(20, 20);
        Q_0.block(0, 0, 19, 19) = Q_a - Q_b * Q_c.inverse() * Q_b.transpose();


        model.SetObjective(1.2 * x + 1.8 * y + 2.1 * z, COPT_MAXIMIZE);
    }

} // namespace estimate
