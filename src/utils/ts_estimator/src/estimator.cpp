#include "estimator.hpp"

template <typename T>
static void print_vector(const char* name, const std::vector<T>& vec){
    std::cout << name << ": " << std::endl;
    for(auto &elem : vec){
        std::cout << elem << ", " << std::endl;
    }
    std::cout << std::endl;
}

namespace estimate
{
    void Solver::init_constrains(){
        // SymMatrix Q_i; // 半定约束系数矩阵
        // std::vector<int> g_i; // 半定约束值
        // g_i = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
        
        std::vector<int> rows;
        std::vector<int> cols;
        std::vector<double> vals;

        // rotation constrains
        rows = {0, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 18};
        cols = {0, 1, 2, 0, 3, 1, 4, 2, 5, 0, 3, 6, 1, 4, 7, 2, 5, 8, 18};
        vals = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -3};
        SymMatrix Q_1 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_1 * Z == 0, "Cons_1");

        rows = {9, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 17, 19};
        cols = {9, 10, 11, 9,  12, 10, 13, 11, 14, 9,  12, 15, 10, 13, 16, 11, 14, 17, 19};
        vals = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -3};
        SymMatrix Q_2 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_2 * Z == 0, "Cons_2");

        rows = {0, 1, 1, 2, 2, 2, 3, 4, 4, 5, 5, 5, 6, 7, 7, 8, 8, 8, 18};
        cols = {0, 0, 1, 0, 1, 2, 3, 3, 4, 3, 4, 5, 6, 6, 7, 6, 7, 8, 18};
        vals = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -3};
        SymMatrix Q_3 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_3 * Z == 0, "Cons_3");

        rows = {9, 10, 10, 11, 11, 11, 12, 13, 13, 14, 14, 14, 15, 16, 16, 17, 17, 17, 19};
        cols = {9, 9,  10, 9,  10, 11, 12, 12, 13, 12, 13, 14, 15, 15, 16, 15, 16, 17, 19};
        vals = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -3};
        SymMatrix Q_4 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_4 * Z == 0, "Cons_4");

        rows = {3, 3, 4, 4, 5, 5, 18, 18, 18};
        cols = {1, 2, 0, 2, 0, 1, 6,  7,  8};
        vals = {1/2, -1/2, -1/2, 1/2, 1/2, -1/2, 1/2, 1/2, 1/2};
        SymMatrix Q_5 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_5 * Z == 0, "Cons_5");

        rows = {12, 12, 13, 13, 14, 14, 19, 19, 19};
        cols = {10, 11, 9,  11, 9,  10, 15, 16, 17};
        vals = {1/2, -1/2, -1/2, 1/2, 1/2, -1/2, 1/2, 1/2, 1/2};
        SymMatrix Q_6 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_6 * Z == 0, "Cons_6");

        rows = {6, 6, 7, 7, 8, 8, 18, 18, 18};
        cols = {4, 5, 3, 5, 3, 4, 0 , 1,  2};
        vals = {1/2, -1/2, -1/2, 1/2, 1/2, -1/2, 1/2, 1/2, 1/2};
        SymMatrix Q_7 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_7 * Z == 0, "Cons_7");

        rows = {15, 15, 16, 16, 17, 17, 19, 19, 19};
        cols = {13, 14, 12, 14, 12, 13, 9,  10, 11};
        vals = {1/2, -1/2, -1/2, 1/2, 1/2, -1/2, 1/2, 1/2, 1/2};
        SymMatrix Q_8 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_8 * Z == 0, "Cons_8");

        rows = {6, 6, 7, 7, 8, 8, 18, 18, 18};
        cols = {1, 2, 0, 2, 0, 1, 3,  4,  5};
        vals = {-1/2, 1/2, 1/2, -1/2, -1/2, 1/2, 1/2, 1/2, 1/2};
        SymMatrix Q_9 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_9 * Z == 0, "Cons_9");

        rows = {15, 15, 16, 16, 17, 17, 19, 19, 19};
        cols = {10, 11, 9,  11, 9,  10, 12, 13, 14};
        vals = {-1/2, 1/2, 1/2, -1/2, -1/2, 1/2, 1/2, 1/2, 1/2};
        SymMatrix Q_10 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_10 * Z == 0, "Cons_10");

        // y * y = 1
        rows = {18};
        cols = {18};
        vals = {1};
        SymMatrix Q_11 = model.AddSparseMat(20, 1, rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_11 * Z == 1, "Cons_11");

        // delta_tau * r_p = y * r_s
        rows = {19, 19, 19, 19, 19, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18, 18};
        cols = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
        vals = {-1/2, -1/2, -1/2, -1/2, -1/2, -1/2, -1/2, -1/2, -1/2, 1/2, 1/2, 1/2, 1/2, 1/2, 1/2, 1/2, 1/2, 1/2};
        SymMatrix Q_12 = model.AddSparseMat(20, 18, rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_12 * Z == 0, "Cons_12");

        // tmp: t_bias = 0
        rows = {9, 10, 11, 12, 13, 14, 15, 16, 17, 19};
        cols = {9, 10, 11, 12, 13, 14, 15, 16, 17, 19};
        vals = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
        SymMatrix Q_tmp_1 = model.AddSparseMat(20, 10, rows.data(), cols.data(), vals.data());
        model.AddPsdConstr(Q_tmp_1 * Z == 0, "tmp_Cons_1");

        // tmp: roll, pitch = 0
        // rows = {2, 5, 6, 7};
        // cols = {2, 5, 6, 7};
        // vals = {1, 1, 1, 1};
        // SymMatrix Q_tmp_2 = model.AddSparseMat(20, 4, rows.data(), cols.data(), vals.data());
        // model.AddPsdConstr(Q_tmp_2 * Z == 0, "tmp_Cons_2");

        // rows = {11, 14, 15, 16};
        // cols = {11, 14, 15, 16};
        // vals = {1, 1, 1, 1};
        // SymMatrix Q_tmp_3 = model.AddSparseMat(20, 4, rows.data(), cols.data(), vals.data());
        // model.AddPsdConstr(Q_tmp_3 * Z == 0, "tmp_Cons_3");

        // tmp: yaw = 0
        // rows = {1, 3};
        // cols = {1, 3};
        // vals = {1, 1};
        // SymMatrix Q_tmp_4 = model.AddSparseMat(20, 2, rows.data(), cols.data(), vals.data());
        // model.AddPsdConstr(Q_tmp_4 * Z == 0, "tmp_Cons_4");

        // rows = {10, 12};
        // cols = {10, 12};
        // vals = {1, 1};
        // SymMatrix Q_tmp_5 = model.AddSparseMat(20, 2, rows.data(), cols.data(), vals.data());
        // model.AddPsdConstr(Q_tmp_5 * Z == 0, "tmp_Cons_5");
    }

    int Solver::init(const bool if_iter, const int N, const double R){
        CONTINUES_ESTIMATE_ = if_iter;
        N_ = N;
        weight = R;
        omp_set_num_threads(10);
        
        T_total = 0;
        At.clear();
        At_bar.clear();

        try{
            model.ResetAll();
            init_constrains();
            model.SetIntParam(COPT_INTPARAM_LOGTOCONSOLE, 0);
        }catch (CoptException e){
            std::cout << "Error Code = " << e.GetCode() << std::endl;
            std::cout << e.what() << std::endl;
            return e.GetCode();
        }
        return 1;
    }

    void Solver::get_nonZero_vals(const Eigen::MatrixXd& Mat, std::vector<int>& rows, std::vector<int>& cols, std::vector<double>& vals){
        Eigen::SparseMatrix<double> sparse_Mat = Mat.sparseView();
        for (int k = 0; k < sparse_Mat.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(sparse_Mat, k); it; ++it) {
                vals.emplace_back(it.value());
                rows.emplace_back(it.row());
                cols.emplace_back(it.col());
            }
        }
    }

    void Solver::print_DenseMatrix_asSym(const Eigen::MatrixXd& Mat, std::string name){
        std::vector<int> rows;
        std::vector<int> cols;
        std::vector<double> vals;
        std::cout << name << " = "<<std::endl;
        get_nonZero_vals(Mat, rows, cols, vals);


        std::cout << "size = " << vals.size() << std::endl;
        std::cout << "rows =";
        for(int i = 0; i < rows.size(); i++){
            std::cout <<" "<< rows[i];
        }
        std::cout << std::endl;

        std::cout << "cols =";
        for(int i = 0; i < cols.size(); i++){
            std::cout <<" "<< cols[i];
        }
        std::cout << std::endl;

        std::cout << "vals =";
        for(int i = 0; i < vals.size(); i++){
            std::cout <<" "<< vals[i];
        }
        std::cout << std::endl;
    }

    int Solver::rank_count(const Eigen::MatrixXd& Mat, const std::string& name){
        Eigen::FullPivLU<Eigen::MatrixXd> lu(Mat);
        lu.setThreshold(1e-5);
        int rank = lu.rank();
        int corank = Mat.cols() - rank;
        // 获取主元行的索引（前 rank 个主元行是线性无关的）
        Eigen::VectorXi pivot_rows = lu.permutationP().indices().head(rank);
        // 提取线性无关行
        Eigen::MatrixXd independent_rows(lu.rank(), Mat.cols());
        for (int i = 0; i < pivot_rows.size(); ++i) {
            independent_rows.row(i) = Mat.row(pivot_rows[i]);
        }
        // std::cout << name << "_rank = " << rank <<std::endl;
        // std::cout << name << "_corank = " << corank << std::endl;
        // std::cout << name << "_independent_row_idx : " << pivot_rows.transpose() << std::endl;

        return rank;
    }

    Eigen::Matrix3d Solver::recoverRotation_from_vector(const Eigen::VectorXd& vec){
        Eigen::Matrix3d rot;
        rot.col(0) = vec.segment(0, 3);
        rot.col(1) = vec.segment(3, 3);
        rot.col(2) = vec.segment(6, 3);
        return rot;
    }

    int Solver::get_inverse_Matrix(const Eigen::MatrixXd& mat, Eigen::MatrixXd& inv){
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = 1e-15; // 设置奇异值阈值
        Eigen::VectorXd singular_values = svd.singularValues();

        if (singular_values.minCoeff() < tolerance * singular_values.maxCoeff()) {
            std::cerr << "Matrix is singular or ill-conditioned!\n";
            return 0;
        }

        // 计算逆矩阵
        inv = svd.matrixV() * (singular_values.array().inverse().matrix().asDiagonal()) * 
                                svd.matrixU().adjoint();
        return 1;
    }

    Eigen::MatrixXd Solver::psdVector_2_MatrixXd(const std::vector<double>& vec, const int dim){
        Eigen::MatrixXd Mat = Eigen::MatrixXd::Zero(dim, dim);

        #pragma omp parallel for
        for(int i = 0; i < vec.size(); i++){
            int row = i / dim;
            int col = i % dim;
            Mat(row, col) = vec[i];
        }
        return Mat;
    }

    Eigen::VectorXd Solver::revert_z_from_Z(const std::vector<double>& vec, const int dim, int& rank){
        Eigen::MatrixXd Z_ = psdVector_2_MatrixXd(vec, dim);
        rank = rank_count(Z_,"Z_");
        Eigen::VectorXd z = Eigen::VectorXd::Zero(dim);

        // // direct decomp
        // for(int i = 0; i < dim; i++){
        //     z(i) = std::sqrt(Z_(i,i));
        // }

        // rank-1 decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(Z_);
        if (eigen_solver.info() != Eigen::Success) {
            std::cerr << "Eigenvalue decomposition failed!\n";
        }
        Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
        Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

        int max_index;
        double max_eigenvalue = eigenvalues.maxCoeff(&max_index);

        // Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Z_);
        z = std::sqrt(max_eigenvalue) * eigenvectors.col(max_index);
        // std::cout << eigenvectors.col(idx)
        
        // int eigennums = eigenvalues.size();
        // for(int i = 0; i < eigennums; i++){
        //     z += std::sqrt(eigenvalues[i]) * eigenvectors.col(i);
        // }
        // z = z / eigennums;
        return z;
    }

    Eigen::Vector3d Solver::time_shift(const Eigen::Vector3d& p, const Eigen::Vector3d& v){
        if(CONTINUES_ESTIMATE_){
            Eigen::Vector3d p_t;
            p_t = p + T_total * v;
            return p_t;
        }
        return p;    
    }

    void Solver::update_At_vector(const int n, Eigen::MatrixXd& Atk){
        int ind = 22 + n;
        Atk.col(ind) = Atk.col(ind + 1);
        Atk.col(ind + 1) = Eigen::Vector3d::Zero();

        if(CONTINUES_ESTIMATE_){
            Eigen::Vector3d p(-Atk(0, 0), -Atk(0, 3), -Atk(0, 6));
            Eigen::Vector3d v(-Atk(0, 9), -Atk(0, 12), -Atk(0, 15));
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
            p = time_shift(p, v);
            Atk.block(0, 0, 3, 9) = Eigen::KroneckerProduct<Eigen::RowVector3d, Eigen::Matrix3d>(p.transpose(), -1 * I);
        }
    }

    Eigen::MatrixXd Solver::get_At_matrix(const Eigen::Vector3d& p_uu, const Eigen::Vector3d& p_cc, const Eigen::Quaterniond& q_uu,
                                    const Eigen::Vector3d& v_cc, const Eigen::Vector3d& b, const int n){ // n start from 0
        Eigen::MatrixXd A_tk = Eigen::MatrixXd::Zero(3, 22 + N_);
        Eigen::Block<Eigen::MatrixXd> A_x = A_tk.block(0, 0, 3, 19);
        Eigen::Block<Eigen::MatrixXd> A_s = A_tk.block(0, 19, 3, (3 + N_));
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);

        Eigen::Vector3d p_cct = time_shift(p_cc, v_cc);

        A_x.block(0, 0, 3, 9) = Eigen::KroneckerProduct<Eigen::RowVector3d, Eigen::Matrix3d>(p_cct.transpose(), -1 * I);
        A_x.block(0, 9, 3, 9) = Eigen::KroneckerProduct<Eigen::RowVector3d, Eigen::Matrix3d>(v_cc.transpose(), -1 * I);
        A_x.col(18) = p_uu;

        A_s.block(0, 0, 3, 3) = -I;
        // A_s.col(3 + n) = q_uu.toRotationMatrix() * b;
        A_s.col(3 + n) = b;
        
        // Eigen::Vector3d err_p = A_s.col(3 + n) * (p_uu - p_cc).norm() + p_uu - p_cc;
        // std::cout << "err_p = " << err_p.transpose() << std::endl;
        return A_tk;
    }

    Eigen::MatrixXd Solver::get_At_bar_matrix(const Eigen::Vector3d& p_uv, const Eigen::Vector3d& p_cc, const Eigen::Vector3d& v_cc){
        Eigen::MatrixXd A_tk = Eigen::MatrixXd::Zero(3, 22);
        Eigen::Block<Eigen::MatrixXd> A_x = A_tk.block(0, 0, 3, 19);
        Eigen::Block<Eigen::MatrixXd> A_s = A_tk.block(0, 19, 3, 3);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);

        A_x.block(0, 0, 3, 9) = Eigen::KroneckerProduct<Eigen::RowVector3d, Eigen::Matrix3d>(p_cc.transpose(), -1 * I);
        A_x.block(0, 9, 3, 9) = Eigen::KroneckerProduct<Eigen::RowVector3d, Eigen::Matrix3d>(v_cc.transpose(), -1 * I);
        A_x.col(18) = p_uv;

        A_s.block(0, 0, 3, 3) = -I;
        // std::cout << "err_p = " << err_p.transpose() << std::endl;
        return A_tk;
    }

    void Solver::update_Q_matrix(Eigen::MatrixXd& Q){
        int size = At.size() - 1;
        // for(int i = 0; i < size; i++){
        //     // std::cout << "+++++++++++++++i = "<<i<<"+++++++++++++++++++"<<std::endl;
        //     update_At_vector(i, At[i]);
        //     // print_DenseMatrix_asSym(At[i], "At[i]"); //debug
        //     Eigen::MatrixXd tmp = At[i].transpose() * At[i] * pow(weight, (N_ - 1 - i));
        //     Q += tmp;
        // }
        #pragma omp parallel
        {
            Eigen::MatrixXd local_Q = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());

            // 并行累加
            #pragma omp for
            for (int i = 0; i < size; i++) {
                update_At_vector(i, At[i]);
                Eigen::MatrixXd tmp = At[i].transpose() * At[i] * weight;

                local_Q += tmp;
            }

            // 将局部结果合并到全局结果 Q
            #pragma omp critical
            {
                Q += local_Q;
            }
        }
    }

    void Solver::update_Q_bar_matrix(Eigen::MatrixXd& Q){
        int size = At_bar.size() - 1;
        #pragma omp parallel
        {
            Eigen::MatrixXd local_Q = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());

            // 并行累加
            #pragma omp for
            for (int i = 0; i < size; i++) {
                Eigen::MatrixXd tmp = At_bar[i].transpose() * At_bar[i] * weight;

                local_Q += tmp;
            }

            // 将局部结果合并到全局结果 Q
            #pragma omp critical
            {
                Q += local_Q;
            }
        }
    }

    int Solver::optimize_vision(const Eigen::Vector3d& p_uv, const Eigen::Vector3d& p_cc, const Eigen::Vector3d& v_cc, Eigen::VectorXd& x){
        // std::cout << "________________entering solver______________" <<std::endl;
        auto tic = std::chrono::steady_clock::now();

        int n = At_bar.size();
        Eigen::MatrixXd A_latest;
        A_latest = get_At_bar_matrix(p_uv, p_cc, v_cc);
        Eigen::Block<Eigen::MatrixXd> A_lx = A_latest.block(0, 0, 3, 19);
        if(A_lx.isZero()){
            return 0;
        }

        if(n < N_){
            At_bar.emplace_back(A_latest);
            return 0;
        }else{
            At_bar.erase(At_bar.begin());
            n -= 1;
            At_bar.emplace_back(A_latest);
        }

        Eigen::MatrixXd Q = A_latest.transpose() * A_latest;
        // rank_count(A_latest.transpose(),"A_latest"); // debug
        update_Q_bar_matrix(Q);
        // int rank_Q = rank_count(Q,"Q"); // debug

        auto toc_1 = std::chrono::steady_clock::now();
        // std::cout << "dur_1 : " << (toc_1 - tic).count() * 1e-6 << "ms" << std::endl;

        Eigen::Block<Eigen::MatrixXd> Q_a = Q.block(0, 0, 19, 19);
        Eigen::Block<Eigen::MatrixXd> Q_b = Q.block(0, 19, 19, 3);
        Eigen::MatrixXd Q_c = Q.block(19, 19, 3, 3);
        Eigen::MatrixXd Q_c_inv;
        if(!get_inverse_Matrix(Q_c, Q_c_inv)){
            return -2;
        }

        // print_DenseMatrix_asSym(Q_a, "Q_a"); //debug
        // print_DenseMatrix_asSym(Q_b, "Q_b"); //debug
        // std::cout << "Q_a.det = " << Q_a.determinant() << std::endl;
        // rank_count(Q_a,"Q_a"); // debug
        // rank_count(Q_b,"Q_b"); // debug

        Eigen::MatrixXd Q_0_x = Q_a;
        Q_0_x = Q_0_x - Q_b * Q_c_inv * Q_b.transpose();
        // rank_count(Q_0_x,"Q_0_x"); // debug
        // print_DenseMatrix_asSym(Q_0_x, "Q_0_x"); //debug
        Q_0_x.triangularView<Eigen::StrictlyUpper>().setZero();

        // try{
        std::vector<int> rows;
        std::vector<int> cols;
        std::vector<double> vals;
        get_nonZero_vals(Q_0_x, rows, cols, vals);
        // print_vector("rows", rows);
        // print_vector("cols", cols);
        // print_vector("vals", vals); // debug
        SymMatrix Q_0 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());

        model.SetPsdObjective(Q_0 * Z, COPT_MINIMIZE);
        model.Solve();

        auto toc_2 = std::chrono::steady_clock::now();
        // std::cout << "dur_2 : " << (toc_2 - toc_1).count() * 1e-6 << "ms" << std::endl;

        // Output solution
        if(model.GetIntAttr(COPT_INTATTR_LPSTATUS) == COPT_LPSTATUS_OPTIMAL){
        std::cout << "\nOptimal objective value: " << model.GetDblAttr(COPT_DBLATTR_LPOBJVAL) << std::endl; // 目标函数最优值
        std::cout << std::endl;

        PsdVarArray psdvars = model.GetPsdVars();
        PsdVar psdvar = psdvars.GetPsdVar(0);
        int psdLen = psdvar.GetLen();
        int psdDim = psdvar.GetDim();

        std::vector<double> psdVal(psdLen);
        // // std::vector<double> psdDual(psdLen);

        /* Get flattened SDP primal/dual solution */
        psdvar.Get(COPT_DBLINFO_VALUE, psdVal.data(), psdLen); // 原变量
        // // psdvar.Get(COPT_DBLINFO_DUAL, psdDual.data(), psdLen); // 对偶变量

        int rankZ;
        Eigen::VectorXd z = revert_z_from_Z(psdVal, psdDim, rankZ);

        Eigen::VectorXd R(9);
        R << z(0), z(1), z(2), z(3), z(4), z(5), z(6), z(7), z(8);
        // std::cout << "R:" << R.transpose() << std::endl;
        Eigen::VectorXd tR(9);
        tR << z[9], z[10], z[11], z[12], z[13], z[14], z[15], z[16], z[17];
        Eigen::Matrix3d Rot = recoverRotation_from_vector(R);
        Eigen::Matrix3d tRot = recoverRotation_from_vector(tR);

        double t_d = std::cbrt(tRot.determinant());
        // double t_d = z[19];
        if(CONTINUES_ESTIMATE_){
            T_total += t_d;
        }
        

        std::cout << "Rot: " << std::endl;
        std::cout << Rot << std::endl;
        std::cout << "tRot: " << std::endl;
        std::cout << tRot << std::endl;
        /* check constrains */
        double t_2 = z[19] * z[19];
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
        Eigen::Vector3d R_c1 = Rot.col(0);
        Eigen::Vector3d R_c2 = Rot.col(1);
        Eigen::Vector3d R_c3 = Rot.col(2);
        Eigen::Vector3d tR_c1 = tRot.col(0);
        Eigen::Vector3d tR_c2 = tRot.col(1);
        Eigen::Vector3d tR_c3 = tRot.col(2);
        std::cout << "cons_1: " << std::endl;
        std::cout << Rot.transpose() * Rot - I << std::endl;
        std::cout << "cons_2: " << std::endl;
        std::cout << tRot.transpose() * tRot - t_2 * I << std::endl;
        std::cout << "cons_3: " << std::endl;
        std::cout << Rot * Rot.transpose() - I << std::endl;
        std::cout << "cons_4: " << std::endl;
        std::cout << tRot * tRot.transpose() - t_2 * I << std::endl;
        std::cout << "cons_5: " << (R_c1.cross(R_c2) - R_c3).transpose() << std::endl;
        std::cout << "cons_6: " << (R_c2.cross(R_c3) - R_c1).transpose() << std::endl;
        std::cout << "cons_7: " << (R_c3.cross(R_c1) - R_c2).transpose() << std::endl;
        std::cout << "cons_8: " << (tR_c1.cross(tR_c2) - z[19] * tR_c3).transpose() << std::endl;
        std::cout << "cons_9: " << (tR_c2.cross(tR_c3) - z[19] * tR_c1).transpose() << std::endl;
        std::cout << "cons_10: " << (tR_c3.cross(tR_c1) - z[19] * tR_c2).transpose() << std::endl;


        Eigen::Quaterniond q_cu(Rot);
        q_cu.normalize();

        // std::cout << "t_d:" << t_d << ", z(19):" << z(19) << std::endl;
        // std::cout << "cons_12:" << (tR * z(18) - R * t_d).norm() << std::endl;

        Eigen::VectorXd x_s = z.head(19);
        x_s = - Q_c_inv * Q_b.transpose() * x_s;
        // std::cout << "x_s:" << x_s.transpose() << std::endl;

        x << q_cu.w(), q_cu.x(), q_cu.y(), q_cu.z(), x_s(0), x_s(1), x_s(2), t_d, rankZ, T_total;
        // std::cout << "x:" << x.transpose() << std::endl;
        return 1;
        }
        // model.Interrupt();
        return -1;
    }

    int Solver::optimize(const Eigen::Vector3d& p_uu, const Eigen::Vector3d& p_cc, const Eigen::Quaterniond& q_uu, 
                        const Eigen::Vector3d& v_cc, const Eigen::Vector3d& b, Eigen::VectorXd& x){
        // std::cout << "________________entering solver______________" <<std::endl;
        auto tic = std::chrono::steady_clock::now();

        int n = At.size();
        Eigen::MatrixXd A_latest;
        
        if(n < N_){
            A_latest = get_At_matrix(p_uu, p_cc, q_uu, v_cc, b, n);
            At.emplace_back(A_latest);
            return 0;
        }else{
            At.erase(At.begin());
            n -= 1;
            A_latest = get_At_matrix(p_uu, p_cc, q_uu, v_cc, b, n);
            At.emplace_back(A_latest);
        }

        Eigen::MatrixXd Q = A_latest.transpose() * A_latest;
        // rank_count(A_latest.transpose(),"A_latest"); // debug
        update_Q_matrix(Q);
        // int rank_Q = rank_count(Q,"Q"); // debug

        auto toc_1 = std::chrono::steady_clock::now();
        // std::cout << "dur_1 : " << (toc_1 - tic).count() * 1e-6 << "ms" << std::endl;

        Eigen::Block<Eigen::MatrixXd> Q_a = Q.block(0, 0, 19, 19);
        Eigen::Block<Eigen::MatrixXd> Q_b = Q.block(0, 19, 19, 3 + N_);
        Eigen::Block<Eigen::MatrixXd> Q_c = Q.block(19, 19, 3 + N_, 3 + N_);
        Eigen::MatrixXd Q_c_inv;
        if(!get_inverse_Matrix(Q_c, Q_c_inv)){
            return -2;
        }

        // std::cout << "Q_c * Q_c_inv :" << std::endl;
        // std::cout << Q_c * Q_c_inv << std::endl;
        // Eigen::MatrixXd Q_c_inv = get_inverse_Matrix(Q_c);
        // print_DenseMatrix_asSym(Q_a, "Q_a"); //debug
        // print_DenseMatrix_asSym(Q_b, "Q_b"); //debug
        // std::cout << "Q_a.det = " << Q_a.determinant() << std::endl;
        // rank_count(Q_a,"Q_a"); // debug
        // rank_count(Q_b,"Q_b"); // debug

        Eigen::MatrixXd Q_0_x = Q_a - Q_b * Q_c_inv * Q_b.transpose();
        // rank_count(Q_0_x,"Q_0_x"); // debug
        // print_DenseMatrix_asSym(Q_0_x, "Q_0_x"); //debug
        Q_0_x.triangularView<Eigen::StrictlyUpper>().setZero();

        // try{
            std::vector<int> rows;
            std::vector<int> cols;
            std::vector<double> vals;
            get_nonZero_vals(Q_0_x, rows, cols, vals);
            SymMatrix Q_0 = model.AddSparseMat(20, vals.size(), rows.data(), cols.data(), vals.data());

            model.SetPsdObjective(Q_0 * Z, COPT_MINIMIZE);
            model.Solve();

            auto toc_2 = std::chrono::steady_clock::now();
            // std::cout << "dur_2 : " << (toc_2 - toc_1).count() * 1e-6 << "ms" << std::endl;

            // Output solution
            if(model.GetIntAttr(COPT_INTATTR_LPSTATUS) == COPT_LPSTATUS_OPTIMAL){
                // std::cout << "\nOptimal objective value: " << model.GetDblAttr(COPT_DBLATTR_LPOBJVAL) << std::endl; // 目标函数最优值
                // std::cout << std::endl;

                PsdVarArray psdvars = model.GetPsdVars();
                PsdVar psdvar = psdvars.GetPsdVar(0);
                int psdLen = psdvar.GetLen();
                int psdDim = psdvar.GetDim();

                std::vector<double> psdVal(psdLen);
                // // std::vector<double> psdDual(psdLen);

                /* Get flattened SDP primal/dual solution */
                psdvar.Get(COPT_DBLINFO_VALUE, psdVal.data(), psdLen); // 原变量
                // // psdvar.Get(COPT_DBLINFO_DUAL, psdDual.data(), psdLen); // 对偶变量
                
                int rankZ;
                Eigen::VectorXd z = revert_z_from_Z(psdVal, psdDim, rankZ);

                Eigen::VectorXd R(9);
                R << z(0), z(1), z(2), z(3), z(4), z(5), z(6), z(7), z(8);
                // std::cout << "R:" << R.transpose() << std::endl;
                Eigen::VectorXd tR(9);
                tR << z[9], z[10], z[11], z[12], z[13], z[14], z[15], z[16], z[17];
                Eigen::Matrix3d Rot = recoverRotation_from_vector(R);
                Eigen::Matrix3d tRot = recoverRotation_from_vector(tR);

                double t_d = std::cbrt(tRot.determinant());
                // double t_d = z[19];
                if(CONTINUES_ESTIMATE_){
                    T_total += t_d;
                }
                Eigen::Quaterniond q_cu(Rot);
                q_cu.normalize();

                // std::cout << "t_d:" << t_d << ", z(19):" << z(19) << std::endl;
                // std::cout << "cons_12:" << (tR * z(18) - R * t_d).norm() << std::endl;

                Eigen::VectorXd x_s = z.head(19);
                x_s = - Q_c_inv * Q_b.transpose() * x_s;
                // std::cout << "x_s:" << x_s.transpose() << std::endl;
                
                x << q_cu.w(), q_cu.x(), q_cu.y(), q_cu.z(), x_s(0), x_s(1), x_s(2), t_d, rankZ, T_total;
                // std::cout << "x:" << x.transpose() << std::endl;

                // std::cout << "solving duration : " << model.GetDblAttr(COPT_DBLATTR_SOLVINGTIME) << " s" << std::endl;
                return 1;
            }
            // model.Interrupt();
            return -1;
        // }catch(CoptException e){
        //     std::cout << "Error Code = " << e.GetCode() << std::endl;
        //     std::cout << e.what() << std::endl;
        // }
    }

} // namespace estimate
