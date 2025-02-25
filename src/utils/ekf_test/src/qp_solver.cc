#include <ekf_test/qp_solver.hpp>
#include <vector>
#include <iostream>

namespace QP_Solver{
    template<typename T>
    static void printVec(const int& size, const T* vec){
        for(int i = 0; i < size; i++){
            std::cout << vec[i] << " ";
        }
        std::cout << std::endl;
    }

    static void printCscMat(OSQPCscMatrix* M_csc){
        printf("m:%d, n:%d, nzmax:%d, nz:%d\n",M_csc->m, M_csc->n, M_csc->nzmax, M_csc->nz);
        printf("p_ad:%p, x_ad:%p, i_ad:%p\n",M_csc->p, M_csc->x, M_csc->i);
        std::cout << "p: ";
        printVec(M_csc->n + 1, M_csc->p);
        std::cout << "i: ";
        printVec(M_csc->nzmax, M_csc->i);
        std::cout << "x: ";
        printVec(M_csc->nzmax, M_csc->x);
        std::cout << "-----------------------------------"<<std::endl;
    }

    void Update_vec::Get_csc_matrix(const int& row_s, // size of row
                                    const int& col_s, // size of col
                                    const Eigen::MatrixXd& Mat, // init matrix with eigen form
                                    OSQPCscMatrix* &M_csc){
        OSQPInt* M_p = (OSQPInt *)malloc(sizeof(OSQPInt) * (col_s + 1)); // the location that nonzero entries of each col in vec_i and vec_x
        std::vector<int> vec_i; // store row indices of nonzero entries
        std::vector<double> vec_x; // store nonzero entries
        int last_col = -1;
        for(int j = 0; j < col_s; j++){
            for(int i = 0; i < row_s; i++){
                if(j > i){
                    continue; // make matrix triplet
                }
                if(Mat(i,j) != 0){
                    if(last_col != j){
                        last_col = j;
                        M_p[j] = vec_x.size();
                    }
                    vec_x.push_back(Mat(i,j));
                    vec_i.push_back(i);
                }
            }
        }
        int num_size = vec_x.size();
        M_p[n_] = num_size;
        OSQPFloat* M_x = (OSQPFloat *)malloc(sizeof(OSQPFloat) * num_size);
        std::copy(vec_x.begin(), vec_x.end(), M_x);
        OSQPInt* M_i = (OSQPInt *)malloc(sizeof(OSQPInt) * num_size);
        std::copy(vec_i.begin(), vec_i.end(), M_i);

        M_csc = (OSQPCscMatrix *)malloc(sizeof(OSQPCscMatrix));

        // std::cout << "address: "<< M_csc <<std::endl;
        // printf("row:%d, col:%d, num_size:%d\n",row_s, col_s, num_size);
        // std::cout << "M_x: ";
        // printVec(num_size, M_x);
        // std::cout << "M_i: ";
        // printVec(num_size, M_i);
        // std::cout << "M_p: ";
        // printVec(col_s + 1, M_p);

        /**
         * Populates a Compressed-Column-Sparse matrix from existing arrays
         (just assigns the pointers - no malloc or copying is done)
        * @param  M     Matrix pointer
        * @param  m     First dimension
        * @param  n     Second dimension
        * @param  nzmax Maximum number of nonzero elements
        * @param  x     Vector of data
        * @param  i     Vector of row indices
        * @param  p     Vector of column pointers
        */
        csc_set_data(M_csc, row_s, col_s, num_size, M_x, M_i, M_p);
    }

    template <typename TYPE>
    void Update_vec::Get_vector(const Eigen::VectorXd& v, TYPE* &v_){
        if(v_ == nullptr){
            TYPE* v_tmp = (TYPE *)malloc(v.size() * sizeof(TYPE));
            v_ = v_tmp;
        } 
        for(int i = 0; i < v.size(); i++){
            v_[i] = v(i);
        }
    }

    OSQPInt Update_vec::init(const int& var_n, 
                            const int& cons_n,
                            const Eigen::MatrixXd& P,
                            const Eigen::MatrixXd& A,
                            const Eigen::VectorXd& q,
                            const Eigen::VectorXd& l,
                            const Eigen::VectorXd& u){
        n_ = var_n;
        m_ = cons_n;
        exitflag = 0;

        /* Load settings */
        settings = (OSQPSettings *)malloc(sizeof(OSQPSettings));
        if(settings) osqp_set_default_settings(settings);
        settings->verbose = false; // close printing

        /* Set csc matrix P */
        Get_csc_matrix(n_, n_, P, P_);

        /* Set csc matrix A */
        Get_csc_matrix(m_, n_, A, A_);

        /* Set vector */
        Get_vector(q, q_);
        Get_vector(l, l_);
        Get_vector(u, u_);

        // std::cout << "P:" << std::endl;
        // std::cout << P << std::endl;
        // printCscMat(P_);
        // std::cout << "A:" << std::endl;
        // std::cout << A << std::endl;
        // printCscMat(A_);
        // std::cout << "l:" << l.transpose() <<std::endl;
        // std::cout << "l_: ";
        // printVec(l.size(), l_);
        // std::cout << "q:" << q.transpose() <<std::endl;
        // std::cout << "q_: ";
        // printVec(q.size(), q_);
        // std::cout << "u:" << u.transpose() <<std::endl;
        // std::cout << "u_: ";
        // printVec(u.size(), u_);

        exitflag = osqp_setup(&solver, P_, q_, A_, l_, u_, m_, n_, settings);

        if(!exitflag) exitflag = osqp_solve(solver);

        return exitflag;
    }

    OSQPInt Update_vec::Count_x(const Eigen::VectorXd& q, const Eigen::VectorXd& l, const Eigen::VectorXd& u){
        if(exitflag){
            return exitflag;
        }
        Get_vector(q, q_);
        Get_vector(l, l_);
        Get_vector(u, u_);
        exitflag = osqp_update_data_vec(solver, q_, l_, u_);
        if(!exitflag) osqp_solve(solver);

        return exitflag;
    }

    Update_vec::~Update_vec(){
        osqp_cleanup(solver);
        if(A_) free(A_);
        if(P_) free(P_);
        if(q_) free(q_);
        if(l_) free(l_);
        if(u_) free(u_);
        if(settings) free(settings);
    }
}