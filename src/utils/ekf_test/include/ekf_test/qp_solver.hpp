#include <osqp.h>
#include <stdlib.h>
#include <Eigen/Core>

namespace QP_Solver{
    // solve problems as:
    //       min     1/2 * x.transpose() * P * x + q.transpose() * x
    //       sub to. l <= A * x <= u
    // note: P and A must have dimension as a * b
    class Update_vec{
        // update vector q, l, u
        private:
        void Get_csc_matrix(const int& row_s, // size of row
                            const int& col_s, // size of col
                            const Eigen::MatrixXd& Mat, // init matrix with eigen form
                            OSQPCscMatrix* &M_csc);

        template <typename TYPE>
        void Get_vector(const Eigen::VectorXd& v, TYPE* &v_);

        public:
        OSQPInt n_,m_;
        OSQPFloat* q_;
        OSQPFloat* l_;
        OSQPFloat* u_;
        OSQPCscMatrix *P_;
        OSQPCscMatrix *A_;

        OSQPSolver *solver;
        OSQPSettings *settings;

        /* 
            flag of solver :
            No error                                        OSQP_NO_ERROR                           0
            Data validation failed                          OSQP_DATA_VALIDATION_ERROR              1
            Settings validation failed                      OSQP_PRIMAL_INFEASIBLE                  2
            Linear system solver initialization failed      OSQP_PRIMAL_INFEASIBLE_INACCURATE       3
            Non convex problem detected                     OSQP_DUAL_INFEASIBLE                    4
            Memory allocation error                         OSQP_DUAL_INFEASIBLE_INACCURATE         5
            Workspace not initialized                       OSQP_MAX_ITER_REACHED                   6
            Error loading algebra library                   OSQP_TIME_LIMIT_REACHED                 7
            Error opening file for writing                  OSQP_NON_CVX                            8
            Error validating given code generation defines  OSQP_SIGINT                             9
            Solver data not initialized                     OSQP_UNSOLVED                           10
            Function not implemented in current algebra     OSQP_FUNC_NOT_IMPLEMENTED               11
        */
        OSQPInt exitflag;

        OSQPInt init(const int& var_n, 
                   const int& cons_n,
                   const Eigen::MatrixXd& P,
                   const Eigen::MatrixXd& A,
                   const Eigen::VectorXd& q,
                   const Eigen::VectorXd& l,
                   const Eigen::VectorXd& u);

        ~Update_vec();

        OSQPInt Count_x(const Eigen::VectorXd& q, const Eigen::VectorXd& l, const Eigen::VectorXd& u);
    };
}