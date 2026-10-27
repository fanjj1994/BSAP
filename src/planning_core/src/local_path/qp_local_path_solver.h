#ifndef QP_LOCAL_PATH_SOLVER_H_
#define QP_LOCAL_PATH_SOLVER_H_

#include "rclcpp/rclcpp.hpp"
#include "local_path_solver_interface.h"
#include "common_type.h"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>

namespace Planning
{
    // QP 局部路径求解器
    class QPLocalPathSolver : public LocalPathSolverInterface
    {
    public:
        QPLocalPathSolver() = default;
        ~QPLocalPathSolver() = default;

        // solve the local path problem by QP
        SolverResult solve(const LocalPathProblem& problem, int timeBudgetMs, bool warmStart, int seed)override;
    
    private:
        // 拼目标函数矩阵
        void buildObjective(const LocalPathProblem& problem, Eigen::SparseMatrix<float64>& hessian, Eigen::Matrix<float64, Eigen::Dynamic, 1>& gradient);
        // 拼约束矩阵
        void buildConstraints(const LocalPathProblem& problem, Eigen::SparseMatrix<float64>& linearMatrix, Eigen::Matrix<float64, Eigen::Dynamic,1>& lowerBound, Eigen::Matrix<float64, Eigen::Dynamic, 1>& upperBound);
    };

}  // namespace Planning
#endif  // QP_LOCAL_PATH_SOLVER_H_
