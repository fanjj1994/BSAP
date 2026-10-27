#include "qp_local_path_solver.h"

namespace Planning
{
  // 拼目标函数矩阵
  void QPLocalPathSolver::buildObjective(const LocalPathProblem& problem, Eigen::SparseMatrix<float64>& hessian,
                                         Eigen::Matrix<float64, Eigen::Dynamic, 1>& gradient)
  {
    const uint32 n = static_cast<uint32>(problem.s.size());
    const float64 w1 = problem.weightSmooth;
    const float64 w2 = problem.weightSlope;
    const float64 w3 = problem.weightOffset;

    std::vector<Eigen::Triplet<float64>> triplets;

    for (uint32 i = 2U; i < n; i++)
    {
      const uint32 idx[3] = { i - 2U, i - 1U, i };
      const float64 coef[3] = { 1.0, -2.0, 1.0 };
      for (uint32 a = 0U; a < 3U; a++)
      {
        for (uint32 b = 0U; b < 3U; b++)
        {
          triplets.emplace_back(idx[a], idx[b], w1 * coef[a] * coef[b]);
        }
      }
    }

    for (uint32 i = 1U; i < n; i++)
    {
      const uint32 idx[2] = { i - 1U, i };
      const float64 coef[2] = { -1.0, 1.0 };
      for (uint32 a = 0U; a < 2U; a++)
      {
        for (uint32 b = 0U; b < 2U; b++)
        {
          triplets.emplace_back(idx[a], idx[b], w2 * coef[a] * coef[b]);
        }
      }
    }

    for (uint32 i = 0U; i < n; i++)
    {
      triplets.emplace_back(i, i, w3);
    }

    hessian.resize(n, n);
    hessian.setFromTriplets(triplets.begin(), triplets.end());
    gradient = Eigen::Matrix<float64, Eigen::Dynamic, 1>::Zero(n);
  }

  // 拼约束矩阵
  void QPLocalPathSolver::buildConstraints(const LocalPathProblem& problem, Eigen::SparseMatrix<float64>& linearMatrix,
                                           Eigen::Matrix<float64, Eigen::Dynamic, 1>& lowerBound,
                                           Eigen::Matrix<float64, Eigen::Dynamic, 1>& upperBound)
  {
    const uint32 n = static_cast<uint32>(problem.s.size());
    const uint32 corridorRows = n;
    const uint32 decisionRows = problem.pathDecisionPoints.size();
    const uint32 firstPointRows = 1U;
    const uint32 totalRows = corridorRows + decisionRows + firstPointRows;

    std::vector<Eigen::Triplet<float64>> triplets;
    lowerBound.resize(totalRows);
    upperBound.resize(totalRows);
    // 防重复跳过的行留 [0,0]：先整体置零，未赋值行成为无害的 l=0 等式
    lowerBound.setZero();
    upperBound.setZero();
    uint32 row = 0U;
    triplets.emplace_back(row, 0U, 1.0);
    lowerBound(row) = 0.0;
    upperBound(row) = 0.0;
    row++;
    for (uint32 i = 0U; i < n; i++)
    {
      triplets.emplace_back(row, i, 1.0);
      lowerBound(row) = problem.lowerBoundary[i];
      upperBound(row) = problem.upperBoundary[i];
      row++;
    }
    for (const auto& dp : problem.pathDecisionPoints)
    {
      uint32 j = 0U;
      float64 minDist = std::numeric_limits<float64>::max();
      for (uint32 i = 0U; i < n; i++)
      {
        const float64 dist = std::fabs(problem.s[i] - dp.s);
        if (dist < minDist)
        {
          minDist = dist;
          j = i;
        }
      }
      if (j == 0U && dp.l == 0.0)
      {
        continue;
      }
      triplets.emplace_back(row, j, 1.0);
      lowerBound(row) = dp.l;
      upperBound(row) = dp.l;
      row++;
    }
    linearMatrix.resize(totalRows, n);
    linearMatrix.setFromTriplets(triplets.begin(), triplets.end());
  }

  // solver the local path problem by QP
  SolverResult QPLocalPathSolver::solve(const LocalPathProblem& problem, int timeBudgetMs, bool warmStart, int seed)
  {
    SolverResult result;
    const uint32 n = static_cast<uint32>(problem.s.size());
    if (n < 3U)
    {
      result.feasible = false;
      result.lateralOffsets.clear();
      return result;
    }
    Eigen::SparseMatrix<float64> hessian;
    Eigen::Matrix<float64, Eigen::Dynamic, 1> gradient;
    buildObjective(problem, hessian, gradient);
    Eigen::SparseMatrix<float64> linearMatrix;
    Eigen::Matrix<float64, Eigen::Dynamic, 1> lowerBound;
    Eigen::Matrix<float64, Eigen::Dynamic, 1> upperBound;
    buildConstraints(problem, linearMatrix, lowerBound, upperBound);

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setScaledTerimination(true);
    // setter 顺序照 smoother：先声明尺寸，再塞数据，逐一检查返回值
    solver.data()->setNumberOfVariables(static_cast<int>(n));
    solver.data()->setNumberOfConstraints(static_cast<int>(linearMatrix.rows()));
    if (!solver.data()->setHessianMatrix(hessian))
    {
      result.feasible = false;
      result.lateralOffsets.clear();
      return result;
    }
    if (!solver.data()->setGradient(gradient))
    {
      result.feasible = false;
      result.lateralOffsets.clear();
      return result;
    }
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
    {
      result.feasible = false;
      result.lateralOffsets.clear();
      return result;
    }
    if (!solver.data()->setLowerBound(lowerBound))
    {
      result.feasible = false;
      result.lateralOffsets.clear();
      return result;
    }
    if (!solver.data()->setUpperBound(upperBound))
    {
      result.feasible = false;
      result.lateralOffsets.clear();
      return result;
    }

    if (!solver.initSolver())
    {
      result.feasible = false;
      result.lateralOffsets.clear();
      return result;
    }
    if (!solver.solve())
    {
      result.feasible = false;
      result.lateralOffsets.clear();
      return result;
    }

    const auto solution = solver.getSolution();
    result.lateralOffsets.resize(n);
    for (uint32 i = 0U; i < n; i++)
    {
      result.lateralOffsets[i] = solution(static_cast<int>(i));
    }
    result.feasible = true;

    const float64 w1 = problem.weightSmooth;
    const float64 w2 = problem.weightSlope;
    const float64 w3 = problem.weightOffset;
    for (uint i = 0U; i < n; i++)
    {
      const float64 li = result.lateralOffsets[i];
      result.cost.lateral += w3 * li * li;
      if (i >= 1U)
      {
        const float64 li_1 = result.lateralOffsets[i - 1U];
        result.cost.slope += w2 * (li - li_1) * (li - li_1);
      }
      if (i >= 2U)
      {
        const float64 li_1 = result.lateralOffsets[i - 1U];
        const float64 li_2 = result.lateralOffsets[i - 2U];
        const float64 d2 = li - 2 * li_1 + li_2;
        result.cost.smoothness += w1 * d2 * d2;
      }
    }
    result.cost.curvature = 0.0;
    result.cost.corridor = 0.0;

    (void)timeBudgetMs;
    (void)warmStart;
    (void)seed;

    return result;
  }
} // namespace Planning