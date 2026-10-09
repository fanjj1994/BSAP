#include "reference_line_smoother.h"

namespace Planning
{
  ReferenceLineSmoother::ReferenceLineSmoother()
  {
    RCLCPP_INFO(rclcpp::get_logger("referenceline"), "ReferenceLineSmoother is running");
  }

  void ReferenceLineSmoother::SmoothReferenceLine(base_msgs::msg::Referline &referline)
  {
    const uint16 ptNum = referline.refer_line.size();
    if (ptNum < 3)
    {
      RCLCPP_WARN(rclcpp::get_logger("referenceline"),
                  "ReferenceLineSmoother: referline size is too small, no need to smooth");
      return;
    }

    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d W1 = 2.0 * w1 * I;
    Eigen::Matrix2d W2 = 2.0 * w2 * I;
    Eigen::Matrix2d W3 = 2.0 * w3 * I;

    Eigen::Matrix2d block1 = W1 + W2 + W3;
    Eigen::Matrix2d block2 = -2.0 * W1 - W2;
    Eigen::Matrix2d block3 = -4.0 * W1 - W2;
    Eigen::Matrix2d block4 = 5.0 * W1 + 2.0 * W2 + W3;
    Eigen::Matrix2d block5 = 6.0 * W1 + 2.0 * W2 + W3;
    Eigen::Matrix2d block6 = 4.0 * W1 + 2.0 * W2 + W3;

    Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(2U * ptNum, 2U * ptNum);

    if (ptNum == 3)
    {
      P_tmp.block<2, 2>(0, 0) = block1;
      P_tmp.block<2, 2>(0, 2) = block2;
      P_tmp.block<2, 2>(0, 4) = W1;
      P_tmp.block<2, 2>(2, 2) = block6;
      P_tmp.block<2, 2>(2, 4) = block2;
      P_tmp.block<2, 2>(4, 4) = block1;
    }
    else
    {
      for (uint16 i = 0; i < ptNum; ++i)
      {
        if (i == 0)
        {
          P_tmp.block<2, 2>(2 * i, 2 * i) = block1;
          P_tmp.block<2, 2>(2 * i, 2 * (i + 1)) = block2;
          P_tmp.block<2, 2>(2 * i, 2 * (i + 2)) = W1;
        }
        else if (i == 1)
        {
          P_tmp.block<2, 2>(2 * i, 2 * i) = block4;
          P_tmp.block<2, 2>(2 * i, 2 * (i + 1)) = block3;
          P_tmp.block<2, 2>(2 * i, 2 * (i + 2)) = W1;
        }
        else if (i == ptNum - 2)
        {
          P_tmp.block<2, 2>(2 * i, 2 * i) = block4;
          P_tmp.block<2, 2>(2 * i, 2 * (i + 1)) = block2;
        }
        else if (i == ptNum - 1)
        {
          P_tmp.block<2, 2>(2 * i, 2 * i) = block1;
        }
        else
        {
          P_tmp.block<2, 2>(2 * i, 2 * i) = block5;
          P_tmp.block<2, 2>(2 * i, 2 * (i + 1)) = block3;
          P_tmp.block<2, 2>(2 * i, 2 * (i + 2)) = W1;
        }
      }
    }
    P_tmp = P_tmp.selfadjointView<Eigen::Upper>();
    Eigen::SparseMatrix<float64> P = P_tmp.sparseView();

    Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(2 * ptNum, 2 * ptNum);
    Eigen::SparseMatrix<double> A = A_tmp.sparseView();
    Eigen::VectorXd X(2 * ptNum);
    for (uint16 i = 0; i < ptNum; ++i)
    {
      X(2 * i) = referline.refer_line[i].pose.pose.position.x;
      X(2 * i + 1) = referline.refer_line[i].pose.pose.position.y;
    }

    Eigen::VectorXd Q = -2.0 * X;
    Eigen::VectorXd buff = Eigen::VectorXd::Constant(2 * ptNum, 0.2);
    buff(0) = buff(1) = buff(2 * ptNum - 2) = buff(2 * ptNum - 1) = 0.0;
    Eigen::VectorXd lowerBound = X - buff;
    Eigen::VectorXd upperBound = X + buff;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(2 * ptNum);
    solver.data()->setNumberOfConstraints(2 * ptNum);
    if (!solver.data()->setHessianMatrix(P))
    {
      return;
    }
    if (!solver.data()->setGradient(Q))
    {
      return;
    }
    if (!solver.data()->setLinearConstraintsMatrix(A))
    {
      return;
    }
    if (!solver.data()->setLowerBound(lowerBound))
    {
      return;
    }
    if (!solver.data()->setUpperBound(upperBound))
    {
      return;
    }
    if (!solver.initSolver())
    {
      return;
    }
    Eigen::VectorXd qpSolution;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
      return;
    }
    qpSolution = solver.getSolution();

    for (uint16 i = 0; i < ptNum; ++i)
    {
      referline.refer_line[i].pose.pose.position.x = qpSolution(2 * i);
      referline.refer_line[i].pose.pose.position.y = qpSolution(2 * i + 1);
    }
  }
} // namespace Planning