// 局部路径求解器抽象接口
#ifndef LOCAL_PATH_SOLVER_INTERFACE_H_
#define LOCAL_PATH_SOLVER_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"
#include "common_type.h"
#include <vector>
#include "config_reader.h"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/local_path_point.hpp"
#include "base_msgs/msg/referline_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "decision_center.h"
#include <cmath>
#include <limits>

namespace Planning
{
  // 求解代价分量
  struct CostBreakdown
  {
    float64 lateral = 0.0;
    float64 smoothness = 0.0;
    float64 curvature = 0.0;
    float64 safety = 0.0;
    float64 corridor = 0.0;
    float64 slope = 0.0;
  };

  // 求解器输出
  struct SolverResult
  {
    std::vector<float64> lateralOffsets;  // 横向偏移 l_i
    bool feasible;                        // 可行性标志
    CostBreakdown cost;
  };

  // 决策点（type: 0 unknown / 1 left / 2 right / 3 stop / 4 start / 5 end）
  struct PathDecisionPoint
  {
    float64 s;    // unit: m
    float64 l;    // unit: m
    uint8 type;
  };

  // 一次求解的输入
  struct LocalPathProblem
  {
    std::vector<float64> s;              // 网格点 s
    std::vector<float64> x;              // 网格点 x
    std::vector<float64> y;              // 网格点 y
    std::vector<float64> theta;          // 网格点切向角
    std::vector<PathDecisionPoint> pathDecisionPoints;  // 决策点序列
    std::vector<float64> lowerBoundary;  // 走廊下界
    std::vector<float64> upperBoundary;  // 走廊上界
    float64 egoWidth;                    // unit: m
    float64 weightSmooth, weightSlope, weightOffset;  // w1/w2/w3
  };

  class LocalPathSolverInterface
  {
  public:
    virtual ~LocalPathSolverInterface() = default;
    // solve the local path problem
    virtual SolverResult solve(const LocalPathProblem& problem, int timeBudgetMs, bool warmStart, int seed) = 0;
  };

} // namespace Planning
#endif // LOCAL_PATH_SOLVER_INTERFACE_H_
