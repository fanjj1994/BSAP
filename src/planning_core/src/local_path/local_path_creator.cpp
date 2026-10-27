#include "local_path_creator.h"
#include "qp_local_path_solver.h"

namespace Planning
{
  LocalPathCreator::LocalPathCreator()
  {
    // 读取配置文件
    localPathConfigReader = std::make_unique<ConfigReader>();
    localPathConfigReader->readLocalPathConfig();

    // 创建求解器
    localPathSolver = std::make_shared<QPLocalPathSolver>();

    // 初始化局部路径消息头
    localPath.header.frame_id = localPathConfigReader->getPNCMap().frame_;
  }

  base_msgs::msg::LocalPath LocalPathCreator::createLocalPath(const base_msgs::msg::Referline& referenceLine,
                                                              const std::vector<SLPoint>& decisionPoints)
  {
    // 清空旧路径
    localPath.local_path.clear();

    const uint32 pointSize = static_cast<uint32>(referenceLine.refer_line.size());
    if (pointSize < 3U)
    {
      RCLCPP_ERROR(rclcpp::get_logger("local_path"), "reference line too short");
      return localPath;
    }

    LocalPathProblem problem;
    problem.s.reserve(pointSize);
    problem.x.reserve(pointSize);
    problem.y.reserve(pointSize);
    problem.theta.reserve(pointSize);
    problem.lowerBoundary.reserve(pointSize);
    problem.upperBoundary.reserve(pointSize);
    problem.egoWidth = localPathConfigReader->getEgoCar().width_;
    for (uint32 i = 0U; i < pointSize; i++)
    {
      const auto& refPoint = referenceLine.refer_line[i];
      problem.s.push_back(refPoint.rs);
      problem.x.push_back(refPoint.pose.pose.position.x);
      problem.y.push_back(refPoint.pose.pose.position.y);
      problem.theta.push_back(refPoint.rtheta);
      problem.lowerBoundary.push_back(-(localPathConfigReader->getPNCMap().lane_width_ * 0.5) + problem.egoWidth / 2.0);
      problem.upperBoundary.push_back(localPathConfigReader->getPNCMap().lane_width_ * 1.5 - problem.egoWidth / 2.0);
    }

    problem.weightSmooth = localPathConfigReader->getLocalPath().weight_smooth_;
    problem.weightSlope = localPathConfigReader->getLocalPath().weight_slope_;
    problem.weightOffset = localPathConfigReader->getLocalPath().weight_offset_;

    problem.pathDecisionPoints.reserve(decisionPoints.size());
    for (const auto& slPoint : decisionPoints)
    {
      PathDecisionPoint pod;
      pod.s = slPoint.s;
      pod.l = slPoint.l;
      pod.type = static_cast<uint8>(slPoint.type);
      problem.pathDecisionPoints.push_back(pod);
    }

    const SolverResult result = localPathSolver->solve(problem, 100, false, 0);

    localPath.local_path.reserve(pointSize);
    for (uint32 i = 0U; i < pointSize; i++)
    {
      base_msgs::msg::LocalPathPoint point;
      const auto& refPoint = referenceLine.refer_line[i];
      const float64 lateral = (i < result.lateralOffsets.size()) ? result.lateralOffsets[i] : 0.0;
      const float64 theta = refPoint.rtheta;

      point.pose.pose.position.x = refPoint.pose.pose.position.x - lateral * std::sin(theta);
      point.pose.pose.position.y = refPoint.pose.pose.position.y + lateral * std::cos(theta);
      point.s = problem.s[i];
      point.l = lateral;
      point.theta = theta;
      localPath.local_path.push_back(point);
    }
    localPath.header.stamp = rclcpp::Time(0, 0);
    return localPath;
  }

  nav_msgs::msg::Path LocalPathCreator::localPathToRviz()
  {
    localPathRviz.poses.clear();
    localPathRviz.header = localPath.header;
    for (const auto& point : localPath.local_path)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = localPath.header;
      pose.pose.position = point.pose.pose.position;
      pose.pose.orientation = point.pose.pose.orientation;
      localPathRviz.poses.push_back(pose);
    }
    return localPathRviz;
  }

} // namespace Planning