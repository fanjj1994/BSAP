#include "reference_line_creator.h"

namespace Planning
{
  ReferenceLineCreator::ReferenceLineCreator()
  {
    RCLCPP_INFO(rclcpp::get_logger("referenceline"), "referenceline created");
  }

  base_msgs::msg::Referline
  ReferenceLineCreator::createReferenceLine(const nav_msgs::msg::Path &globalPath,
                                            const geometry_msgs::msg::PoseStamped &vehiclePose)
  {
    // 如果全局路径为空，返回空参考线
    if (globalPath.poses.empty())
    {
      return referenceLine;
    }
    // 查找匹配点
    matchPointIndex = Curve::findMatchPointIndex(globalPath, lastMatchPointIndex, vehiclePose);
    lastMatchPointIndex = matchPointIndex; // 更新上一帧匹配点
    if (matchPointIndex < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("referenceline"), "find matchPointindex failed");
      return referenceLine;
    }
    // 计算最前点和最后点的下标
    const int16 globalPathSize = static_cast<int16>(globalPath.poses.size());
    frontIndex =(matchPointIndex + FRONT_SIZE >= globalPathSize - 1) ? (globalPathSize - 1) : (matchPointIndex + FRONT_SIZE);
    backIndex = (matchPointIndex - BACK_SIZE <= 0) ? 0 : (matchPointIndex - BACK_SIZE);

    // 填充进参考线
    referenceLine.header.frame_id = "world";
    referenceLine.header.stamp = rclcpp::Clock().now();
    referenceLine.refer_line.clear();
    base_msgs::msg::ReferlinePoint pointTmp;
    for (int16 i = backIndex; i <= frontIndex; i++)
    {
      pointTmp.pose = globalPath.poses[i];
      referenceLine.refer_line.emplace_back(pointTmp);
    }

    // 计算投影点参数
    Curve::calculateProjectedPointParameters(referenceLine);
    return referenceLine;
  }

  nav_msgs::msg::Path ReferenceLineCreator::referenceLineToRviz()
  {
    referlineRviz.header = referenceLine.header;
    referlineRviz.poses.clear();

    geometry_msgs::msg::PoseStamped pointTmp;
    for (const auto &point : referenceLine.refer_line)
    {
      pointTmp.header = referlineRviz.header;
      pointTmp.pose = point.pose.pose;
      referlineRviz.poses.emplace_back(pointTmp);
    }
    return referlineRviz;
  }

} // namespace Planning