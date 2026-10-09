#include "curve.h"

namespace Planning
{
  // 查找匹配点下标
  int16 Planning::Curve::findMatchPointIndex(const nav_msgs::msg::Path &path, const int16 lastMatchPointIndex,
                                             const geometry_msgs::msg::PoseStamped &targetPoint)
  {
    const int16 pathSize = path.poses.size();

    if (pathSize <= 1)
    {
      return pathSize - 1;
    }
    float64 minDis = std::numeric_limits<float64>::max();
    int16 closestIndex = -1;
    for (int i = 0; i < pathSize; i++)
    {
      if (lastMatchPointIndex >= 0 && std::abs(lastMatchPointIndex - i) > MAX_JUMP)
      {
        continue;
      }

      float64 dis = std::hypot(path.poses[i].pose.position.x - targetPoint.pose.position.x,
                               path.poses[i].pose.position.y - targetPoint.pose.position.y);
      if (dis < minDis)
      {
        minDis = dis;
        closestIndex = i;
      }
    }
    return closestIndex;
  }
  // 计算投影点参数
  void Planning::Curve::calculateProjectedPointParameters(base_msgs::msg::Referline &referenceline)
  {
    // 判断参考线是否为空
    const int16 pathSize = referenceline.refer_line.size();
    if (pathSize < 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("math"), "refer_line too short");
      return;
    }
    // 计算rs
    float64 rs = 0.0;
    for (int i = 0; i < pathSize; i++)
    {
      if (i == 0)
      {
        rs = 0.0;
      }
      else
      {
        rs += std::hypot(
            referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1].pose.pose.position.y,
            referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1].pose.pose.position.x);
      }
      referenceline.refer_line[i].rs = rs;
    }
    // 计算航向角
    for (int i = 0; i < pathSize; i++)
    {
      if (i < pathSize - 1)
      {
        referenceline.refer_line[i].rtheta = std::atan2(
            referenceline.refer_line[i + 1].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
            referenceline.refer_line[i + 1].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
      }
      else
      {
        referenceline.refer_line[i].rtheta = std::atan2(
            referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1].pose.pose.position.y,
            referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1].pose.pose.position.x);
      }
    }

    // 计算曲率
    for (int i = 0; i < pathSize; i++)
    {
      if (i < pathSize - 1)
      {
        const float64 dis = std::hypot(
            referenceline.refer_line[i + 1].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
            referenceline.refer_line[i + 1].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
        if (dis <= EPSILON)
        {
          referenceline.refer_line[i].rkappa = 0.0;
        }
        else
        {
          referenceline.refer_line[i].rkappa =
              (referenceline.refer_line[i + 1].rtheta - referenceline.refer_line[i].rtheta) / dis;
        }
      }
      else
      {
        const float64 dis = std::hypot(
            referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1].pose.pose.position.y,
            referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1].pose.pose.position.x);
        if (dis <= EPSILON)
        {
          referenceline.refer_line[i].rkappa = 0.0;
        }
        else
        {
          referenceline.refer_line[i].rkappa =
              (referenceline.refer_line[i].rtheta - referenceline.refer_line[i - 1].rtheta) / dis;
        }
      }
    }

    // 计算曲率变化率
    for (int i = 0; i < pathSize; i++)
    {
      if (i < pathSize - 1)
      {
        const float64 dis = std::hypot(
            referenceline.refer_line[i + 1].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
            referenceline.refer_line[i + 1].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
        if (dis <= EPSILON)
        {
          referenceline.refer_line[i].rdkappa = 0.0;
        }
        else
        {
          referenceline.refer_line[i].rdkappa =
              (referenceline.refer_line[i + 1].rkappa - referenceline.refer_line[i].rkappa) / dis;
        }
      }
      else
      {
        const float64 dis = std::hypot(
            referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1].pose.pose.position.y,
            referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1].pose.pose.position.x);
        if (dis <= EPSILON)
        {
          referenceline.refer_line[i].rdkappa = 0.0;
        }
        else
        {
          referenceline.refer_line[i].rdkappa =
              (referenceline.refer_line[i].rkappa - referenceline.refer_line[i - 1].rkappa) / dis;
        }
      }
    }
  }
} // namespace Planning
