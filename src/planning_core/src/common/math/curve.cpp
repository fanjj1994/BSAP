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

  // 计算参考线各点的 rs / rtheta / rkappa / rdkappa
  void Planning::Curve::calculateProjectedPointParameters(base_msgs::msg::Referline &referenceline)
  {
    const int16 pathSize = referenceline.refer_line.size();
    if (pathSize < 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("math"), "refer_line too short");
      return;
    }
    // 计算 rs
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

  // 角度归一化到 [-pi, pi]
  float64 Curve::NormalizeAngle(const float64 &angle)
  {
    float64 a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
    {
      a += 2.0 * M_PI;
    }
    return a - M_PI;
  }

  // 查找匹配点下标(参考线)
  int16 Curve::findMatchPointIndex(const base_msgs::msg::Referline &referenceline,
                                   const geometry_msgs::msg::PoseStamped &target)
  {
    const int16 referlineSize = referenceline.refer_line.size();
    if (referlineSize <= 1)
    {
      return referlineSize - 1;
    }
    float64 minDis = std::numeric_limits<float64>::max();
    int16 closestIndex = 0;
    for (int16 i = 0; i < referlineSize; i++)
    {
      const float64 dis = std::hypot(referenceline.refer_line[i].pose.pose.position.x - target.pose.position.x,
                                     referenceline.refer_line[i].pose.pose.position.y - target.pose.position.y);
      if (dis < minDis)
      {
        minDis = dis;
        closestIndex = i;
      }
    }
    return closestIndex;
  }

  // 求投影点：把匹配点的六个几何量装进 projectedPoint
  void Curve::figureOutProjectedPoint(const base_msgs::msg::Referline &referenceline,
                                      const geometry_msgs::msg::PoseStamped &target, ProjectedPointInfo &projectedPoint)
  {
    const int16 matchIndex = findMatchPointIndex(referenceline, target);
    if (matchIndex < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("math"), "Failed to find projected point!");
      return;
    }
    projectedPoint.rs = referenceline.refer_line[matchIndex].rs;
    projectedPoint.rx = referenceline.refer_line[matchIndex].pose.pose.position.x;
    projectedPoint.ry = referenceline.refer_line[matchIndex].pose.pose.position.y;
    projectedPoint.rtheta = referenceline.refer_line[matchIndex].rtheta;
    projectedPoint.rkappa = referenceline.refer_line[matchIndex].rkappa;
    projectedPoint.rdkappa = referenceline.refer_line[matchIndex].rdkappa;
  }

  // 笛卡尔 -> Frenet
  void Curve::CartesianToFrenet(const CartesianState &cartesian, const ProjectedPointInfo &projectedPoint,
                                FrenetState &frenet)
  {
    frenet.s = projectedPoint.rs;
    const float64 dx = cartesian.x - projectedPoint.rx;
    const float64 dy = cartesian.y - projectedPoint.ry;
    const float64 cosThetaR = std::cos(projectedPoint.rtheta);
    const float64 sinThetaR = std::sin(projectedPoint.rtheta);
    const float64 crossRX = cosThetaR * dy - sinThetaR * dx;
    frenet.l = std::copysign(std::hypot(dx, dy), crossRX);

    const float64 deltaTheta = NormalizeAngle(cartesian.theta - projectedPoint.rtheta);
    const float64 tanDeltaTheta = std::tan(deltaTheta);
    const float64 cosDeltaTheta = std::cos(deltaTheta);
    const float64 sinDeltaTheta = std::sin(deltaTheta);
    const float64 oneMinusKappaL = 1.0 - frenet.l * projectedPoint.rkappa;

    frenet.dl_ds = oneMinusKappaL * tanDeltaTheta;
    const float64 kappaLPrime = projectedPoint.rdkappa * frenet.l + projectedPoint.rkappa * frenet.dl_ds;
    const float64 deltaThetaPrime = oneMinusKappaL * cartesian.curvature / cosDeltaTheta - projectedPoint.rkappa;
    frenet.ddl_ds = -kappaLPrime * tanDeltaTheta + oneMinusKappaL / (cosDeltaTheta * cosDeltaTheta) * deltaThetaPrime;
    frenet.ds_dt = cartesian.speed * cosDeltaTheta / oneMinusKappaL;
    frenet.dds_dt = (cartesian.acceleration * cosDeltaTheta -
                     frenet.ds_dt * frenet.ds_dt * (frenet.dl_ds * deltaThetaPrime - kappaLPrime)) /
                    oneMinusKappaL;
    frenet.dl_dt = cartesian.speed * sinDeltaTheta;
    frenet.ddl_dt = cartesian.acceleration * sinDeltaTheta;
  }
} // namespace Planning
