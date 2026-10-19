#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include "common_type.h"
#include "base_msgs/msg/referline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <cmath>

namespace Planning
{
  struct CartesianState
  {
    float64 x = 0.0;
    float64 y = 0.0;
    float64 theta = 0.0;
    float64 speed = 0.0;
    float64 acceleration = 0.0;
    float64 curvature = 0.0;
  };

  struct FrenetState
  {
    float64 s = 0.0;
    float64 ds_dt = 0.0;
    float64 dds_dt = 0.0;
    float64 l = 0.0;
    float64 dl_ds = 0.0;
    float64 dl_dt = 0.0;
    float64 ddl_ds = 0.0;
    float64 ddl_dt = 0.0;
  };

  struct ProjectedPointInfo
  {
    float64 rs = 0.0;
    float64 rx = 0.0;
    float64 ry = 0.0;
    float64 rtheta = 0.0;   // 曾拼作 rthete——与 ReferlinePoint.msg 字段对齐
    float64 rkappa = 0.0;
    float64 rdkappa = 0.0;
  };

  class Curve
  {
  public:
    Curve() = default;
    ~Curve() = default;

  private:
    static constexpr uint8 MAX_JUMP = 100U;
    static constexpr float64 EPSILON = 1.0e-6;

  public:
    static int16 findMatchPointIndex(const nav_msgs::msg::Path &path, const int16 lastMatchPointIndex,
                                     const geometry_msgs::msg::PoseStamped &targetPoint);
    static void calculateProjectedPointParameters(base_msgs::msg::Referline &referenceline);
    static float64 NormalizeAngle(const float64 &angle);
    static int16 findMatchPointIndex(const base_msgs::msg::Referline &referenceline,
                                     const geometry_msgs::msg::PoseStamped &target);
    static void figureOutProjectedPoint(const base_msgs::msg::Referline &referenceline,
                                        const geometry_msgs::msg::PoseStamped &target,
                                        ProjectedPointInfo &projectedPoint);
    static void CartesianToFrenet(const CartesianState &cartesian, const ProjectedPointInfo &projectedPoint,
                                  FrenetState &frenet);
  };
} // namespace Planning
#endif // CURVE_H_
