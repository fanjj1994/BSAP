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
    class Curve
    {
    public:
        Curve() = default;
        ~Curve() = default;
    private:
        static constexpr uint8 MAX_JUMP = 100U;
        static constexpr float64 EPSILON = 1.0e-6;
    public:
        static int16 findMatchPointIndex(const nav_msgs::msg::Path &path,const int16 lastMatchPointIndex,const geometry_msgs::msg::PoseStamped &targetPoint);
        static void calculateProjectedPointParameters(base_msgs::msg::Referline &referenceline);
    };
}  // namespace Planning
#endif  // CURVE_H_
