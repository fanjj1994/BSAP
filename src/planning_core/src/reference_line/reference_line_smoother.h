#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "common_type.h"
#include <Eigen/Dense>
#include <cmath>
#include <OsqpEigen/OsqpEigen.h>

namespace Planning
{
  class ReferenceLineSmoother
  {
  public:
    ReferenceLineSmoother();
    ReferenceLineSmoother(const ReferenceLineSmoother &) = delete;
    ReferenceLineSmoother &operator=(const ReferenceLineSmoother &) = delete; // 赋值也删（原版有）
    ~ReferenceLineSmoother() = default;

    void SmoothReferenceLine(base_msgs::msg::Referline &referline);

  private:
    const float64 w1 = 100.0; // 惩罚急弯
    const float64 w2 = 10.0;  // 惩罚锯齿
    const float64 w3 = 1.0;   // 惩罚跑偏
  };
} // namespace Planning
#endif // REFERENCE_LINE_SMOOTHER_H_
