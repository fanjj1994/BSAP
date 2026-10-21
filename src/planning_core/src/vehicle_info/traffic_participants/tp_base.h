#ifndef TP_BASE_H_
#define TP_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_base.h"
#include "config_reader.h"

namespace Planning
{
  class TP : public VehicleInfoBase
  {
  public:
    TP(const uint8 id);
    TP(const TP &) = delete;
    TP &operator=(const TP &) = delete;
    ~TP() = default;
    void vehicleCartesianToFrenet(const base_msgs::msg::Referline &referline) override;
  };
} // namespace Planning
#endif // TP_BASE_H_
