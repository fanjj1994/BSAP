#ifndef EGO_CAR_MOVE_CMD_H_
#define EGO_CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "common_type.h"
#include "config_reader.h"
#include "ego_car.h"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace Planning
{
  struct CarParam
  {
    float64 posX = 0.0;
    float64 posY = 0.0;
    float64 theta = 0.0;
    float64 speed = 0.0;
  };

  class EgoCarMoveCmd : public rclcpp::Node
  {
  public:
    EgoCarMoveCmd();
    EgoCarMoveCmd(const EgoCarMoveCmd&) = delete;
    EgoCarMoveCmd &operator=(const EgoCarMoveCmd &) = delete;
    ~EgoCarMoveCmd() = default;
  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr referenceLineSubscription;
    std::unique_ptr<ConfigReader> configReaderCarMoveCmd;
    std::shared_ptr<VehicleInfoBase> egoCar;
    CarParam carParam;
    void carBroadcastTf(const nav_msgs::msg::Path::SharedPtr msg);
  };
} // namespace Planning
#endif // EGO_CAR_MOVE_CMD_H_
