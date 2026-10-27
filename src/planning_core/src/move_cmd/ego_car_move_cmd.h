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
  // 自车运动状态
  struct CarParam
  {
    float64 posX = 0.0;   // unit: m
    float64 posY = 0.0;   // unit: m
    float64 theta = 0.0;
    float64 speed = 0.0;  // unit: m/s
  };

  // 自车运动控制节点：订阅参考线，按周期积分推进自车并广播 TF
  class EgoCarMoveCmd : public rclcpp::Node
  {
  public:
    EgoCarMoveCmd();
    EgoCarMoveCmd(const EgoCarMoveCmd&) = delete;
    EgoCarMoveCmd &operator=(const EgoCarMoveCmd &) = delete;
    ~EgoCarMoveCmd() = default;

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;              // tf broadcaster
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr referenceLineSubscription;  // reference line subscription
    std::unique_ptr<ConfigReader> configReaderCarMoveCmd;                       // config reader
    std::shared_ptr<VehicleInfoBase> egoCar;                                    // ego car
    CarParam carParam;                                                          // ego car motion state
    float64 cycleTime = 0.1;  // unit: s，积分步长

    // 按速度积分推进自车并广播 TF
    void carBroadcastTf(const nav_msgs::msg::Path::SharedPtr msg);
  };
} // namespace Planning
#endif // EGO_CAR_MOVE_CMD_H_
