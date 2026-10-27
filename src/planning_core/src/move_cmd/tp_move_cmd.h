#ifndef TP_MOVE_CMD_H_
#define TP_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "tp_base.h"
#include "tf2_ros/transform_broadcaster.h"
#include "common_type.h"

namespace Planning
{
  // TP 车辆参数
  struct TpParam
  {
    float64 posX = 0.0;     // unit: m
    float64 posY = 0.0;     // unit: m
    float64 theta = 0.0;
    float64 speed = 0.0;    // unit: m/s
    std::shared_ptr<TP> tpCar;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;  // tf broadcaster
  };

  // TP 运动控制节点：按配置速度积分推进各 TP 并广播 TF
  class TPMoveCmd : public rclcpp::Node
  {
  public:
    TPMoveCmd();
    TPMoveCmd(const TPMoveCmd &) = delete;
    TPMoveCmd &operator=(const TPMoveCmd &) = delete;
    ~TPMoveCmd() = default;

  private:
    std::unique_ptr<ConfigReader> configReaderTp;       // config reader
    std::vector<TpParam> tpCarParams;                   // tp list
    rclcpp::TimerBase::SharedPtr timerTpMoveCmd;        // broadcast timer

    // 按速度积分推进各 TP 并广播 TF
    void tpBroadcastTf();
  };
} // namespace Planning
#endif // TP_MOVE_CMD_H_
