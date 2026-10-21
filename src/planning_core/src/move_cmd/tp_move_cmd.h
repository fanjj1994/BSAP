#ifndef TP_MOVE_CMD_H_
#define TP_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "tp_base.h"
#include "tf2_ros/transform_broadcaster.h"
#include "common_type.h"

namespace Planning
{
  struct TpParam
  {
    float64 posX = 0.0;
    float64 posY = 0.0;
    float64 theta = 0.0;
    float64 speed = 0.0;
    std::shared_ptr<TP> tpCar;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  };
  class TPMoveCmd : public rclcpp::Node
  {
  public:
    TPMoveCmd();
    TPMoveCmd(const TPMoveCmd &) = delete;
    TPMoveCmd &operator=(const TPMoveCmd &) = delete;
    ~TPMoveCmd() = default;

  private:
    std::unique_ptr<ConfigReader> configReaderTp;
    std::vector<TpParam> tpCarParams;
    rclcpp::TimerBase::SharedPtr timerTpMoveCmd;
    void tpBroadcastTf();
  };
} // namespace Planning
#endif // TP_MOVE_CMD_H_
