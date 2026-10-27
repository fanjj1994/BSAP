#include "tp_move_cmd.h"

namespace Planning
{
  TPMoveCmd::TPMoveCmd() : Node("tp_move_cmd_node")
  {
    RCLCPP_INFO(this->get_logger(), "TPMoveCmd node has been created.");
    // 读配置：地图与车辆
    configReaderTp = std::make_unique<ConfigReader>();
    configReaderTp->readPNCMapConfig();
    configReaderTp->readVehiclesConfig();

    // 初始化各 TP（跳过 ego）
    for (const auto &vehiclePair : configReaderTp->getVehiclePairs())
    {
      if (vehiclePair.first == 0U)
      {
        continue;
      }
      TpParam tpCarParam;
      tpCarParam.tpCar = std::make_shared<TP>(vehiclePair.first);
      tpCarParam.tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      tpCarParam.posX = tpCarParam.tpCar->getVehiclePose().pose.position.x;
      tpCarParam.posY = tpCarParam.tpCar->getVehiclePose().pose.position.y;
      tpCarParam.theta = tpCarParam.tpCar->getVehicleTheta();
      tpCarParam.speed = tpCarParam.tpCar->getVehicleVelocity();
      tpCarParams.emplace_back(tpCarParam);
    }

    // 周期定时器：推进各 TP 并广播 TF
    timerTpMoveCmd =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TPMoveCmd::tpBroadcastTf, this));
    for (const auto &tpCarParam : tpCarParams)
    {
      RCLCPP_INFO(this->get_logger(), "TP %s at (%.2f, %.2f) is ready.",
                  tpCarParam.tpCar->getVehicleChildFrame().c_str(), tpCarParam.posX, tpCarParam.posY);
    }
  }

  // 按速度积分推进各 TP 并广播 TF
  void TPMoveCmd::tpBroadcastTf()
  {
    for (auto &tpCarParam : tpCarParams)
    {
      // 位置积分：速度 x 周期（当前场景 TP 静止，积分量为零）
      const float64 speed_x = tpCarParam.speed * std::cos(tpCarParam.theta);
      const float64 speed_y = tpCarParam.speed * std::sin(tpCarParam.theta);
      tpCarParam.posX += speed_x;
      tpCarParam.posY += speed_y;
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.frame_id = configReaderTp->getPNCMap().frame_;
      transformStamped.header.stamp = this->get_clock()->now();
      transformStamped.child_frame_id = tpCarParam.tpCar->getVehicleChildFrame();
      transformStamped.transform.translation.x = tpCarParam.posX;
      transformStamped.transform.translation.y = tpCarParam.posY;
      transformStamped.transform.translation.z = 0.0;
      tf2::Quaternion qtn;
      qtn.setRPY(0.0, 0.0, tpCarParam.theta);
      transformStamped.transform.rotation.x = qtn.getX();
      transformStamped.transform.rotation.y = qtn.getY();
      transformStamped.transform.rotation.z = qtn.getZ();
      transformStamped.transform.rotation.w = qtn.getW();
      tpCarParam.tfBroadcaster->sendTransform(transformStamped);
    }
  }
} // namespace Planning

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::TPMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}