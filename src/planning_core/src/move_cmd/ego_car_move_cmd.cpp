#include "ego_car_move_cmd.h"

namespace Planning
{
  EgoCarMoveCmd::EgoCarMoveCmd() : Node("car_move_cmd_node")
  {
    RCLCPP_INFO(this->get_logger(), "EgoCarMoveCmd node has been created.");
    // 读配置：地图、车辆与周期
    configReaderCarMoveCmd = std::make_unique<ConfigReader>();
    configReaderCarMoveCmd->readPNCMapConfig();
    configReaderCarMoveCmd->readVehiclesConfig();
    configReaderCarMoveCmd->readProcessConfig();
    cycleTime = configReaderCarMoveCmd->getProcess().cycle_time_;

    // 初始化自车状态
    egoCar = std::make_shared<EgoCar>(configReaderCarMoveCmd->getEgoCar(), configReaderCarMoveCmd->getPNCMap().frame_);
    carParam.posX = egoCar->getVehiclePose().pose.position.x;
    carParam.posY = egoCar->getVehiclePose().pose.position.y;
    carParam.theta = egoCar->getVehicleTheta();
    carParam.speed = egoCar->getVehicleVelocity();

    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // 订阅参考线：每收到一条消息推进一步
    referenceLineSubscription = this->create_subscription<nav_msgs::msg::Path>(
        "reference_line", 10, std::bind(&EgoCarMoveCmd::carBroadcastTf, this, std::placeholders::_1));
  }

  // 按周期积分推进自车并广播 TF
  void EgoCarMoveCmd::carBroadcastTf(const nav_msgs::msg::Path::SharedPtr msg)
  {
    const uint8 msgSize = static_cast<uint8>(msg->poses.size());
    if (msgSize < 3U)
    {
      RCLCPP_ERROR(this->get_logger(), "msg too short!");
      return;
    }

    // 位置积分：速度 x 周期
    carParam.posX += carParam.speed * cycleTime * std::cos(carParam.theta);
    carParam.posY += carParam.speed * cycleTime * std::sin(carParam.theta);

    // 组装并广播 TF
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = configReaderCarMoveCmd->getPNCMap().frame_;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.child_frame_id = egoCar->getVehicleChildFrame();
    transformStamped.transform.translation.x = carParam.posX;
    transformStamped.transform.translation.y = carParam.posY;
    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, carParam.theta);
    transformStamped.transform.rotation.x = qtn.getX();
    transformStamped.transform.rotation.y = qtn.getY();
    transformStamped.transform.rotation.z = qtn.getZ();
    transformStamped.transform.rotation.w = qtn.getW();

    tfBroadcaster->sendTransform(transformStamped);
  }
} // namespace Planning

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::EgoCarMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}