#include "global_path_server.h"

namespace Planning
{
  GlobalPathServer::GlobalPathServer() : Node("global_path_server_node")
  {
    RCLCPP_INFO(this->get_logger(), "globalPathServer node created");

    // 两个发布器
    globalPathPub = this->create_publisher<nav_msgs::msg::Path>("global_path", 10U);
    globalRvizPub = this->create_publisher<visualization_msgs::msg::Marker>("global_path_rviz", 10U);

    // 服务
    globalPathServer = this->create_service<base_msgs::srv::GlobalPathService>(
        "global_path_server",
        std::bind(&GlobalPathServer::responseGlobalPathCallBack, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  void GlobalPathServer::responseGlobalPathCallBack(
      const std::shared_ptr<base_msgs::srv::GlobalPathService::Request> request,
      const std::shared_ptr<base_msgs::srv::GlobalPathService::Response> response)
  {
    // Step1: 解析请求（多态分发）
    switch (request->global_planner_type)
    {
    case static_cast<int>(GlobalPlannerType::NORMAL):
      globalPlannerCreator = std::make_shared<GlobalPlannerNormal>();
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Invalid global planner type");
      return;
    }

    if (request->pnc_map.midline.points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "pnc_map is empty! Global path cannot be created.");
      return;
    }

    const auto globalPath = globalPlannerCreator->searchGlobalPath(request->pnc_map);
    response->global_path = globalPath;

    // Step4: 双发布
    globalPathPub->publish(globalPath);
    const auto globalPathMarker = globalPlannerCreator->getGlobalPathMarker();
    globalRvizPub->publish(globalPathMarker);
  }
}  // namespace Planning

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::GlobalPathServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
