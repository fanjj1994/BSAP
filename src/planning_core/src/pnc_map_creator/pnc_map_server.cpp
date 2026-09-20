#include "pnc_map_server.h"

namespace Planning
{
  PNCMapServer::PNCMapServer() : Node("pnc_map_server_node")
  {
    RCLCPP_INFO(this->get_logger(),"pncMapServer node created");
    // 地图发布器
    mapPub = this->create_publisher<base_msgs::msg::PNCMap>("pnc_map",10U);
    mapRvizPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("pnc_map_markerarray",10U);
    // 地图服务器
    mapServer = this->create_service<base_msgs::srv::PNCMapService>("pnc_map_server",std::bind(&PNCMapServer::responsePNCMapCallBack,this,std::placeholders::_1,std::placeholders::_2)); 
  }

  // 响应并发布地图回调
  void PNCMapServer::responsePNCMapCallBack(const std::shared_ptr<base_msgs::srv::PNCMapService::Request> request,
                                            const std::shared_ptr<base_msgs::srv::PNCMapService::Response> response)
  {
    // 接收请求（多态）
    switch (request->map_type)
    {
        case static_cast<int>(PNCMapType::STRAIGHT):
            mapCreator = std::make_shared<PNCMapCreatorStraight>();
            break;
        default:
            RCLCPP_WARN(this->get_logger(),"Invalid map type");
            return;
    }
    //创建地图并且响应地图
    const auto pncMap = mapCreator->createPNCMap();
    response->pnc_map = pncMap;
    //发布地图
    mapPub->publish(pncMap);
    //发布用于RVIZ显示的地图
    const auto pncMapMarkerArray = mapCreator->getPNCMapMarkerArray();
    mapRvizPub->publish(pncMapMarkerArray);
  }
} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}