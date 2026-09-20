#ifndef PNC_MAP_SERVER_H_
#define PNC_MAP_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "pnc_map_creator_straight.h"

namespace Planning
{
  // PNC 地图服务器节点：持有地图，对外提供"取地图"服务
  class PNCMapServer : public rclcpp::Node
  {
  public:
    PNCMapServer();
    PNCMapServer(const PNCMapServer &) = delete;
    ~PNCMapServer() = default;

  private:
    // 服务回调
    void responsePNCMapCallBack(
        const std::shared_ptr<base_msgs::srv::PNCMapService::Request> request,
        const std::shared_ptr<base_msgs::srv::PNCMapService::Response> response);

    // 地图创建器（基类指针，多态调用具体子类）
    std::shared_ptr<PNCMapCreatorBase> mapCreator;

    // 地图发布器（发地图本体，给规划节点用）
    rclcpp::Publisher<base_msgs::msg::PNCMap>::SharedPtr mapPub;

    // 地图发布器（发 MarkerArray，给 RViz 显示用）
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mapRvizPub;

    // 地图服务（等待客户端请求）
    rclcpp::Service<base_msgs::srv::PNCMapService>::SharedPtr mapServer;
  };
}  // namespace Planning

#endif  // PNC_MAP_SERVER_H_
