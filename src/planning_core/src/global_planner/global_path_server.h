#ifndef GLOBAL_PATH_SERVER_H_
#define GLOBAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "global_planner_normal.h"

namespace Planning
{   
    class GlobalPathServer : public rclcpp::Node
    {
    public:
        GlobalPathServer();
        GlobalPathServer(const GlobalPathServer &) = delete;
        ~GlobalPathServer() = default;

    private:
        // 服务器回调
        void responseGlobalPathCallBack(
            const std::shared_ptr<base_msgs::srv::GlobalPathService::Request> request,
            const std::shared_ptr<base_msgs::srv::GlobalPathService::Response> response);
        
        // 全局路径创建器
        std::shared_ptr<GlobalPlannerBase> globalPlannerCreator;

        // 全局路径发布器
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr globalPathPub;

        // 全局路径发布器（Rviz）
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr globalRvizPub;

        // 全局路径服务器
        rclcpp::Service<base_msgs::srv::GlobalPathService>::SharedPtr globalPathServer;
    };

}  // namespace Planning
#endif  // GLOBAL_PATH_SERVER_H_
