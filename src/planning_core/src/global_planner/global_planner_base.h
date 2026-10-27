#ifndef GLOBAL_PLANNER_BASE_H_
#define GLOBAL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "common_type.h"
#include "visualization_msgs/msg/marker.hpp"

namespace Planning
{
    // 全局规划器类型
    enum GlobalPlannerType : uint8
    {
        NORMAL = 0U,
        ASTAR = 1U,
        DEFAULT = 255U
    };

    // 全局规划器基类
    class GlobalPlannerBase
    {
    public:
        GlobalPlannerBase() = default;
        GlobalPlannerBase(const GlobalPlannerBase &) = delete;
        virtual ~GlobalPlannerBase() = default;

        // 从 PNC 地图搜索生成全局路径
        virtual nav_msgs::msg::Path searchGlobalPath(const base_msgs::msg::PNCMap &) = 0;

        // 获取已生成的全局路径
        nav_msgs::msg::Path getGlobalPath() const { return globalPath;}

        // 获取全局路径的可视化数据（Marker，给 RViz 用）
        visualization_msgs::msg::Marker getGlobalPathMarker() const { return globalPathMarker;}

    protected:
        nav_msgs::msg::Path globalPath;                    // global path
        visualization_msgs::msg::Marker globalPathMarker; // marker for rviz
        uint8 globalPlannerType { 0U };                   // planner type
    };
}  // namespace Planning
#endif  // GLOBAL_PLANNER_BASE_H_
