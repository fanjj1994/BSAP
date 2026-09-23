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
    enum GlobalPlannerType : uint8
    {
        NORMAL = 0U,
        ASTAR = 1U,
        DEFAULT = 255U
    };

    class GlobalPlannerBase
    {
    public:
        GlobalPlannerBase() = default;
        GlobalPlannerBase(const GlobalPlannerBase &) = delete;
        virtual ~GlobalPlannerBase() = default;
        virtual nav_msgs::msg::Path searchGlobalPath(const base_msgs::msg::PNCMap &) = 0;

        nav_msgs::msg::Path getGlobalPath() const { return globalPath;}
        visualization_msgs::msg::Marker getGlobalPathMarker() const { return globalPathMarker;}
    protected:
        nav_msgs::msg::Path globalPath;
        visualization_msgs::msg::Marker globalPathMarker;
        uint8 globalPlannerType { 0U };
    };
}  // namespace Planning
#endif  // GLOBAL_PLANNER_BASE_H_
