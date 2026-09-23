#include "global_planner_normal.h"

namespace Planning
{
    GlobalPlannerNormal::GlobalPlannerNormal()
    {
        RCLCPP_INFO(rclcpp::get_logger("global_path"),"GlobalPlanner is running");

        globalPlannerType = static_cast<uint8>(GlobalPlannerType::NORMAL);
    }

    nav_msgs::msg::Path GlobalPlannerNormal::searchGlobalPath(const base_msgs::msg::PNCMap &pncMap)
    {
        globalPath.header.frame_id = pncMap.header.frame_id;
        globalPath.header.stamp = rclcpp::Clock().now();
        globalPath.poses.clear();

        geometry_msgs::msg::PoseStamped p_tmp;
        p_tmp.header = globalPath.header;
        p_tmp.pose.orientation.x = 0.0;
        p_tmp.pose.orientation.y = 0.0;
        p_tmp.pose.orientation.z = 0.0;
        p_tmp.pose.orientation.w = 0.0;

        const uint32 midLineSize = static_cast<uint32>(pncMap.midline.points.size());
        for (uint32 i  =0U; i < midLineSize; i++)
        {
            p_tmp.pose.position.x = (pncMap.midline.points[i].x + base_msgs::msg::PNCMap().right_boundary.points[i].x)/ 2.0;
            p_tmp.pose.position.y = (pncMap.midline.points[i].y + base_msgs::msg::PNCMap().right_boundary.points[i].y)/ 2.0;
            globalPath.poses.emplace_back(p_tmp);   
        }

        path2Marker();

        RCLCPP_INFO(rclcpp::get_logger("global_path"),"points: %ld",globalPath.poses.size());
        return globalPath;
    }

    void GlobalPlannerNormal::initGlobalPathMarker()
    {
        globalPathMarker.header.frame_id = "world";
        globalPathMarker.header.stamp = rclcpp::Clock().now();

        globalPathMarker.ns = "gloal_path";
        globalPathMarker.id = 0;
        globalPathMarker.action = visualization_msgs::msg::Marker::ADD;
        globalPathMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        globalPathMarker.scale.x = 0.1;
        globalPathMarker.color.r = 1.0;
        globalPathMarker.color.g = 0.0;
        globalPathMarker.color.b = 0.0;
        globalPathMarker.color.a = 1.0;
        globalPathMarker.lifetime = rclcpp::Duration::max();
        globalPathMarker.frame_locked = true;
    }

    void GlobalPlannerNormal::path2Marker()
    {
        initGlobalPathMarker();

        globalPathMarker.points.clear();
        const uint32 pathSize = static_cast<uint32>(globalPath.poses.size());
        for(uint32 i = 0U; i < pathSize; i++)
        {
            geometry_msgs::msg::Point pt;
            pt.x = globalPath.poses[i].pose.position.x;
            pt.y = globalPath.poses[i].pose.position.y;
            pt.z = globalPath.poses[i].pose.position.z;
            globalPathMarker.points.emplace_back(pt);
        }
    }

} // namespace Planning