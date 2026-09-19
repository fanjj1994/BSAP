#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include "common_type.h"

namespace Planning
{
    enum class PNCMapType : uint8// 地图类型
    {
        STRAIGHT = 0U,
        STURN = 1U,
        DEFAULT = 255U
    };
    class PNCMapCreatorBase // 地图创建器基类
    {
    public:
        PNCMapCreatorBase() = default;
        PNCMapCreatorBase(const PNCMapCreatorBase &) = delete;
        virtual ~PNCMapCreatorBase() = default;
        virtual base_msgs::msg::PNCMap createPNCMap() = 0;

    protected:
        base_msgs::msg::PNCMap pncMap;
        visualization_msgs::msg::MarkerArray pncMapMarkerArray;
        geometry_msgs::msg::Point pCenter;
        geometry_msgs::msg::Point pLeft;
        geometry_msgs::msg::Point pRight;
        float64 lengthStep{ 0.0 };
    };
}  // namespace Planning
#endif  // PNC_MAP_CREATOR_BASE_H_
