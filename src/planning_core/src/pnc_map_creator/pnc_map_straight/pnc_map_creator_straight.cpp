#include "pnc_map_creator_straight.h"

namespace Planning
{
  PncMapCreatorStraight::PncMapCreatorStraight()
  {
    RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "PNCMapCreatorStraight is running");

    // 地图参数
    lengthStep = 1.0;
    pncMap.road_length = 200.0;
    pncMap.road_half_width = 3.5;

    // 采样起点
    pCenter.x = 0.0;
    pCenter.y = pncMap.road_half_width / 2.0;
    pCenter.z = 0.0;

    initPNCMap();
  }

  // 创建直道地图
  base_msgs::msg::PNCMap PncMapCreatorStraight::createPNCMap()
  {
    drawStraightX(pncMap.road_length,1.0);
    if(pncMap.midline.points.size()%2 == 1)
    {
        pncMap.midline.points.pop_back();
    }

    pncMapMarkerArray.markers.emplace_back(pncMap.midline);
    pncMapMarkerArray.markers.emplace_back(pncMap.left_boundary);
    pncMapMarkerArray.markers.emplace_back(pncMap.right_boundary);

    RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "PNCMapCreatorStraight has been created,midline has %ld points",pncMap.midline.points.size());
    return pncMap;
  }

  // 初始化地图
  void PncMapCreatorStraight::initPNCMap()
  {
    pncMap.header.frame_id = "world";
    pncMap.header.stamp = rclcpp::Clock().now();

    // 中心线格式
    pncMap.midline.header = pncMap.header;
    pncMap.midline.ns = "pnc_map";
    pncMap.midline.id = 0;
    pncMap.midline.action = visualization_msgs::msg::Marker::ADD;
    pncMap.midline.type = visualization_msgs::msg::Marker::LINE_LIST;
    pncMap.midline.scale.x = 0.05;
    pncMap.midline.color.r = 1.0;
    pncMap.midline.color.g = 1.0;
    pncMap.midline.color.b = 0.0;
    pncMap.midline.color.a = 1.0;
    pncMap.midline.lifetime = rclcpp::Duration::max();
    pncMap.midline.frame_locked = true;

    // 左边界格式
    pncMap.left_boundary.header = pncMap.header;
    pncMap.left_boundary.ns = "pnc_map";
    pncMap.left_boundary.id = 1;
    pncMap.left_boundary.action = visualization_msgs::msg::Marker::ADD;
    pncMap.left_boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
    pncMap.left_boundary.scale.x = 0.05;
    pncMap.left_boundary.color.r = 1.0;
    pncMap.left_boundary.color.g = 1.0;
    pncMap.left_boundary.color.b = 1.0;
    pncMap.left_boundary.color.a = 1.0;
    pncMap.left_boundary.lifetime = rclcpp::Duration::max();
    pncMap.left_boundary.frame_locked = true;

    // 右边界格式
    pncMap.right_boundary.header = pncMap.header;
    pncMap.right_boundary.ns = "pnc_map";
    pncMap.right_boundary.id = 2;
    pncMap.right_boundary.action = visualization_msgs::msg::Marker::ADD;
    pncMap.right_boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
    pncMap.right_boundary.scale.x = 0.05;
    pncMap.right_boundary.color.r = 1.0;
    pncMap.right_boundary.color.g = 1.0;
    pncMap.right_boundary.color.b = 1.0;
    pncMap.right_boundary.color.a = 1.0;
    pncMap.right_boundary.lifetime = rclcpp::Duration::max();
    pncMap.right_boundary.frame_locked = true;
  }

  // 沿x轴绘制直线
  void PncMapCreatorStraight::drawStraightX(const float64 &length, const float64 &plusFlag, const float64 &ratio)
  {
    float64 lenRuler = 0.0;
    while ( lenRuler < length)
    {
        pLeft.x = pCenter.x;
        pLeft.y = pCenter.y + pncMap.road_half_width;
        pRight.x = pCenter.x;
        pRight.y = pCenter.y - pncMap.road_half_width;
        pncMap.midline.points.emplace_back(pCenter);
        pncMap.left_boundary.points.emplace_back(pLeft);
        pncMap.right_boundary.points.emplace_back(pRight);

        lenRuler += lengthStep * ratio;
        pCenter.x += lengthStep * plusFlag;
    }

  }
} // namespace Planning