#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline_point.hpp"
#include "base_msgs/msg/referline.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include "curve.h"

namespace Planning
{
  // 参考线生成器
  class ReferenceLineCreator
  {
  public:
    ReferenceLineCreator(uint32 frontSize, uint32 backSize,const std::string& frameId);
    ReferenceLineCreator(const ReferenceLineCreator &) = delete;
    ~ReferenceLineCreator() = default;

    // create reference line from global path and vehicle pose
    base_msgs::msg::Referline createReferenceLine(const nav_msgs::msg::Path &globalPath,
                                                  const geometry_msgs::msg::PoseStamped &vehiclePose);
    // reference line convert to rviz display
    nav_msgs::msg::Path referenceLineToRviz();

    inline base_msgs::msg::Referline getReferenceLine() const { return referenceLine; }
    inline nav_msgs::msg::Path getReferenceLineRviz() const { return referlineRviz; }
    inline int16 getMatchPointIndex() const { return matchPointIndex; }
    inline int16 getFrontIndex() const { return frontIndex; }
    inline int16 getBackIndex() const { return backIndex; }
  private:
    base_msgs::msg::Referline referenceLine;   // reference line for planning
    std::string frameId;                       // world frame
    nav_msgs::msg::Path referlineRviz;         // reference line for rviz display
    int16 lastMatchPointIndex = -1;            // last match point index
    int16 matchPointIndex = -1;                // current match point index
    int16 frontIndex = -1;                     // front index
    int16 backIndex = -1;                      // back index
    uint32 frontSize;                          // front points of match point
    uint32 backSize;                           // back points of match point
  };
} // namespace Planning
#endif // REFERENCE_LINE_CREATOR_H_
