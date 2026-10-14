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
  class ReferenceLineCreator
  {
  public:
    ReferenceLineCreator(uint32 frontSize, uint32 backSize,const std::string& frameId);
    ReferenceLineCreator(const ReferenceLineCreator &) = delete;
    ~ReferenceLineCreator() = default;
    base_msgs::msg::Referline createReferenceLine(const nav_msgs::msg::Path &globalPath,
                                                  const geometry_msgs::msg::PoseStamped &vehiclePose);
    nav_msgs::msg::Path referenceLineToRviz();
    base_msgs::msg::Referline getReferenceLine() const { return referenceLine; }
    nav_msgs::msg::Path getReferenceLineRviz() const { return referlineRviz; }
    inline int16 getMatchPointIndex() const { return matchPointIndex; }
    inline int16 getFrontIndex() const { return frontIndex; }
    inline int16 getBackIndex() const { return backIndex; }
  private:
    base_msgs::msg::Referline referenceLine;
    std::string frameId;
    nav_msgs::msg::Path referlineRviz;
    int16 lastMatchPointIndex = -1;
    int16 matchPointIndex = -1;
    int16 frontIndex = -1;
    int16 backIndex = -1;
    uint32 frontSize;
    uint32 backSize;
  };
} // namespace Planning
#endif // REFERENCE_LINE_CREATOR_H_
