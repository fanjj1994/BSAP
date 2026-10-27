#include "decision_center.h"

namespace Planning
{
  DecisionCenter::DecisionCenter()
  {
    decisionConfigReader = std::make_unique<ConfigReader>();
    decisionConfigReader->readDecisionConfig();
    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "DecisionCenter is running");
  }
  void DecisionCenter::pathDecisionInitialize()
  {
    pathDecisionPoints.clear();
  }

  void DecisionCenter::makePathDecision(const std::shared_ptr<VehicleInfoBase>& egoCarInfo,
                                        const std::vector<std::shared_ptr<VehicleInfoBase>>& tpInfoList)
  {
    if (tpInfoList.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("decision_center"), "No traffic participant information available");
      return;
    }
    pathDecisionInitialize();

    // decision boundaries
    const float64 leftBoundaryDistance = static_cast<float64>(decisionConfigReader->getPNCMap().lane_width_ * 1.5f);
    const float64 rightBoundaryDistance = -(static_cast<float64>(decisionConfigReader->getPNCMap().lane_width_ * 0.5f));
    // longitudinal filter bounds
    const float64 referenceLineEndDistance = static_cast<float64>(decisionConfigReader->getReferenceLine().front_size_ *
                                                                  decisionConfigReader->getPNCMap().segment_len_);
    const float64 decisionMakingLeastDistance = static_cast<float64>(std::max(egoCarInfo->getDsDt() * 40U, 30.0));

    SLPoint p{};
    for (const auto& tpInfo : tpInfoList)
    {
      const float64 distanceToEgoCar = tpInfo->getS() - egoCarInfo->getS();
      // longitudinal range filter
      if (distanceToEgoCar > referenceLineEndDistance || distanceToEgoCar < -decisionMakingLeastDistance)
      {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "TP %d filtered: range(s=%.2f)", tpInfo->getVehicleID(),
                    tpInfo->getS());
        continue;
      }

      // corridor filter
      if (tpInfo->getL() > leftBoundaryDistance || tpInfo->getL() < rightBoundaryDistance)
      {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "TP %d filtered: corridor(l=%.2f)", tpInfo->getVehicleID(),
                    tpInfo->getL());
        continue;
      }

      // quasi-static filter
      if (std::fabs(tpInfo->getDlDt()) >= 0.03 || tpInfo->getDsDt() >= egoCarInfo->getDsDt() / 2.0)
      {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "TP %d filtered: moving", tpInfo->getVehicleID());
        continue;
      }

      // encounter point projection
      const float64 relativeLongSpeed = egoCarInfo->getDsDt() - tpInfo->getDsDt();
      const float64 approachTime = (relativeLongSpeed != 0.0) ? distanceToEgoCar / relativeLongSpeed : 0.0;
      p.s = tpInfo->getS() + tpInfo->getDsDt() * approachTime;

      // gap check against ego width plus two safety margins
      const float64 tpLeftEdge = tpInfo->getL() + tpInfo->getVehicleWidth() / 2.0;
      const float64 tpRightEdge = tpInfo->getL() - tpInfo->getVehicleWidth() / 2.0;
      const float64 leftGap = leftBoundaryDistance - tpLeftEdge;
      const float64 rightGap = tpRightEdge - rightBoundaryDistance;
      const float64 gapNeeded =
          egoCarInfo->getVehicleWidth() + 2.0 * decisionConfigReader->getDecision().lat_safe_margin_;

      if (leftGap > gapNeeded)
      {
        // midpoint of the left gap
        p.l = (leftBoundaryDistance + tpLeftEdge) / 2.0;
        p.type = SLPointType::LEFT_OVERTAKE;
        pathDecisionPoints.emplace_back(p);
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "TP %d LEFT_OVERTAKE at s = %.2f, l = %.2f",
                    tpInfo->getVehicleID(), p.s, p.l);
      }
      else if (rightGap > gapNeeded)
      {
        // midpoint of the right gap
        p.l = (rightBoundaryDistance + tpRightEdge) / 2.0;
        p.type = SLPointType::RIGHT_OVERTAKE;
        pathDecisionPoints.emplace_back(p);
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "TP %d RIGHT_OVERTAKE at s = %.2f, l = %.2f",
                    tpInfo->getVehicleID(), p.s, p.l);
      }
      else
      {
        // stop ahead of the encounter point
        p.s -= decisionConfigReader->getDecision().long_safe_margin_;
        p.l = 0.0;
        p.type = SLPointType::STOP;
        pathDecisionPoints.emplace_back(p);
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "TP %d STOP at s = %.2f, l = %.2f", tpInfo->getVehicleID(),
                    p.s, p.l);
        break;
      }
    }
    if (pathDecisionPoints.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("decision_center"), "No decision point generated in this cycle");
      return;
    }

    // prepend START point ahead of the first decision point
    SLPoint startPoint;
    startPoint.s = pathDecisionPoints.front().s - decisionMakingLeastDistance;
    startPoint.l = 0.0;
    startPoint.type = SLPointType::START;
    pathDecisionPoints.emplace(pathDecisionPoints.begin(), startPoint);

    // append END point behind the last decision point
    if (pathDecisionPoints.back().type != SLPointType::STOP)
    {
      SLPoint endPoint;
      endPoint.s = pathDecisionPoints.back().s + decisionMakingLeastDistance;
      endPoint.l = 0.0;
      endPoint.type = SLPointType::END;
      pathDecisionPoints.emplace_back(endPoint);
    }
  }

} // namespace Planning