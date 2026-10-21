#ifndef DECISION_CENTER_H_
#define DECISION_CENTER_H_

#include "rclcpp/rclcpp.hpp"
#include "common_type.h"
#include "config_reader.h"
#include "vehicle_info_base.h"
#include <vector>
#include <algorithm>

namespace Planning
{
    enum class SLPointType : uint8
    {
        UNKNOWN = 0U,
        LEFT_OVERTAKE = 1U,
        RIGHT_OVERTAKE = 2U,
        STOP = 3U,
        START = 4U,
        END = 5U
    };

    struct SLPoint
    {
        float64 s = 0.0;
        float64 l = 0.0;
        SLPointType type = SLPointType::UNKNOWN;
    };

    class DecisionCenter
    {
    public:
        DecisionCenter();
        DecisionCenter(const DecisionCenter &) = delete;
        ~DecisionCenter() = default;

        void makePathDecision(const std::shared_ptr<VehicleInfoBase>&egoCarInfo, const std::vector<std::shared_ptr<VehicleInfoBase>>&tpInfoList);
        const std::vector<SLPoint>& getPathDecisionPoints() const  { return pathDecisionPoints; }
    private:
        std::unique_ptr<ConfigReader> decisionConfigReader;
        std::vector<SLPoint> pathDecisionPoints;
        void pathDecisionInitialize();
    };
}  // namespace Planning
#endif  // DECISION_CENTER_H_
