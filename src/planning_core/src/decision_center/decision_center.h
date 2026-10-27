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
    // 决策点类型
    enum class SLPointType : uint8
    {
        UNKNOWN = 0U,
        LEFT_OVERTAKE = 1U,
        RIGHT_OVERTAKE = 2U,
        STOP = 3U,
        START = 4U,
        END = 5U
    };

    // 决策点
    struct SLPoint
    {
        float64 s = 0.0;   // unit: m
        float64 l = 0.0;   // unit: m
        SLPointType type = SLPointType::UNKNOWN;
    };

    // 决策中心：把 TP 列表转换为决策点序列
    class DecisionCenter
    {
    public:
        DecisionCenter();
        DecisionCenter(const DecisionCenter &) = delete;
        ~DecisionCenter() = default;

        // make path decision for one cycle
        void makePathDecision(const std::shared_ptr<VehicleInfoBase>&egoCarInfo, const std::vector<std::shared_ptr<VehicleInfoBase>>&tpInfoList);

        // get decision points of this cycle
        const std::vector<SLPoint>& getPathDecisionPoints() const  { return pathDecisionPoints; }

    private:
        std::unique_ptr<ConfigReader> decisionConfigReader;  // config reader
        std::vector<SLPoint> pathDecisionPoints;             // decision points of this cycle

        // clear decision points at the beginning of each cycle
        void pathDecisionInitialize();
    };
}  // namespace Planning
#endif  // DECISION_CENTER_H_
