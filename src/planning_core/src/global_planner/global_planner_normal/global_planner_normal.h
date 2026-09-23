#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"

// Normal 全局规划器：对地图每个采样点取 midline 与 right_boundary 的中点，
// 得到 y=0 的自车道中心线 —— 车道中心 = 车应行驶的路线，故为全局路径。
// 策略：无避障无变道，老实沿自车道中心开到底（变道逻辑留给 decision_center）。

namespace Planning
{
  class GlobalPlannerNormal : public GlobalPlannerBase
  {
  public:
    GlobalPlannerNormal();
    GlobalPlannerNormal(const GlobalPlannerNormal &) = delete;
    ~GlobalPlannerNormal() = default;

    // 实现基类接口：从 PNC 地图搜索生成全局路径
    nav_msgs::msg::Path searchGlobalPath(const base_msgs::msg::PNCMap &pncMap) override;

  private:
    // 配置全局路径 Marker 的显示样式
    void initGlobalPathMarker();

    // 将 globalPath 的每个点搬运进 globalPathMarker.points
    void path2Marker();
  };
}  // namespace Planning

#endif  // GLOBAL_PLANNER_NORMAL_H_
