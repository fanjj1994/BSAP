#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "common_type.h"
#include "config_reader.h"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "nav_msgs/msg/path.hpp"

namespace Planning
{
  using namespace std::chrono_literals;
  class PlanningProcess : public rclcpp::Node
  {
  public:
    static constexpr uint8 PROCESS_FAILURE{ 255U };
    PlanningProcess();
    ~PlanningProcess() = default;
    bool process();
    bool initPlanning();
    template <typename T> bool connectServer(const T& client, const std::string& serverName);
    bool requestPNCMap();
    bool requestGlobalPath();

  private:
    std::unique_ptr<ConfigReader> configReaderProcess;
    base_msgs::msg::PNCMap pncMap;
    nav_msgs::msg::Path globalPath;
    rclcpp::Client<base_msgs::srv::GlobalPathService>::SharedPtr globalPathClient;
    rclcpp::Client<base_msgs::srv::PNCMapService>::SharedPtr pncMapClient;
  };
} // namespace Planning
#endif // PLANNING_PROCESS_H_
