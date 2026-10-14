#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "common_type.h"
#include "config_reader.h"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "nav_msgs/msg/path.hpp"
#include "reference_line_creator.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ego_car.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
    void planningCallback();
    void spawnVehicle(const std::shared_ptr<VehicleInfoBase> &vehicle);
    void getVehicleLocation(const std::shared_ptr<VehicleInfoBase> &vehicle);

  private:
    std::unique_ptr<ConfigReader> configReaderProcess;
    base_msgs::msg::PNCMap pncMap;
    nav_msgs::msg::Path globalPath;
    rclcpp::Client<base_msgs::srv::GlobalPathService>::SharedPtr globalPathClient;
    rclcpp::Client<base_msgs::srv::PNCMapService>::SharedPtr pncMapClient;
    std::shared_ptr<ReferenceLineCreator> referenceLineCreator;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr referenceLineRvizPublisher;
    rclcpp::TimerBase::SharedPtr runtime;
    std::shared_ptr<VehicleInfoBase> egoCar;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
  };
} // namespace Planning
#endif // PLANNING_PROCESS_H_
