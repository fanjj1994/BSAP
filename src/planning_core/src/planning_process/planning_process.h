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
#include "tp_base.h"
#include "decision_center.h"
#include "local_path_creator.h"

namespace Planning
{
  using namespace std::chrono_literals;
  // 规划主节点
  class PlanningProcess : public rclcpp::Node
  {
  public:
    static constexpr uint8 PROCESS_FAILURE{ 255U };
    PlanningProcess();
    ~PlanningProcess() = default;

    // entry point: init and start the cycle timer
    bool process();
    // init: connect servers, request map and global path, spawn vehicles
    bool initPlanning();
    // wait for the server
    template <typename T>
    bool connectServer(const T &client, const std::string &serverName);
    // request PNCMap service
    bool requestPNCMap();
    // request GlobalPath service
    bool requestGlobalPath();
    // cycle callback
    void planningCallback();
    // spawn vehicle with static tf
    void spawnVehicle(const std::shared_ptr<VehicleInfoBase> &vehicle);
    // update vehicle pose by tf
    void getVehicleLocation(const std::shared_ptr<VehicleInfoBase> &vehicle);

  private:
    std::unique_ptr<ConfigReader> configReaderProcess;             // config reader
    base_msgs::msg::PNCMap pncMap;                                 // pnc map from server
    nav_msgs::msg::Path globalPath;                                // global path from server
    rclcpp::Client<base_msgs::srv::GlobalPathService>::SharedPtr globalPathClient;  // global path client
    rclcpp::Client<base_msgs::srv::PNCMapService>::SharedPtr pncMapClient;          // pnc map client
    std::shared_ptr<ReferenceLineCreator> referenceLineCreator;    // reference line creator
    rclcpp::Publisher<base_msgs::msg::LocalPath>::SharedPtr localPathPublisher;     //local path publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr referenceLineRvizPublisher;   // reference line publisher
    rclcpp::TimerBase::SharedPtr runtime;                          // cycle timer
    std::shared_ptr<VehicleInfoBase> egoCar;                       // ego car
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster;  // spawn tf broadcaster
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;                     // tf buffer
    std::shared_ptr<tf2_ros::TransformListener> tfListener;        // tf listener
    std::shared_ptr<DecisionCenter> decisionCenter;                // decision center
    std::shared_ptr<LocalPathCreator> localPathCreator;            // local path
    std::vector<std::shared_ptr<VehicleInfoBase>> tpCars;          // traffic participants
  };
} // namespace Planning
#endif // PLANNING_PROCESS_H_
