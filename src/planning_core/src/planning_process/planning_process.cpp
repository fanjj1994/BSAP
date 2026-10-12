#include "planning_process.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process")
  {
    configReaderProcess = std::make_unique<ConfigReader>();
    RCLCPP_INFO(this->get_logger(), "PlanningProcess node has been created.");
    // 调用地图配置
    configReaderProcess->readPNCMapConfig();
    RCLCPP_INFO(this->get_logger(), "PNCMap configuration has been read.frame: %s, road_length: %f",
                configReaderProcess->getPNCMap().frame_.c_str(), configReaderProcess->getPNCMap().road_length_);
    // 调用全局路径配置
    configReaderProcess->readGlobalPathConfig();
    RCLCPP_INFO(this->get_logger(), "GlobalPath configuration has been read. global_path_type: %u",
                configReaderProcess->getGlobalPath().type_);
    // 调用参考线配置
    configReaderProcess->readReferenceLineConfig();
    RCLCPP_INFO(this->get_logger(), "ReferenceLine configuration has been read. front_size : %u, back_size: %u",
                configReaderProcess->getReferenceLine().front_size_,
                configReaderProcess->getReferenceLine().back_size_);
    // 调用规划流程配置
    configReaderProcess->readProcessConfig();
    RCLCPP_INFO(this->get_logger(), "Process configuration has been read. cycle_time: %f",
                configReaderProcess->getProcess().cycle_time_);
    pncMapClient = this->create_client<base_msgs::srv::PNCMapService>("pnc_map_server");
    globalPathClient = this->create_client<base_msgs::srv::GlobalPathService>("global_path_server");
  }
  template <typename T>
  bool PlanningProcess::connectServer(const T& client, const std::string& serverName)
  {
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service.", serverName.c_str());
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "%s server not available, waiting again...", serverName.c_str());
    }
    return true;
  }

  bool PlanningProcess::process()
  {
    rclcpp::Rate rate(1.0);
    rate.sleep();
    if (!initPlanning())
    {
      RCLCPP_ERROR(this->get_logger(), "Planning init failed");
      return false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "PlanningProcess node is running.");
    }
    return true;
  }

  bool PlanningProcess::initPlanning()
  {
    if (!connectServer(pncMapClient, "pnc_map"))
    {
      RCLCPP_ERROR(this->get_logger(), "Can not connect to pncMap");
      return false;
    }
    if (!requestPNCMap())
    {
      RCLCPP_ERROR(this->get_logger(), "Map response and request failed");
      return false;
    }
    if (!connectServer(globalPathClient, "global_path"))
    {
      RCLCPP_ERROR(this->get_logger(), "Can not connect to globalPath");
      return false;
    }
    if (!requestGlobalPath())
    {
      RCLCPP_ERROR(this->get_logger(), "GlobalPath response and request failed");
      return false;
    }
    return true;
  }

  bool PlanningProcess::requestPNCMap()
  {
    RCLCPP_INFO(this->get_logger(), "Sending map request");
    auto request = std::make_shared<base_msgs::srv::PNCMapService::Request>();
    request->map_type = configReaderProcess->getPNCMap().type_;
    auto result = pncMapClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      pncMap = result.get()->pnc_map;
      RCLCPP_INFO(this->get_logger(), "Map response success, pnc_midline_size:%zu", pncMap.midline.points.size());
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Map response failed!");
      return false;
    }
  }

  bool PlanningProcess::requestGlobalPath()
  {
    RCLCPP_INFO(this->get_logger(), "Sending globalPath request");
    auto request = std::make_shared<base_msgs::srv::GlobalPathService::Request>();
    request->global_planner_type = configReaderProcess->getGlobalPath().type_;
    request->pnc_map = pncMap;
    auto result = globalPathClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      globalPath = result.get()->global_path;
      RCLCPP_INFO(this->get_logger(), "GlobalPath response success, global_path_size:%zu.", globalPath.poses.size());
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "GlobalPath response failed!");
      return false;
    }
  }

} // namespace Planning
