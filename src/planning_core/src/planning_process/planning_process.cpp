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
    decisionCenter = std::make_shared<DecisionCenter>();
    tpCars = { std::make_shared<TP>(1U), std::make_shared<TP>(2U), std::make_shared<TP>(3U) };
    // 调用规划流程配置
    configReaderProcess->readProcessConfig();
    RCLCPP_INFO(this->get_logger(), "Process configuration has been read. cycle_time: %f",
                configReaderProcess->getProcess().cycle_time_);

    // 调用车辆信息
    configReaderProcess->readVehiclesConfig();
    egoCar = std::make_shared<EgoCar>(configReaderProcess->getEgoCar(), configReaderProcess->getPNCMap().frame_);
    RCLCPP_INFO(this->get_logger(), "ego car pose: x = %.2f, y = %.2f", egoCar->getVehiclePose().pose.position.x,
                egoCar->getVehiclePose().pose.position.y);
    pncMapClient = this->create_client<base_msgs::srv::PNCMapService>("pnc_map_server");
    globalPathClient = this->create_client<base_msgs::srv::GlobalPathService>("global_path_server");
    referenceLineCreator = std::make_shared<ReferenceLineCreator>(configReaderProcess->getReferenceLine().front_size_,
                                                                  configReaderProcess->getReferenceLine().back_size_,
                                                                  configReaderProcess->getPNCMap().frame_);
    referenceLineRvizPublisher = this->create_publisher<nav_msgs::msg::Path>("reference_line", 10);
    tfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
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
      const float64 cycleTime = configReaderProcess->getProcess().cycle_time_;
      runtime = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<float64>(cycleTime)),
          std::bind(&PlanningProcess::planningCallback, this));
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
    spawnVehicle(egoCar);
    return true;
  }
  void PlanningProcess::planningCallback()
  {
    getVehicleLocation(egoCar);
    const auto referenceLine_ = referenceLineCreator->createReferenceLine(globalPath, egoCar->getVehiclePose());
    egoCar->vehicleCartesianToFrenet(referenceLine_);
    for (auto& tpCar : tpCars)
    {
      tpCar->vehicleCartesianToFrenet(referenceLine_);
    }
    decisionCenter->makePathDecision(egoCar, tpCars);

    if (referenceLine_.refer_line.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "reference line is empty!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "ego frenet: s = %.2f, l = %.2f, ds_dt = %.2f", egoCar->getS(), egoCar->getL(),
                egoCar->getDsDt());
    RCLCPP_INFO(this->get_logger(), "decision points: %zu", decisionCenter->getPathDecisionPoints().size());
    referenceLineRvizPublisher->publish(referenceLineCreator->referenceLineToRviz());
  }

  void PlanningProcess::spawnVehicle(const std::shared_ptr<VehicleInfoBase>& vehicle)
  {
    geometry_msgs::msg::TransformStamped spawn;
    spawn.header.frame_id = configReaderProcess->getPNCMap().frame_;
    spawn.header.stamp = this->get_clock()->now();
    spawn.child_frame_id = vehicle->getVehicleChildFrame();
    spawn.transform.translation.x = vehicle->getVehiclePose().pose.position.x;
    spawn.transform.translation.y = vehicle->getVehiclePose().pose.position.y;
    spawn.transform.translation.z = vehicle->getVehiclePose().pose.position.z;
    spawn.transform.rotation.x = vehicle->getVehiclePose().pose.orientation.x;
    spawn.transform.rotation.y = vehicle->getVehiclePose().pose.orientation.y;
    spawn.transform.rotation.z = vehicle->getVehiclePose().pose.orientation.z;
    spawn.transform.rotation.w = vehicle->getVehiclePose().pose.orientation.w;
    RCLCPP_INFO(this->get_logger(), "vehicle %s is spawned, x= %.2f, y = %.2f", spawn.child_frame_id.c_str(),
                vehicle->getVehiclePose().pose.position.x, vehicle->getVehiclePose().pose.position.y);
    tfBroadcaster->sendTransform(spawn);
  }

  void PlanningProcess::getVehicleLocation(const std::shared_ptr<VehicleInfoBase>& vehicle)
  {
    try
    {
      geometry_msgs::msg::PoseStamped point;
      auto ts = tfBuffer->lookupTransform(configReaderProcess->getPNCMap().frame_, vehicle->getVehicleChildFrame(),
                                          tf2::TimePointZero);
      point.header = ts.header;
      point.pose.position.x = ts.transform.translation.x;
      point.pose.position.y = ts.transform.translation.y;
      point.pose.position.z = ts.transform.translation.z;
      point.pose.orientation.x = ts.transform.rotation.x;
      point.pose.orientation.y = ts.transform.rotation.y;
      point.pose.orientation.z = ts.transform.rotation.z;
      point.pose.orientation.w = ts.transform.rotation.w;
      vehicle->updateVehiclePose(point);
    }
    catch (const tf2::LookupException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", e.what());
    }
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
      if (pncMap.midline.points.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Received empty PNCMap from server!");
        return false;
      }

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
      if (globalPath.poses.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Received empty global path from server!");
        return false;
      }

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
