#include "config_reader.h"

namespace Planning
{
  ConfigReader::ConfigReader()
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("planning_core");
    config = YAML::LoadFile(package_share_directory + "/config/planning_config.yaml");
  }
  void ConfigReader::readPNCMapConfig()
  {
    try
    {
      pncMap.frame_ = config["pnc_map"]["frame"].as<std::string>();
      pncMap.type_ = config["pnc_map"]["type"].as<uint32>();
      pncMap.road_length_ = config["pnc_map"]["road_length"].as<float64>();
      pncMap.road_half_width_ = config["pnc_map"]["road_half_width"].as<float64>();
      pncMap.segment_len_ = config["pnc_map"]["segment_len"].as<float64>();
      pncMap.lane_width_  = config["pnc_map"]["lane_width"].as<float64>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load pnc_map config: %s", e.what());
      throw;
    }
  }

  void ConfigReader::readReferenceLineConfig()
  {
    try
    {
      referenceLine.front_size_ = config["reference_line"]["front_size"].as<uint32>();
      referenceLine.back_size_ = config["reference_line"]["back_size"].as<uint32>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load reference_line config: %s", e.what());
      throw;
    }
  }

  void ConfigReader::readProcessConfig()
  {
    try
    {
      process.cycle_time_ = config["planning_process"]["cycle_time"].as<float64>();
      process.perception_range_ = config["planning_process"]["perception_range"].as<float64>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load planning_process config: %s", e.what());
      throw;
    }
  }

  void ConfigReader::readGlobalPathConfig()
  {
    try
    {
      globalPath.type_ = config["global_path"]["type"].as<uint32>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load global_path config: %s", e.what());
      throw;
    }
  }

  void ConfigReader::readVehiclesConfig()
  {
    try
    {
      for(auto it : config["vehicle"])
      {
        EgoCarStruct vehicleStruct;
        vehicleStruct.frame_ = it.second["frame"].as<std::string>();
        vehicleStruct.id_ = it.second["id"].as<uint8>();
        vehicleStruct.length_ = it.second["length"].as<float64>();
        vehicleStruct.width_ = it.second["width"].as<float64>();
        vehicleStruct.pose_x_ = it.second["pose_x"].as<float64>();
        vehicleStruct.pose_y_ = it.second["pose_y"].as<float64>();
        vehicleStruct.pose_theta_ = it.second["pose_theta"].as<float64>();
        vehicleStruct.set_speed_ = it.second["set_speed"].as<float64>();
        vehiclePairs.emplace(vehicleStruct.id_,vehicleStruct);
      }
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load vehicles config: %s", e.what());
      throw;
    }
  }

  void ConfigReader::readDecisionConfig()
  {
    try
    {
      readPNCMapConfig();
      readReferenceLineConfig();
      readLocalPathConfig();
      decision.lat_safe_margin_ = config["decision"]["lat_safe_margin"].as<float32>();
      decision.long_safe_margin_ = config["decision"]["long_safe_margin"].as<float32>();
    }
    catch(const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"),"Failed to load decision config: %s", e.what());
      throw;
    }   
  }

  // local path（依赖 pnc_map/global_path/reference_line 三套餐；键名与 yaml 段名 local_path 对齐）
  void ConfigReader::readLocalPathConfig()
  {
    try
    {
      readPNCMapConfig();
      readGlobalPathConfig();
      readReferenceLineConfig();
      readVehiclesConfig();
      localPath.path_size_ = config["local_path"]["path_size"].as<uint32>();
      localPath.interval_ = config["local_path"]["interval"].as<float64>();
      localPath.weight_smooth_ = config["local_path"]["weight_smooth"].as<float64>();
      localPath.weight_slope_ = config["local_path"]["weight_slope"].as<float64>();
      localPath.weight_offset_ = config["local_path"]["weight_offset"].as<float64>();
    }
    catch(const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load local_path config: %s", e.what());
      throw;
    }
  }

} // namespace Planning