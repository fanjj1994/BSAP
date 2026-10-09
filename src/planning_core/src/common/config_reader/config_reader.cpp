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
    }
    catch(const YAML::Exception& e)
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
    catch(const YAML::Exception& e)
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
    catch(const YAML::Exception& e)
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
    catch(const YAML::Exception& e)
    {
         RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load global_path config: %s", e.what());
        throw;
    }
    
  }

} // namespace Planning