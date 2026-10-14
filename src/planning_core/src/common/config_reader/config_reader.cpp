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

  void ConfigReader::readVehicleConfig()
  {
    try
    {
      egoCar.frame_ = config["vehicle"]["ego_car"]["frame"].as<std::string>();
      egoCar.id_ = config["vehicle"]["ego_car"]["id"].as<uint8>();
      egoCar.length_ = config["vehicle"]["ego_car"]["length"].as<float64>();
      egoCar.width_ = config["vehicle"]["ego_car"]["width"].as<float64>();
      egoCar.pose_x_ = config["vehicle"]["ego_car"]["pose_x"].as<float64>();
      egoCar.pose_y_ = config["vehicle"]["ego_car"]["pose_y"].as<float64>();
      egoCar.pose_theta_ = config["vehicle"]["ego_car"]["pose_theta"].as<float64>();
      egoCar.set_speed_ = config["vehicle"]["ego_car"]["set_speed"].as<float64>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load vehicle config: %s", e.what());
      throw;
    }
  }

} // namespace Planning