#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include "common_type.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace Planning
{
    struct EgoCarStruct
    {
        std::string frame_ = "";
        uint8 id_ = 0U;
        float64 length_ = 0.0;
        float64 width_ = 0.0;
        float64 pose_x_ = 0.0;
        float64 pose_y_ = 0.0;
        float64 pose_theta_ = 0.0;
        float64 set_speed_ = 0.0;
    };

    struct PNCMapStruct
    {
        std::string frame_ = "";
        uint32 type_ = 0U;
        float64 road_length_ = 0.0;
        float64 road_half_width_ = 0.0;
        float64 segment_len_ = 0.0;
    };

    struct ReferenceLineStruct
    {
        uint32 front_size_ = 0U;
        uint32 back_size_ = 0U;
    };

    struct ProcessStruct
    {
        float64 cycle_time_ = 0.0;
        float64 perception_range_ = 0.0;
    };

    struct GlobalPathStruct
    {
        uint32 type_ = 0U;
    };

    class ConfigReader
    {
    public:
        ConfigReader();
        ConfigReader(const ConfigReader &) = delete;
        ~ConfigReader() = default;

        void readPNCMapConfig();
        void readReferenceLineConfig();
        void readProcessConfig();
        void readGlobalPathConfig();
        void readVehicleConfig();

        PNCMapStruct getPNCMap() const { return pncMap; }
        ReferenceLineStruct getReferenceLine() const { return referenceLine; }
        ProcessStruct getProcess() const { return process; }
        GlobalPathStruct getGlobalPath() const { return globalPath; }
        EgoCarStruct getEgoCar() const { return egoCar; }
    private:
        YAML::Node config;
        PNCMapStruct pncMap;
        ReferenceLineStruct referenceLine;
        ProcessStruct process;
        GlobalPathStruct globalPath;
        EgoCarStruct egoCar;
    };
}  // namespace Planning
#endif  // CONFIG_READER_H_
