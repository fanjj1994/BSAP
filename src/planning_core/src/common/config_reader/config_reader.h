#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include "common_type.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace Planning
{
    struct PNCMapStruct
    {
        std::string frame_ = "";
        uint32 type_ = 0;
        float64 road_length_ = 0.0;
        float64 road_half_width_ = 0.0;
        float64 segment_len_ = 0.0;
    };

    struct ReferenceLineStruct
    {
        uint32 front_size_ = 0;
        uint32 back_size_ = 0;
    };

    struct ProcessStruct
    {
        float64 cycle_time_ = 0.0;
        float64 perception_range_ = 0.0;
    };

    struct GlobalPathStruct
    {
        uint32 type_ = 0;
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

        PNCMapStruct getPNCMap() const { return pncMap; }
        ReferenceLineStruct getReferenceLine() const { return referenceLine; }
        ProcessStruct getProcess() const { return process; }
        GlobalPathStruct getGlobalPath() const { return globalPath; }
    private:
        YAML::Node config;
        PNCMapStruct pncMap;
        ReferenceLineStruct referenceLine;
        ProcessStruct process;
        GlobalPathStruct globalPath;
    };
}  // namespace Planning
#endif  // CONFIG_READER_H_
