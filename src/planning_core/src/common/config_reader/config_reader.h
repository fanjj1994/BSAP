#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include "common_type.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace Planning
{
    struct EgoCarStruct
    {
        std::string frame_ = "";    // child frame name
        uint8 id_ = 0U;
        float64 length_ = 0.0;      // unit: m
        float64 width_ = 0.0;       // unit: m
        float64 pose_x_ = 0.0;      // unit: m
        float64 pose_y_ = 0.0;      // unit: m
        float64 pose_theta_ = 0.0;
        float64 set_speed_ = 0.0;   // unit: m/s
    };

    struct PNCMapStruct
    {
        std::string frame_ = "";        // world frame name
        uint32 type_ = 0U;              // 0: straight; 1: turn
        float64 road_length_ = 0.0;     // unit: m
        float64 road_half_width_ = 0.0; // unit: m
        float64 segment_len_ = 0.0;     // unit: m
        float64 lane_width_ = 0.0;      // unit: m
    };

    struct ReferenceLineStruct
    {
        uint32 front_size_ = 0U;    // match point 前方点数
        uint32 back_size_ = 0U;     // match point 后方点数
    };

    struct ProcessStruct
    {
        float64 cycle_time_ = 0.0;      // unit: s
        float64 perception_range_ = 0.0; // unit: m
    };

    struct GlobalPathStruct
    {
        uint32 type_ = 0U;          // 0: normal
    };

    struct DecisionStruct
    {
        float32 lat_safe_margin_ = 0.0;   // unit: m
        float32 long_safe_margin_ = 0.0;  // unit: m
    };

    struct LocalPathStruct
    {
        uint32 path_size_ = 100U;       // QP 网格点数
        float64 interval_ = 0.5;        // unit: m，网格间隔
        float64 weight_smooth_ = 100.0; // w1
        float64 weight_slope_ = 10.0;   // w2
        float64 weight_offset_ = 1.0;   // w3
    };

    class ConfigReader
    {
    public:
        ConfigReader();
        ConfigReader(const ConfigReader &) = delete;
        ~ConfigReader() = default;

        // pnc_map
        void readPNCMapConfig();
        inline PNCMapStruct getPNCMap() const { return pncMap; }

        // reference line
        void readReferenceLineConfig();
        inline ReferenceLineStruct getReferenceLine() const { return referenceLine; }

        // process
        void readProcessConfig();
        inline ProcessStruct getProcess() const { return process; }

        // global path
        void readGlobalPathConfig();
        inline GlobalPathStruct getGlobalPath() const { return globalPath; }

        // vehicle
        void readVehiclesConfig();
        inline EgoCarStruct getEgoCar() const { return vehiclePairs.at(0U); }
        inline const std::unordered_map<uint8,EgoCarStruct> &getVehiclePairs() const { return vehiclePairs; }

        // decision
        void readDecisionConfig();
        inline DecisionStruct getDecision() const { return decision; }

        // local path
        void readLocalPathConfig();
        inline LocalPathStruct getLocalPath() const { return localPath; }

    private:
        YAML::Node config;

        // pnc_map
        PNCMapStruct pncMap;

        // reference_line
        ReferenceLineStruct referenceLine;

        // process
        ProcessStruct process;

        // global_path
        GlobalPathStruct globalPath;

        // vehicle
        std::unordered_map<uint8, EgoCarStruct> vehiclePairs;

        // decision
        DecisionStruct decision;

        // local_path
        LocalPathStruct localPath;
    };
}  // namespace Planning
#endif  // CONFIG_READER_H_
