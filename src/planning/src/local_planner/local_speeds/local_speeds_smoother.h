#ifndef LOCAL_SPEEDS_SMOOTHER_H_
#define LOCAL_SPEEDS_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "config_reader.h"

#include <cmath>

namespace Planning
{
    using base_msgs::msg::LocalSpeeds;

    class LocalSpeedsSmoother //速度平滑规划器
    {
    public:
        LocalSpeedsSmoother(); 
        void smooth_local_speeds(LocalSpeeds speeds); // 平滑速度

    private:
        std::unique_ptr<ConfigReader> local_speeds_config_; // 配置

    };
}  // namespace Planning
#endif  // LOCAL_SPEEDS_SMOOTHER_H_
