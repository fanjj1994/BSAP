#include "planning_process.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Planning::PlanningProcess>();
    if(!node->process())
    {
        RCLCPP_ERROR(rclcpp::get_logger("planning_process_main"), "PlanningProcess node start failed.");
        rclcpp::shutdown();
        return Planning::PlanningProcess::PROCESS_FAILURE;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}