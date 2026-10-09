cmake_minimum_required(VERSION 3.8)
project(planning_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(yaml-cpp REQUIRED)

# 头文件搜索路径（common_type 和各地图创建器子目录）
include_directories(
  src/common/common_type
  src/pnc_map_creator
  src/pnc_map_creator/pnc_map_straight
  src/global_planner/global_planner_normal
  src/global_planner
  src/common/math
  src/reference_line
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 编译 pnc_map_server
add_executable(pnc_map_server
  src/pnc_map_creator/pnc_map_server.cpp
  src/pnc_map_creator/pnc_map_straight/pnc_map_creator_straight.cpp
)
ament_target_dependencies(pnc_map_server rclcpp base_msgs geometry_msgs visualization_msgs nav_msgs)

# 编译 global_path_server
add_executable(global_path_server
  src/global_planner/global_path_server.cpp
  src/global_planner/global_planner_normal/global_planner_normal.cpp
)

ament_target_dependencies(global_path_server rclcpp base_msgs geometry_msgs visualization_msgs nav_msgs)

# 安装到 lib/planning_core
install(TARGETS pnc_map_server global_path_server
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件到 share/planning_core/config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()