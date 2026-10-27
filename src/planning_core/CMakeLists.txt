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
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 头文件搜索路径（common_type 和各地图创建器子目录）
include_directories(
  src/common/common_type
  src/pnc_map_creator
  src/pnc_map_creator/pnc_map_straight
  src/global_planner/global_planner_normal
  src/global_planner
  src/common/math
  src/reference_line
  src/common/config_reader
  src/planning_process
  src/vehicle_info
  src/vehicle_info/ego_car
  src/move_cmd
  src/vehicle_info/traffic_participants
  src/decision_center
  src/local_path
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 编译 pnc_map_server
add_executable(pnc_map_server
  src/pnc_map_creator/pnc_map_server.cpp
  src/pnc_map_creator/pnc_map_straight/pnc_map_creator_straight.cpp
  src/common/config_reader/config_reader.cpp
)
ament_target_dependencies(pnc_map_server rclcpp base_msgs geometry_msgs visualization_msgs nav_msgs ament_index_cpp)
target_link_libraries(pnc_map_server yaml-cpp::yaml-cpp)

# 编译 global_path_server
add_executable(global_path_server
  src/global_planner/global_path_server.cpp
  src/global_planner/global_planner_normal/global_planner_normal.cpp
)

ament_target_dependencies(global_path_server rclcpp base_msgs geometry_msgs visualization_msgs nav_msgs)

# 编译 planning_process
add_executable(planning_process
  src/planning_process/planning_process_main.cpp
  src/planning_process/planning_process.cpp
  src/common/math/curve.cpp
  src/common/config_reader/config_reader.cpp
  src/reference_line/reference_line_creator.cpp
  src/reference_line/reference_line_smoother.cpp
  src/vehicle_info/ego_car/ego_car.cpp
  src/decision_center/decision_center.cpp
  src/vehicle_info/traffic_participants/tp_base.cpp
  src/local_path/local_path_creator.cpp
  src/local_path/qp_local_path_solver.cpp
)

ament_target_dependencies(planning_process rclcpp base_msgs geometry_msgs visualization_msgs nav_msgs ament_index_cpp tf2 tf2_ros)

target_link_libraries(planning_process yaml-cpp::yaml-cpp OsqpEigen::OsqpEigen Eigen3::Eigen)

add_executable(ego_car_move_cmd
  src/move_cmd/ego_car_move_cmd.cpp
  src/common/config_reader/config_reader.cpp
  src/vehicle_info/ego_car/ego_car.cpp
  src/common/math/curve.cpp
)

ament_target_dependencies(ego_car_move_cmd rclcpp base_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros ament_index_cpp)
target_link_libraries(ego_car_move_cmd yaml-cpp::yaml-cpp)

add_executable(tp_move_cmd
  src/move_cmd/tp_move_cmd.cpp
  src/common/config_reader/config_reader.cpp
  src/common/math/curve.cpp
  src/vehicle_info/traffic_participants/tp_base.cpp
)

ament_target_dependencies(tp_move_cmd rclcpp base_msgs geometry_msgs visualization_msgs nav_msgs ament_index_cpp tf2 tf2_ros)
target_link_libraries(tp_move_cmd yaml-cpp::yaml-cpp)
# 安装到 lib/planning_core
install(TARGETS pnc_map_server global_path_server planning_process ego_car_move_cmd tp_move_cmd
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件到 share/planning_core/config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()