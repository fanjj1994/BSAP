cmake_minimum_required(VERSION 3.8)
project(base_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PNCMap.msg"
  "msg/Referline.msg"
  "msg/ReferlinePoint.msg"
  "msg/LocalPath.msg"
  "msg/LocalPathPoint.msg"
  "srv/PNCMapService.srv"
  "srv/GlobalPathService.srv"
  DEPENDENCIES std_msgs geometry_msgs visualization_msgs nav_msgs
)

ament_package()
