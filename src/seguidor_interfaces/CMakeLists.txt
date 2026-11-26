cmake_minimum_required(VERSION 3.10)
project(seguidor_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTarget.srv"
  "action/Buscar.action"
)

ament_package()
