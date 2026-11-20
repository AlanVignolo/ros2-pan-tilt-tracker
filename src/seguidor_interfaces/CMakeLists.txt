cmake_minimum_required(VERSION 3.10)
project(seguidor_interfaces)

# ament_cmake: el sistema de construcción de ROS2 para paquetes CMake.
find_package(ament_cmake REQUIRED)
# rosidl: el generador que convierte los .srv en código de Python y C++.
find_package(rosidl_default_generators REQUIRED)

# Le dice al generador qué archivos de interfaz tiene este paquete.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTarget.srv"
  "action/Buscar.action"
)

ament_package()