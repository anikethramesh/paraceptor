cmake_minimum_required(VERSION 3.5)
project(paraceptor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneCoords.msg"
)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()



