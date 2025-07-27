cmake_minimum_required(VERSION 3.5)
project (articubot_gpio_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/articubot_gpio_hardware.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
