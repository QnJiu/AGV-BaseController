cmake_minimum_required(VERSION 3.8)
project(agv_base_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED) #生命周期节点


# 将cpp 文件编译成动态链接库 (Shared Library)
add_library(agv_hardware SHARED
  src/agv_hardware.cpp
)

# 包含头文件目录
target_include_directories(agv_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖包
ament_target_dependencies(agv_hardware
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)


install(
  TARGETS agv_hardware
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
# 安装插件声明文件
install(
  FILES agv_base_control.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# 向 ROS 2 的 Ament 索引强制注册这个插件
pluginlib_export_plugin_description_file(hardware_interface agv_base_control.xml)
ament_package()