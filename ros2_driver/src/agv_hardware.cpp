#include "agv_base_control/agv_hardware.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agv_base_control
{

// ==========================================================
// 1. 硬件初始化：解析 URDF 传来的参数
// ==========================================================
hardware_interface::CallbackReturn AGVHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化状态载体
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==========================================================
// 2. 导出状态与指令接口
// ==========================================================
std::vector<hardware_interface::StateInterface>
AGVHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AGVHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

// ==========================================================
// 3. 激活硬件：打开并配置 Linux 串口
// ==========================================================
hardware_interface::CallbackReturn AGVHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AGVHardwareInterface"), "正在打开串口 /dev/ttyUSB0 ...");
  
  // Linux 串口配置
  serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("AGVHardwareInterface"), "串口打开失败！请检查权限。");
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios options;
  tcgetattr(serial_fd_, &options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag |= (CLOCAL | CREAD); // 启用接收器，忽略控制线
  options.c_cflag &= ~PARENB;          // 无校验
  options.c_cflag &= ~CSTOPB;          // 1个停止位
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;              // 8个数据位
  tcsetattr(serial_fd_, TCSANOW, &options);

  RCLCPP_INFO(rclcpp::get_logger("AGVHardwareInterface"), "串口打开成功，底盘已激活！");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AGVHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ >= 0) close(serial_fd_);
  RCLCPP_INFO(rclcpp::get_logger("AGVHardwareInterface"), "底盘已休眠，串口关闭。");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==========================================================
// 4. 读取反馈：从 STM32 接收里程计 (read)
// ==========================================================
hardware_interface::return_type AGVHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // 这里省略了复杂的串口拼包逻辑，为了清晰展示核心思想
  // 实际开发中，这里会读取 8 字节 AA 55 帧，提取出 STM32 发来的左/右轮脉冲数
  // 假设我们解析出了 left_pulse_per_50ms 和 right_pulse_per_50ms

  /*
  // 核心数学转换：将【50ms内的脉冲数】转换为 ROS2 的【rad/s】与【rad】
  // 假设电机转一圈是 1560 个脉冲（这个数值需要你根据减速比确定）
  double ticks_per_rev = 720.0; 
  
  // 1. 算速度 (rad/s) = (脉冲/50ms) * 20(变成每秒) / ticks_per_rev * 2Pi
  hw_velocities_[0] = (left_pulse_per_50ms * 20.0 / ticks_per_rev) * (2 * M_PI);
  hw_velocities_[1] = (right_pulse_per_50ms * 20.0 / ticks_per_rev) * (2 * M_PI);

  // 2. 算位置 (rad) = 速度 * 周期(dt)
  hw_positions_[0] += hw_velocities_[0] * period.seconds();
  hw_positions_[1] += hw_velocities_[1] * period.seconds();
  */

  return hardware_interface::return_type::OK;
}

// ==========================================================
// 5. 下发指令：向 STM32 发送目标速度 (write)
// ==========================================================
hardware_interface::return_type AGVHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  double ticks_per_rev = 720.0;

  // 左轮目标 (rad/s -> rev/s -> ticks/s -> ticks/50ms)
  int16_t left_target = (hw_commands_[0] / (2 * M_PI)) * ticks_per_rev * 0.05;
  int16_t right_target = (hw_commands_[1] / (2 * M_PI)) * ticks_per_rev * 0.05;

  // 组装 8 字节 Hex 帧
  uint8_t tx_buf[8];
  tx_buf[0] = 0xAA;
  tx_buf[1] = 0x55;
  tx_buf[2] = (left_target >> 8) & 0xFF;
  tx_buf[3] = left_target & 0xFF;
  tx_buf[4] = (right_target >> 8) & 0xFF;
  tx_buf[5] = right_target & 0xFF;
  tx_buf[6] = (tx_buf[2] + tx_buf[3] + tx_buf[4] + tx_buf[5]) & 0xFF; // Checksum
  tx_buf[7] = 0x0A;
  RCLCPP_INFO(rclcpp::get_logger("AGVHardwareInterface"), 
      "ROS speed : %.2f rad/s -> 算出的脉冲: %d -> 左轮Hex: %02X %02X", 
      hw_commands_[0], left_target, tx_buf[2], tx_buf[3]);
  // 写入串口
  if (serial_fd_ >= 0) {
    ::write(serial_fd_, tx_buf, 8);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace agv_base_control

// 插件导出宏定义 (告诉 ROS2 控制管理器这是一个标准插件)
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  agv_base_control::AGVHardwareInterface, hardware_interface::SystemInterface)