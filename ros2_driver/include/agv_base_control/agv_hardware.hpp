#ifndef AGV_BASE_CONTROL__AGV_HARDWARE_HPP_
#define AGV_BASE_CONTROL__AGV_HARDWARE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace agv_base_control
{
class AGVHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AGVHardwareInterface)

  // 1. 硬件初始化：解析 URDF 中的参数
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // 2. 导出状态接口：告诉 ROS2编码器的位置和速度
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 3. 导出指令接口：告诉 ROS2 角速度指令
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 4. 激活与挂起：打开/关闭串口
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 5.读写循环：20Hz 实时读取串口反馈、下发速度指令
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 串口设备描述符
  int serial_fd_;

  // ROS2 体系下的左右轮数据载体
  std::vector<double> hw_commands_;   // 接收下发的目标速度 (rad/s)
  std::vector<double> hw_positions_;  // 上报当前的里程计位置 (rad)
  std::vector<double> hw_velocities_; // 上报当前的真实速度 (rad/s)
};

}  // namespace agv_base_control

#endif  // AGV_BASE_CONTROL__AGV_HARDWARE_HPP_