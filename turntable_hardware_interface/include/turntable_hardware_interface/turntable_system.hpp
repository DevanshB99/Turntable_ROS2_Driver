#ifndef TURNTABLE_HARDWARE_INTERFACE__TURNTABLE_SYSTEM_HPP_
#define TURNTABLE_HARDWARE_INTERFACE__TURNTABLE_SYSTEM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <thread>
#include <string>
#include <vector>

namespace turntable_hardware_interface
{

class TurntableSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TurntableSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  // Node and communication
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_angle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::string joint_name_;
  std::string target_angle_topic_;
  std::string joint_states_topic_;
  bool publish_command_;
  double hw_position_;
  double hw_velocity_;
  double hw_command_;
  double hw_speed_scaling_factor_;
  
  rclcpp::Time last_update_time_;
  bool hardware_connected_;
};

}  // namespace turntable_hardware_interface

#endif  // TURNTABLE_HARDWARE_INTERFACE__TURNTABLE_SYSTEM_HPP_