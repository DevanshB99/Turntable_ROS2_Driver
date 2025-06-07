#include "turntable_hardware_interface/turntable_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>

namespace turntable_hardware_interface
{

hardware_interface::CallbackReturn TurntableSystem::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  info_ = info;
  joint_name_ = info_.joints[0].name;
  hw_position_ = 0.0;
  hw_velocity_ = 0.0;
  hw_command_ = 0.0;
  hw_speed_scaling_factor_ = 1.0; 
  hardware_connected_ = false;
  
  // Parameters from YAML
  if (info_.hardware_parameters.count("publish_command"))
  {
    publish_command_ = std::stoi(info_.hardware_parameters.at("publish_command"));
  }
  else
  {
    publish_command_ = 1; // Default value
    RCLCPP_WARN(rclcpp::get_logger("TurntableSystem"), 
               "Parameter 'publish_command' not found, using default: %d", publish_command_);
  }
  
  if (info_.hardware_parameters.count("target_angle_topic"))
  {
    target_angle_topic_ = info_.hardware_parameters.at("target_angle_topic");
  }
  else
  {
    target_angle_topic_ = "/target_angle"; // Default value
    RCLCPP_WARN(rclcpp::get_logger("TurntableSystem"), 
               "Parameter 'target_angle_topic' not found, using default: %s", target_angle_topic_.c_str());
  }
  
  if (info_.hardware_parameters.count("joint_states_topic"))
  {
    joint_states_topic_ = info_.hardware_parameters.at("joint_states_topic");
  }
  else
  {
    joint_states_topic_ = "/turntables/joint_states"; // Default value
    RCLCPP_WARN(rclcpp::get_logger("TurntableSystem"), 
               "Parameter 'joint_states_topic' not found, using default: %s", joint_states_topic_.c_str());
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurntableSystem::on_activate(const rclcpp_lifecycle::State &)
{
  rclcpp::NodeOptions options;
  node_ = std::make_shared<rclcpp::Node>("turntable_system_node", options);

  // Initialize state tracking timestamp
  last_update_time_ = node_->get_clock()->now();
  hardware_connected_ = false;

  if (publish_command_)
  {
    target_angle_pub_ = node_->create_publisher<std_msgs::msg::Float32>(target_angle_topic_, 10);
  }

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TurntableSystem::joint_state_callback, this, std::placeholders::_1));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(node_->get_logger(), "[TurntableSystem] Activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurntableSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (executor_)
    executor_->cancel();
  if (executor_thread_.joinable())
    executor_thread_.join();

  target_angle_pub_.reset();
  joint_state_sub_.reset();
  executor_.reset();
  node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TurntableSystem::read(const rclcpp::Time &time, const rclcpp::Duration &)
{
  // Check if we've received state updates recently
  auto time_since_last_update = time - last_update_time_;
  if (time_since_last_update.seconds() > 1.0) {
    if (hardware_connected_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, 
                         "No joint state updates received for %.2f seconds", 
                         time_since_last_update.seconds());
    }
    hardware_connected_ = false;
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurntableSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (publish_command_ && target_angle_pub_)
  {
    std_msgs::msg::Float32 msg;
    // No sign flip
    msg.data = static_cast<float>(hw_command_ * 180.0 / M_PI);
    target_angle_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> TurntableSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_);
  interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_);
  interfaces.emplace_back("speed_scaling", "speed_scaling_factor", &hw_speed_scaling_factor_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> TurntableSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &hw_command_);
  return interfaces;
}

void TurntableSystem::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  auto it = std::find(msg->name.begin(), msg->name.end(), joint_name_);
  if (it != msg->name.end())
  {
    auto idx = std::distance(msg->name.begin(), it);
    if (idx >= 0 && static_cast<size_t>(idx) < msg->position.size()) {
      // No sign flip needed - using direct value
      hw_position_ = msg->position[idx];
    }
    
    if (idx >= 0 && static_cast<size_t>(idx) < msg->velocity.size()) {
      // No sign flip needed
      hw_velocity_ = msg->velocity[idx];
    }
    
    last_update_time_ = node_->get_clock()->now();
    hardware_connected_ = true;
  }
}

}  // namespace turntable_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(turntable_hardware_interface::TurntableSystem, hardware_interface::SystemInterface)