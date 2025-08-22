#ifndef MICROROS_INTERFACE_H
#define MICROROS_INTERFACE_H
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/joint_state.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include "config.h"

class MicroROSInterface {
private:
  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;

  //publisher - joint_state
  rcl_publisher_t joint_state_pub_;

  //subscriber - /target_angle, /joint_trajecory, /commands
  rcl_subscription_t target_angle_sub_;
  rcl_subscription_t trajectory_sub_;
  rcl_subscription_t commands_sub_;
  
  //messages - sensor_msgs: encoder, Float32: target_angle/keyboard_publisher_node, Float64: turntable_forward_position_controller, trajectory_msgs: JointTrajectory
  sensor_msgs__msg__JointState joint_state_msg_;
  std_msgs__msg__Float32 target_angle_msg_;
  trajectory_msgs__msg__JointTrajectory trajectory_msg_;
  std_msgs__msg__Float64MultiArray commands_msg_;
  
  char joint_name_storage_[32];
  char frame_id_storage_[32];
  double joint_position_[1];
  double joint_velocity_[1];
  double joint_effort_[1];
  bool micro_ros_connected_;
  unsigned long last_status_time_;
  
  //executing a trajectory
  bool executing_trajectory_;
  int current_trajectory_point_;
  unsigned long trajectory_start_time_;
  static void targetAngleCallback(const void* msg);
  static void trajectoryCallback(const void* msg);
  static void commandsCallback(const void* msg);
  void setupStaticMessages();
  bool connectToAgent();
  bool setupWiFi();
  
public:
  MicroROSInterface();
  void begin();
  void spin();
  void publishJointState(double position_deg, double velocity_deg_per_sec);
  void publishStatus(bool online);
  bool isConnected() const { return micro_ros_connected_; }
  void handleTargetAngle(const std_msgs__msg__Float32* msg);
  void handleTrajectory(const trajectory_msgs__msg__JointTrajectory* msg);
  void handleCommands(const std_msgs__msg__Float64MultiArray* msg);
};

extern MicroROSInterface micro_ros;
#endif // MICROROS_INTERFACE_H