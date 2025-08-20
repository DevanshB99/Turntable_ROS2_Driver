#ifndef MICROROS_INTERFACE_H
#define MICROROS_INTERFACE_H

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/joint_state.h>
#include <trajectory_msgs/msg/joint_trajectory.h>

#include "config.h"

class MicroROSInterface {
private:
  // micro-ROS entities
  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rcl_node_t node_;
  rcl_timer_t timer_;
  rclc_executor_t executor_;
  
  // Publishers
  rcl_publisher_t joint_state_pub_;
  rcl_publisher_t status_pub_;
  
  // Subscribers  
  rcl_subscription_t target_angle_sub_;
  rcl_subscription_t trajectory_sub_;
  
  // Messages
  sensor_msgs__msg__JointState joint_state_msg_;
  std_msgs__msg__Bool status_msg_;
  std_msgs__msg__Float32 target_angle_msg_;
  trajectory_msgs__msg__JointTrajectory trajectory_msg_;
  
  // Message storage
  char joint_name_storage_[32];
  double joint_position_[1];
  double joint_velocity_[1];
  double joint_effort_[1];
  
  bool micro_ros_connected_;
  unsigned long last_ping_time_;
  
  // Static callback wrappers
  static void targetAngleCallback(const void* msg);
  static void trajectoryCallback(const void* msg);
  static void timerCallback(rcl_timer_t* timer, int64_t last_call_time);
  
  void setupStaticMessages();
  bool connectToAgent();
  
public:
  MicroROSInterface();
  void begin();
  void spin();
  void publishJointState(double position_deg, double velocity_deg_per_sec);
  void publishStatus(bool online);
  bool isConnected() const { return micro_ros_connected_; }
  
  // Callback handlers (called by static wrappers)
  void handleTargetAngle(const std_msgs__msg__Float32* msg);
  void handleTrajectory(const trajectory_msgs__msg__JointTrajectory* msg);
};

extern MicroROSInterface micro_ros;

#endif // MICROROS_INTERFACE_H