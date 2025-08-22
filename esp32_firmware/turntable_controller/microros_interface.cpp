#include "microros_interface.h"
#include "stepper_control.h"

static MicroROSInterface* micro_ros_instance = nullptr;
MicroROSInterface::MicroROSInterface()
  : micro_ros_connected_(false)
  , last_status_time_(0)
  , executing_trajectory_(false)
  , current_trajectory_point_(0)
  , trajectory_start_time_(0)
{
  micro_ros_instance = this;
}

void MicroROSInterface::begin() {
  Serial.println("Initializing micro-ROS...");
  // Setting up the WiFi
  int wifi_attempts = 0;
  while (wifi_attempts < 2 && !setupWiFi()) {
    wifi_attempts++;
    Serial.printf("WiFi attempt %d failed. Waiting 5 seconds before retry...\n", wifi_attempts);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    if (wifi_attempts < 2) {
      delay(2000);
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi after attempts");
    Serial.println("Please check your WiFi credentials and network:");
    Serial.printf("SSID: %s\n", WIFI_SSID);
    Serial.println("Make sure the network is 2.4GHz (ESP32 doesn't support 5GHz)");
    return;
  }
  //Setting micro-ROS transport (EXTREMELY CRUCIAL)
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  delay(2000);
  //connect to an agent
  int agent_attempts = 0;
  while (agent_attempts < 3 && !connectToAgent()) {
    agent_attempts++;
    Serial.printf("micro-ROS agent connection attempt %d failed. Retrying...\n", agent_attempts);
    delay(2000);
  }
  
  if (micro_ros_connected_) {
    Serial.println("micro-ROS connected successfully");
  } else {
    Serial.println("Failed to connect to micro-ROS agent");
    Serial.printf("Please check agent is running at %s:%d\n", AGENT_IP, AGENT_PORT);
  }
}
bool MicroROSInterface::setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi"); 
  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < 20000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    return true;
  } 
  else {
    Serial.println("\nWiFi connection failed");
    Serial.println("WiFi Debug Info:");
    Serial.printf("SSID: %s\n", WIFI_SSID);
    Serial.printf("Status: %d\n", WiFi.status());
    Serial.println("Attempting reconnection...");
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifi_start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < 20000) {
      delay(1000);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.printf("WiFi connected on retry: %s\n", WiFi.localIP().toString().c_str());
      return true;
    } else {
      Serial.println("\nSecond WiFi attempt failed");
      return false;
    }
  }
}

bool MicroROSInterface::connectToAgent() {
  allocator_ = rcl_get_default_allocator();
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) {
    Serial.println("Failed to initialize support");
    return false;
  }
  //creating uros node
  if (rclc_node_init_default(&node_, MICROROS_NODE_NAME, MICROROS_NAMESPACE, &support_) != RCL_RET_OK) {
    Serial.println("Failed to create node");
    return false;
  }
  //joint_state_publisher
  if (rclc_publisher_init_best_effort(&joint_state_pub_, &node_, 
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                     JOINT_STATES_TOPIC) != RCL_RET_OK) {
    Serial.println("Failed to create joint state publisher");
    return false;
  }
  //target_angle
  if (rclc_subscription_init_best_effort(&target_angle_sub_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                        TARGET_ANGLE_TOPIC) != RCL_RET_OK) {
    Serial.println("Failed to create target angle subscription");
    return false;
  }
  //joint_trajectory
  if (rclc_subscription_init_best_effort(&trajectory_sub_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
                                        TRAJECTORY_TOPIC) != RCL_RET_OK) {
    Serial.println("Failed to create trajectory subscription");
    return false;
  }
  //turntable_forward_position_controller - /commands
  if (rclc_subscription_init_best_effort(&commands_sub_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
                                        COMMANDS_TOPIC) != RCL_RET_OK) {
    Serial.println("Failed to create commands subscription");
    return false;
  }
  if (rclc_executor_init(&executor_, &support_.context, 3, &allocator_) != RCL_RET_OK) {
    Serial.println("Failed to create executor");
    return false;
  }
  if (rclc_executor_add_subscription(&executor_, &target_angle_sub_, &target_angle_msg_,
                                    &targetAngleCallback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Failed to add target angle subscription");
    return false;
  }
  if (rclc_executor_add_subscription(&executor_, &trajectory_sub_, &trajectory_msg_,
                                    &trajectoryCallback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Failed to add trajectory subscription");
    return false;
  }
  if (rclc_executor_add_subscription(&executor_, &commands_sub_, &commands_msg_,
                                    &commandsCallback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Failed to add commands subscription");
    return false;
  }
  setupStaticMessages();
  micro_ros_connected_ = true;
  return true;
}

void MicroROSInterface::setupStaticMessages(){
  strcpy(joint_name_storage_, "disc_joint");
  strcpy(frame_id_storage_, "base_link");
  joint_state_msg_.name.data = (rosidl_runtime_c__String*)malloc(sizeof(rosidl_runtime_c__String));
  joint_state_msg_.name.size = 1;
  joint_state_msg_.name.capacity = 1;
  joint_state_msg_.name.data[0].data = joint_name_storage_;
  joint_state_msg_.name.data[0].size = strlen(joint_name_storage_);
  joint_state_msg_.name.data[0].capacity = sizeof(joint_name_storage_);
  joint_state_msg_.position.data = joint_position_;
  joint_state_msg_.position.size = 1;
  joint_state_msg_.position.capacity = 1;
  joint_state_msg_.velocity.data = joint_velocity_;
  joint_state_msg_.velocity.size = 1;
  joint_state_msg_.velocity.capacity = 1;
  joint_state_msg_.effort.data = joint_effort_;
  joint_state_msg_.effort.size = 1;
  joint_state_msg_.effort.capacity = 1;
  joint_state_msg_.header.frame_id.data = frame_id_storage_;
  joint_state_msg_.header.frame_id.size = strlen(frame_id_storage_);
  joint_state_msg_.header.frame_id.capacity = sizeof(frame_id_storage_);
}

void MicroROSInterface::spin() {
  if (micro_ros_connected_) {
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
  }
}

void MicroROSInterface::publishJointState(double position_deg, double velocity_deg_per_sec) {
  if (!micro_ros_connected_) return;
  //convert to radians and normalize position
  double position_rad = position_deg * M_PI / 180.0;
  double velocity_rad_per_sec = velocity_deg_per_sec * M_PI / 180.0;
  //normalize position to -180 to +180
  while (position_rad > M_PI) position_rad -= 2.0 * M_PI;
  while (position_rad < -M_PI) position_rad += 2.0 * M_PI;
  joint_position_[0] = position_rad;
  joint_velocity_[0] = velocity_rad_per_sec;
  joint_effort_[0] = 0.0;
  unsigned long current_time = millis();
  joint_state_msg_.header.stamp.sec = current_time / 1000;
  joint_state_msg_.header.stamp.nanosec = (current_time % 1000) * 1000000;
  rcl_publish(&joint_state_pub_, &joint_state_msg_, NULL);
}
void MicroROSInterface::publishStatus(bool online) {
  (void)online;
}

//callbacks
void MicroROSInterface::targetAngleCallback(const void* msg) {
  if (micro_ros_instance) {
    micro_ros_instance->handleTargetAngle((const std_msgs__msg__Float32*)msg);
  }
}

void MicroROSInterface::trajectoryCallback(const void* msg) {
  if (micro_ros_instance) {
    micro_ros_instance->handleTrajectory((const trajectory_msgs__msg__JointTrajectory*)msg);
  }
}

void MicroROSInterface::commandsCallback(const void* msg) {
  if (micro_ros_instance) {
    micro_ros_instance->handleCommands((const std_msgs__msg__Float64MultiArray*)msg);
  }
}

void MicroROSInterface::handleTargetAngle(const std_msgs__msg__Float32* msg) {
  //direct angle command in degrees
  stepper.setTargetPosition(msg->data);
  Serial.printf("Target: %.2f°\n", msg->data);
}

//handle MoveIt trajectory commands
void MicroROSInterface::handleTrajectory(const trajectory_msgs__msg__JointTrajectory* msg) {
  if (msg->points.size > 0) {
    Serial.printf("Trajectory: %d points\n", (int)msg->points.size);
    if (msg->points.data[0].positions.size > 0) {
      double target_rad = msg->points.data[0].positions.data[0];
      double target_deg = target_rad * 180.0 / M_PI;
      stepper.setTargetPosition(target_deg);
      Serial.printf("Executing trajectory point: %.2f°\n", target_deg);
    }
    if (msg->points.size > 1) {
      int last_point = msg->points.size - 1;
      if (msg->points.data[last_point].positions.size > 0) {
        double final_rad = msg->points.data[last_point].positions.data[0];
        double final_deg = final_rad * 180.0 / M_PI;
        stepper.setTargetPosition(final_deg);
        Serial.printf("Final trajectory target: %.2f°\n", final_deg);
      }
    }
  }
}

void MicroROSInterface::handleCommands(const std_msgs__msg__Float64MultiArray* msg) {
  if (msg->data.size > 0) {
    double target_rad = msg->data.data[0];
    double target_deg = target_rad * 180.0 / M_PI;
    stepper.setTargetPosition(target_deg);
    Serial.printf("Position command: %.2f°\n", target_deg);
  }
}