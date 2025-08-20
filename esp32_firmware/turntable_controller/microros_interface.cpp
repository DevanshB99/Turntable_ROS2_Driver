#include "microros_interface.h"
#include "stepper_control.h"

// Static instance for callback access
static MicroROSInterface* micro_ros_instance = nullptr;

MicroROSInterface::MicroROSInterface()
  : micro_ros_connected_(false)
  , last_ping_time_(0)
{
  micro_ros_instance = this;
}

void MicroROSInterface::begin() {
  Serial.println("Initializing micro-ROS...");
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  
  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    
    // Set micro-ROS transport
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
    
    delay(2000);
    
    // Initialize micro-ROS
    if (connectToAgent()) {
      Serial.println("micro-ROS initialized successfully");
    } else {
      Serial.println("Failed to initialize micro-ROS");
    }
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}

bool MicroROSInterface::connectToAgent() {
  allocator_ = rcl_get_default_allocator();
  
  // Initialize support
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) {
    Serial.println("Failed to initialize support");
    return false;
  }
  
  // Create node
  if (rclc_node_init_default(&node_, MICROROS_NODE_NAME, MICROROS_NAMESPACE, &support_) != RCL_RET_OK) {
    Serial.println("Failed to create node");
    return false;
  }
  
  // Create publishers
  if (rclc_publisher_init_default(&joint_state_pub_, &node_, 
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                 "/turntables/joint_states") != RCL_RET_OK) {
    Serial.println("Failed to create joint state publisher");
    return false;
  }
  
  if (rclc_publisher_init_default(&status_pub_, &node_,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                 "/esp32/status") != RCL_RET_OK) {
    Serial.println("Failed to create status publisher");
    return false;
  }
  
  // Create subscribers
  if (rclc_subscription_init_default(&target_angle_sub_, &node_,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                    "/esp32/target_angle") != RCL_RET_OK) {
    Serial.println("Failed to create target angle subscription");
    return false;
  }
  
  if (rclc_subscription_init_default(&trajectory_sub_, &node_,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
                                    "/esp32/trajectory") != RCL_RET_OK) {
    Serial.println("Failed to create trajectory subscription");
    return false;
  }
  
  // Create timer
  if (rclc_timer_init_default(&timer_, &support_, RCL_MS_TO_NS(100), timerCallback) != RCL_RET_OK) {
    Serial.println("Failed to create timer");
    return false;
  }
  
  // Create executor
  if (rclc_executor_init(&executor_, &support_.context, 3, &allocator_) != RCL_RET_OK) {
    Serial.println("Failed to create executor");
    return false;
  }
  
  // Add subscriptions to executor
  if (rclc_executor_add_subscription(&executor_, &target_angle_sub_, &target_angle_msg_,
                                    &targetAngleCallback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Failed to add target angle subscription to executor");
    return false;
  }
  
  if (rclc_executor_add_subscription(&executor_, &trajectory_sub_, &trajectory_msg_,
                                    &trajectoryCallback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Failed to add trajectory subscription to executor");
    return false;
  }
  
  if (rclc_executor_add_timer(&executor_, &timer_) != RCL_RET_OK) {
    Serial.println("Failed to add timer to executor");
    return false;
  }
  
  setupStaticMessages();
  micro_ros_connected_ = true;
  return true;
}

void MicroROSInterface::setupStaticMessages() {
  // Setup joint state message
  strcpy(joint_name_storage_, "disc_joint");
  
  joint_state_msg_.name.data = (char**)malloc(sizeof(char*));
  joint_state_msg_.name.data[0] = joint_name_storage_;
  joint_state_msg_.name.size = 1;
  joint_state_msg_.name.capacity = 1;
  
  joint_state_msg_.position.data = joint_position_;
  joint_state_msg_.position.size = 1;
  joint_state_msg_.position.capacity = 1;
  
  joint_state_msg_.velocity.data = joint_velocity_;
  joint_state_msg_.velocity.size = 1;
  joint_state_msg_.velocity.capacity = 1;
  
  joint_state_msg_.effort.data = joint_effort_;
  joint_state_msg_.effort.size = 1;
  joint_state_msg_.effort.capacity = 1;
}

void MicroROSInterface::spin() {
  if (micro_ros_connected_) {
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));
  }
}

void MicroROSInterface::publishJointState(double position_deg, double velocity_deg_per_sec) {
  if (!micro_ros_connected_) return;
  
  // Convert to radians and set normalized range
  double position_rad = position_deg * M_PI / 180.0;
  double velocity_rad_per_sec = velocity_deg_per_sec * M_PI / 180.0;
  
  // Normalize position to -π to +π
  while (position_rad > M_PI) position_rad -= 2.0 * M_PI;
  while (position_rad < -M_PI) position_rad += 2.0 * M_PI;
  
  joint_position_[0] = position_rad;
  joint_velocity_[0] = velocity_rad_per_sec;
  joint_effort_[0] = 0.0;
  
  // Set timestamp
  joint_state_msg_.header.stamp.sec = millis() / 1000;
  joint_state_msg_.header.stamp.nanosec = (millis() % 1000) * 1000000;
  
  rcl_publish(&joint_state_pub_, &joint_state_msg_, NULL);
}

void MicroROSInterface::publishStatus(bool online) {
  if (!micro_ros_connected_) return;
  
  status_msg_.data = online;
  rcl_publish(&status_pub_, &status_msg_, NULL);
}

// Static callback wrappers
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

void MicroROSInterface::timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  // Timer callback for periodic tasks
  (void)timer;
  (void)last_call_time;
}

// Callback handlers
void MicroROSInterface::handleTargetAngle(const std_msgs__msg__Float32* msg) {
  Serial.printf("Received target angle: %.2f degrees\n", msg->data);
  stepper.setTargetPosition(msg->data);
}

void MicroROSInterface::handleTrajectory(const trajectory_msgs__msg__JointTrajectory* msg) {
  Serial.printf("Received trajectory with %d points\n", (int)msg->points.size);
  // Implementation for trajectory following would go here
}