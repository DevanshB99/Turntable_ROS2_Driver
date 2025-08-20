#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <Wire.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/joint_state.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <geometry_msgs/msg/twist.h>

#include "config.h"
#include "stepper_control.h"
#include "encoder_handler.h"
#include "microros_interface.h"

// Global objects
StepperControl stepper;
EncoderHandler encoder;
MicroROSInterface micro_ros;

// FreeRTOS task handles
TaskHandle_t stepperTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t microrosTaskHandle;

// Setup function
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("ESP32 Turntable Controller Starting...");
  
  // Initialize hardware
  stepper.begin();
  encoder.begin();
  
  // Initialize micro-ROS
  micro_ros.begin();
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    stepperTask,
    "StepperTask", 
    4096,
    NULL,
    3,  // High priority for real-time control
    &stepperTaskHandle,
    1   // Pin to core 1 for dedicated stepper control
  );
  
  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    4096,
    NULL,
    2,  // Medium priority
    &controlTaskHandle,
    0   // Pin to core 0 with micro-ROS
  );
  
  xTaskCreatePinnedToCore(
    microrosTask,
    "MicroROSTask",
    8192,
    NULL,
    1,  // Lower priority
    &microrosTaskHandle,
    0   // Pin to core 0
  );
  
  Serial.println("ESP32 Turntable Controller Initialized");
}

// Main loop - minimal to let FreeRTOS handle tasks
void loop() {
  delay(1000);
}

// Dedicated stepper control task (Core 1 - Real-time)
void stepperTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1);  // 1ms = 1000Hz
  
  for (;;) {
    stepper.update();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Control logic task (Core 0)
void controlTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_RATE_HZ);
  
  for (;;) {
    // Read encoder
    encoder.update();
    
    // Execute trajectory if active
    stepper.executeTrajectory();
    
    // Update position control
    stepper.updatePositionControl(encoder.getPositionDegrees());
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// micro-ROS communication task (Core 0)
void microrosTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / PUBLISH_RATE_HZ);
  
  for (;;) {
    // Spin micro-ROS
    micro_ros.spin();
    
    // Publish joint states
    micro_ros.publishJointState(
      encoder.getPositionDegrees(),
      encoder.getVelocityDegPerSec()
    );
    
    // Publish status
    micro_ros.publishStatus(true);
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}