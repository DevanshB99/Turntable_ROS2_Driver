/*
  ESP32 Turntable Controller with micro-ROS over WiFi
  
  Hardware:
  - ESP32 Dev Module
  - A4988 Stepper Driver
  - NEMA 17 Stepper Motor
  - AS5600 Magnetic Encoder

*/

#include "config.h"
#include "encoder_handler.h"
#include "stepper_control.h"
#include "microros_interface.h"

EncoderHandler encoder;
StepperControl stepper;
MicroROSInterface micro_ros;
unsigned long last_publish_time = 0;
unsigned long last_encoder_update = 0;
unsigned long last_control_update = 0;

void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 Turntable Controller ===");
  Serial.println("Optimized for speed and precision");
  Serial.println();
  Serial.println("Initializing hardware...");
  //initialize encoder
  encoder.begin();
  delay(100);
  //initialize stepper motor
  stepper.begin();
  delay(100);
  // Set initial target to current position
  double initial_position = encoder.getPositionDegrees();
  stepper.setTargetPosition(initial_position);
  //initialize micro-ROS communication
  Serial.println("Connecting to micro-ROS...");
  micro_ros.begin();
  Serial.println();
  Serial.println("=== SYSTEM READY ===");
  Serial.printf("Initial position: %.2f°\n", initial_position);
  Serial.println("Waiting for commands on /target_angle...");
  Serial.println();
}

void loop() {
  unsigned long current_time = millis();
  //encoder reading @ 100Hz
  if (current_time - last_encoder_update >= ENCODER_READ_RATE_MS) {
    encoder.update();
    last_encoder_update = current_time;
  }
  //motor control @ 100Hz
  if (current_time - last_control_update >= (1000 / CONTROL_LOOP_RATE_HZ)) {
    double current_position = encoder.getPositionDegrees();
    stepper.updatePositionControl(current_position);
    stepper.update();
    last_control_update = current_time;
  }
  //Publish joint states @ 100Hz
  if (current_time - last_publish_time >= PUBLISH_RATE_MS) {
    if (micro_ros.isConnected()) {
      double position = encoder.getPositionDegrees();
      double velocity = encoder.getVelocityDegPerSec();
      micro_ros.publishJointState(position, velocity);
    }
    last_publish_time = current_time;
  }
  micro_ros.spin();
  delay(1);
}