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
unsigned long last_status_publish_time = 0;
bool previous_moving_state = false;

bool isTurntableMoving(double current_position, double target_position, double current_velocity) {
  // Calculate position error using shortest path (accounting for wraparound)
  double error = target_position - current_position;
  if (error > 180.0) error -= 360.0;
  else if (error < -180.0) error += 360.0;
  
  // Turntable is considered moving if:
  // 1. Position error is greater than tolerance, OR
  // 2. Velocity is above a minimum threshold
  bool position_error_exists = fabs(error) > POSITION_TOLERANCE_DEG;
  bool velocity_above_threshold = fabs(current_velocity) > 0.5; // deg/sec threshold
  
  return position_error_exists || velocity_above_threshold;
}

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
  
<<<<<<< HEAD
  // DOMAIN ID CONFIGURATION:
  // Change this number to your desired domain ID before connecting
  micro_ros.setDomainId(11);  // Set to match your agent domain ID
=======
  // Optional: Set a different domain ID before connecting
  // micro_ros.setDomainId(1);  // Uncomment and change number to set domain ID
>>>>>>> 766fb78 (Integrated feature - ROS DOMAIN ID - set it in microros-interface.cpp file.)
  
  //initialize micro-ROS communication
  Serial.println("Connecting to micro-ROS...");
  micro_ros.begin();
  
  Serial.println();
  Serial.println("=== SYSTEM READY ===");
  Serial.printf("Initial position: %.2f°\n", initial_position);
  Serial.printf("ROS Domain ID: %zu\n", micro_ros.getDomainId());
  Serial.println("Waiting for commands on /target_angle...");
  Serial.println();
}

void loop() {
  unsigned long current_time = millis();
<<<<<<< HEAD
  
  //encoder reading @ 100Hz
=======

  static unsigned long last_debug = 0;
  if (current_time - last_debug > 5000) {
    Serial.println("[DEBUG] Main loop running...");
    Serial.printf("[DEBUG] last_status_publish_time = %lu, current_time = %lu\n", 
                  last_status_publish_time, current_time);
    last_debug = current_time;
  }

  //encoder reading @ 125Hz
>>>>>>> 766fb78 (Integrated feature - ROS DOMAIN ID - set it in microros-interface.cpp file.)
  if (current_time - last_encoder_update >= ENCODER_READ_RATE_MS) {
    encoder.update();
    last_encoder_update = current_time;
  }
  
<<<<<<< HEAD
  //motor control @ 100Hz
=======
  //motor control @ 125Hz
>>>>>>> 766fb78 (Integrated feature - ROS DOMAIN ID - set it in microros-interface.cpp file.)
  if (current_time - last_control_update >= (1000 / CONTROL_LOOP_RATE_HZ)) {
    double current_position = encoder.getPositionDegrees();
    stepper.updatePositionControl(current_position);
    stepper.update();
    last_control_update = current_time;
  }
  
<<<<<<< HEAD
  //Publish joint states @ 100Hz
=======
  //Publish joint states @ 125Hz
>>>>>>> 766fb78 (Integrated feature - ROS DOMAIN ID - set it in microros-interface.cpp file.)
  if (current_time - last_publish_time >= PUBLISH_RATE_MS) {
    if (micro_ros.isConnected()) {
      double position = encoder.getPositionDegrees();
      double velocity = encoder.getVelocityDegPerSec();
      micro_ros.publishJointState(position, velocity);
    }
    last_publish_time = current_time;
  }
<<<<<<< HEAD
=======

  //Publish turntable status @ 125Hz
  if (current_time - last_status_publish_time >= STATUS_PUBLISH_RATE_MS) {
    Serial.println("[DEBUG] Status publish time reached");
    if (micro_ros.isConnected()) {
      Serial.println("[DEBUG] micro-ROS connected, publishing status");
      double current_position = encoder.getPositionDegrees();
      double target_position = stepper.getTargetPosition();
      double current_velocity = encoder.getVelocityDegPerSec();
      
      bool is_moving = isTurntableMoving(current_position, target_position, current_velocity);
      micro_ros.publishTurntableStatus(is_moving);
      Serial.printf("[DEBUG] Status published: %s (pos: %.2f°, target: %.2f°, vel: %.2f°/s)\n", 
                    is_moving ? "MOVING" : "STOPPED", 
                    current_position, target_position, current_velocity);
      
      previous_moving_state = is_moving;
    } 
    else {
      Serial.println("[DEBUG] micro-ROS not connected - cannot publish status");
    }
    last_status_publish_time = current_time;
  }
>>>>>>> 766fb78 (Integrated feature - ROS DOMAIN ID - set it in microros-interface.cpp file.)
  
  micro_ros.spin();
}