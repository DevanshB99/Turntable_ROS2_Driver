#ifndef CONFIG_H
#define CONFIG_H
//A4988 Stepper Driver connections to ESP32 Dev Module
#define STEP_PIN 25
#define DIR_PIN 26
#define ENABLE_PIN 27
#define MS1_PIN 14
#define MS2_PIN 12
#define MS3_PIN 13
//AS5600 Magnetic Encoder connectins to ESP32 Dev Module
#define SDA_PIN 21
#define SCL_PIN 22
//motor configs
#define STEPS_PER_REV 200
#define MICROSTEP_MODE 16
#define TOTAL_STEPS_PER_REV (STEPS_PER_REV * MICROSTEP_MODE)  // 3200
#define BELT_RATIO 4
#define TOTAL_SYSTEM_STEPS (TOTAL_STEPS_PER_REV * BELT_RATIO)  // 12800 steps per revolution
//encoder I2C configs
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_REGISTER 0x0C
#define AS5600_STATUS_REGISTER 0x0B
#define AS5600_MD_BIT 0x20

#define STEP_DELAY_DEFAULT_US 1000  // 1ms default step delay
#define MIN_STEP_DELAY_US 300       // Minimum safe step delay
#define CONTROL_LOOP_RATE_HZ 100    // Main control loop
#define PUBLISH_RATE_MS 10          // 100Hz joint state publishing
#define PUBLISH_RATE_HZ 100         // 100Hz joint state publishing
#define ENCODER_READ_RATE_MS 10     // 100Hz encoder reading
#define POSITION_TOLERANCE_DEG 0.5   
#define MAX_VELOCITY_DEG_PER_SEC 180.0
#define ACCELERATION_DEG_PER_SEC2 90.0
#define PROPORTIONAL_GAIN 3.0
#define WIFI_SSID "ENTER WIFI SSID"
#define WIFI_PASSWORD "ENTER WIFI PASSWORD"
#define AGENT_IP "ENTER AGENT IP"
#define AGENT_PORT 8888

#define MICROROS_NODE_NAME "turntable_esp32"
#define MICROROS_NAMESPACE ""

#define TARGET_ANGLE_TOPIC "/target_angle"
#define JOINT_STATES_TOPIC "/turntables/joint_states"
#define TRAJECTORY_TOPIC "/turntable_trajectory_controller/joint_trajectory"
#define COMMANDS_TOPIC "/turntable_forward_position_controller/commands"
#define STATUS_TOPIC "/esp32/status"

#endif