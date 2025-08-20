#ifndef CONFIG_H
#define CONFIG_H

// Hardware pin definitions (matching your current setup)
#define STEP_PIN 25
#define DIR_PIN 26
#define ENABLE_PIN 27
#define MS1_PIN 14
#define MS2_PIN 12
#define MS3_PIN 13

// I2C pins for AS5600 encoder
#define SDA_PIN 21
#define SCL_PIN 22

// Motor configuration (matching your turntable.py)
#define STEPS_PER_REV 200
#define MICROSTEP_MODE 16
#define TOTAL_STEPS_PER_REV (STEPS_PER_REV * MICROSTEP_MODE)  // 3200
#define BELT_RATIO 4
#define TOTAL_SYSTEM_STEPS (TOTAL_STEPS_PER_REV * BELT_RATIO)  // 12800 steps per revolution

// AS5600 I2C configuration
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_REGISTER 0x0C
#define AS5600_STATUS_REGISTER 0x0B
#define AS5600_MD_BIT 0x20

// Timing configuration
#define STEP_DELAY_DEFAULT_US 1000  // 1ms default step delay
#define MIN_STEP_DELAY_US 300       // Minimum safe step delay
#define CONTROL_LOOP_RATE_HZ 100    // Main control loop frequency
#define PUBLISH_RATE_HZ 50          // Joint state publishing rate

// Movement parameters
#define POSITION_TOLERANCE_DEG 1.0   // Position tolerance in degrees
#define MAX_VELOCITY_DEG_PER_SEC 180.0  // Maximum velocity
#define ACCELERATION_DEG_PER_SEC2 90.0  // Acceleration limit

// WiFi and micro-ROS configuration
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define AGENT_IP "192.168.1.100"  // Your ROS2 host computer IP
#define AGENT_PORT 8888

// micro-ROS configuration
#define MICROROS_NODE_NAME "turntable_esp32"
#define MICROROS_NAMESPACE ""

#endif // CONFIG_H