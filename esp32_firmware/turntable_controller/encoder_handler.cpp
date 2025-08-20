#include "encoder_handler.h"

EncoderHandler::EncoderHandler()
  : current_position_deg_(0.0)
  , previous_position_deg_(0.0)
  , velocity_deg_per_sec_(0.0)
  , last_read_time_(0)
  , last_velocity_calc_time_(0)
  , encoder_connected_(false)
{
}

void EncoderHandler::begin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz I2C speed
  
  delay(100);
  
  // Test encoder connection
  Wire.beginTransmission(AS5600_ADDRESS);
  encoder_connected_ = (Wire.endTransmission() == 0);
  
  if (encoder_connected_) {
    Serial.println("AS5600 encoder connected successfully");
    
    // Read initial position
    current_position_deg_ = normalizeAngle(readRawAngle() * 360.0 / 4096.0);
    previous_position_deg_ = current_position_deg_;
    
    // Apply inversion as per turntable.py
    current_position_deg_ = normalizeAngle(360.0 - current_position_deg_);
    previous_position_deg_ = current_position_deg_;
    
    Serial.printf("Initial encoder position: %.2f degrees\n", current_position_deg_);
  } else {
    Serial.println("AS5600 encoder not detected!");
  }
  
  last_read_time_ = millis();
  last_velocity_calc_time_ = millis();
}

void EncoderHandler::update() {
  if (!encoder_connected_) return;
  
  unsigned long current_time = millis();
  
  // Read encoder at high frequency for accuracy
  if (current_time - last_read_time_ >= 10) {  // 100Hz reading
    uint16_t raw_angle = readRawAngle();
    double raw_angle_deg = raw_angle * 360.0 / 4096.0;
    
    // Apply inversion (matching turntable.py logic)
    double new_position = normalizeAngle(360.0 - raw_angle_deg);
    
    // Calculate velocity every 50ms for smoothing
    if (current_time - last_velocity_calc_time_ >= 50) {
      double delta_angle = new_position - previous_position_deg_;
      
      // Handle wrap-around
      if (delta_angle > 180.0) delta_angle -= 360.0;
      else if (delta_angle < -180.0) delta_angle += 360.0;
      
      double delta_time = (current_time - last_velocity_calc_time_) / 1000.0;
      velocity_deg_per_sec_ = delta_angle / delta_time;
      
      previous_position_deg_ = current_position_deg_;
      last_velocity_calc_time_ = current_time;
    }
    
    current_position_deg_ = new_position;
    last_read_time_ = current_time;
  }
}

uint16_t EncoderHandler::readRawAngle() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_REGISTER);
  Wire.endTransmission(false);
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() == 2) {
    uint16_t angle = Wire.read() << 8;
    angle |= Wire.read();
    return angle;
  }
  
  return 0;  // Error case
}

double EncoderHandler::normalizeAngle(double angle) {
  // Normalize to -180 to +180 range (matching turntable.py)
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}