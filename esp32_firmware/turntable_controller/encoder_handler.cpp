#include "encoder_handler.h"

EncoderHandler::EncoderHandler()
  : current_position_deg_(0.0)
  , previous_position_deg_(0.0)
  , velocity_deg_per_sec_(0.0)
  , last_read_time_(0)
  , last_velocity_calc_time_(0)
  , encoder_connected_(false)
  , velocity_index_(0)
{
  for (int i = 0; i < VELOCITY_SAMPLES; i++) {
    velocity_history_[i] = 0.0;
  }
}

void EncoderHandler::begin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);
  //testing the encoder connection
  Wire.beginTransmission(AS5600_ADDRESS);
  encoder_connected_ = (Wire.endTransmission() == 0);
  if (encoder_connected_) {
    Serial.println("AS5600 encoder connected");
    //read the first position after boot
    current_position_deg_ = normalizeAngle(readRawAngle() * 360.0 / 4096.0);
    //applying inversion
    current_position_deg_ = normalizeAngle(360.0 - current_position_deg_);
    previous_position_deg_ = current_position_deg_;
    Serial.printf("Initial position: %.2f°\n", current_position_deg_);
    //check whether magnet is within the range
    if (!checkMagnetDetection()) {
      Serial.println("WARNING: Magnet not properly detected!");
    }
  } else {
    Serial.println("ERROR: AS5600 encoder not detected!");
  }
  last_read_time_ = millis();
  last_velocity_calc_time_ = millis();
}
void EncoderHandler::update() {
  if (!encoder_connected_) return; 
  unsigned long current_time = millis();
  //reading encoder
  if (current_time - last_read_time_ >= ENCODER_READ_RATE_MS) {
    uint16_t raw_angle = readRawAngle();
    double raw_angle_deg = raw_angle * 360.0 / 4096.0;
    //applying inversion
    double new_position = normalizeAngle(360.0 - raw_angle_deg);
    
    //calculate velocity for smoothing
    if (current_time - last_velocity_calc_time_ >= 50) {
      double delta_angle = new_position - previous_position_deg_;
      if (delta_angle > 180.0) delta_angle -= 360.0;
      else if (delta_angle < -180.0) delta_angle += 360.0;
      double delta_time = (current_time - last_velocity_calc_time_) / 1000.0;
      
      if (delta_time > 0) {
        //store velocity
        velocity_history_[velocity_index_] = delta_angle / delta_time;
        velocity_index_ = (velocity_index_ + 1) % VELOCITY_SAMPLES;
        //calculate smoothed velocity
        velocity_deg_per_sec_ = calculateSmoothedVelocity();
      }
      
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
  return 0;
}

double EncoderHandler::normalizeAngle(double angle) {
  //normalize to -180 to +180 range 
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

double EncoderHandler::calculateSmoothedVelocity() {
  double sum = 0.0;
  int count = 0;
  for (int i = 0; i < VELOCITY_SAMPLES; i++) {
    sum += velocity_history_[i];
    count++;
  }
  return count > 0 ? sum / count : 0.0;
}

bool EncoderHandler::checkMagnetDetection() {
  try {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(AS5600_STATUS_REGISTER);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDRESS, 1);
    if (Wire.available()) {
      uint8_t status = Wire.read();
      return (status & AS5600_MD_BIT) == AS5600_MD_BIT;
    }
  } catch (...) {
  }
  return false;
}

void EncoderHandler::resetVelocityHistory() {
  for (int i = 0; i < VELOCITY_SAMPLES; i++) {
    velocity_history_[i] = 0.0;
  }
  velocity_index_ = 0;
  velocity_deg_per_sec_ = 0.0;
}