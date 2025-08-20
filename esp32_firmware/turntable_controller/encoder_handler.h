#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class EncoderHandler {
private:
  double current_position_deg_;
  double previous_position_deg_;
  double velocity_deg_per_sec_;
  unsigned long last_read_time_;
  unsigned long last_velocity_calc_time_;
  bool encoder_connected_;
  
  uint16_t readRawAngle();
  double normalizeAngle(double angle);
  
public:
  EncoderHandler();
  void begin();
  void update();
  
  double getPositionDegrees() const { return current_position_deg_; }
  double getPositionRadians() const { return current_position_deg_ * M_PI / 180.0; }
  double getVelocityDegPerSec() const { return velocity_deg_per_sec_; }
  double getVelocityRadPerSec() const { return velocity_deg_per_sec_ * M_PI / 180.0; }
  bool isConnected() const { return encoder_connected_; }
};

extern EncoderHandler encoder;

#endif // ENCODER_HANDLER_H