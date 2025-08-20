#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include "config.h"

enum MotorState {
  STOPPED,
  MOVING_TO_TARGET,
  FOLLOWING_TRAJECTORY
};

class StepperControl {
private:
  volatile bool step_state_;
  volatile unsigned long step_interval_us_;
  volatile unsigned long last_step_time_;
  volatile bool direction_;  // true = CW, false = CCW
  
  MotorState current_state_;
  double target_position_deg_;
  double current_velocity_deg_per_sec_;
  double max_velocity_;
  double acceleration_;
  
  // Trajectory following
  bool executing_trajectory_;
  int trajectory_point_index_;
  unsigned long trajectory_start_time_;
  
  hw_timer_t* step_timer_;
  
  void setupMicrostepping();
  void setDirection(bool clockwise);
  void updateStepTiming();
  static void IRAM_ATTR stepTimerISR();
  
public:
  StepperControl();
  void begin();
  void update();
  void setTargetPosition(double position_deg);
  void setVelocity(double velocity_deg_per_sec);
  void executeTrajectory();
  void updatePositionControl(double current_position_deg);
  void stop();
  void enable(bool enabled);
  
  MotorState getState() const { return current_state_; }
  double getTargetPosition() const { return target_position_deg_; }
  double getCurrentVelocity() const { return current_velocity_deg_per_sec_; }
};

extern StepperControl stepper;

#endif // STEPPER_CONTROL_H