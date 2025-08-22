#include "stepper_control.h"
static StepperControl* stepper_instance = nullptr;

StepperControl::StepperControl() 
  : step_state_(false)
  , step_interval_us_(STEP_DELAY_DEFAULT_US)
  , direction_(true)
  , current_state_(STOPPED)
  , target_position_deg_(0.0)
  , current_velocity_deg_per_sec_(0.0)
  , max_velocity_(MAX_VELOCITY_DEG_PER_SEC)
  , acceleration_(ACCELERATION_DEG_PER_SEC2)
  , step_timer_(nullptr)
{
  stepper_instance = this;
}

void StepperControl::begin() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(ENABLE_PIN, LOW); //enable motor (LOW = enabled for A4988)
  setupMicrostepping();
  step_timer_ = timerBegin(1000000);
  timerAttachInterrupt(step_timer_, &StepperControl::stepTimerISR);
  Serial.println("Stepper controller initialized");
}

void StepperControl::setupMicrostepping() {
  //setting 1/16 microstepping
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, HIGH);
  Serial.println("Microstepping: 1/16");
}

void IRAM_ATTR StepperControl::stepTimerISR() {
  if (stepper_instance && stepper_instance->current_state_ != STOPPED) {
    digitalWrite(STEP_PIN, stepper_instance->step_state_);
    stepper_instance->step_state_ = !stepper_instance->step_state_;
  }
}

void StepperControl::update() {
  updateStepTiming();
}

void StepperControl::updateStepTiming() {
  if (current_velocity_deg_per_sec_ == 0.0) {
    current_state_ = STOPPED;
    if (step_timer_) {
      timerStop(step_timer_);
    }
    return;
  }
  double steps_per_sec = (fabs(current_velocity_deg_per_sec_) * TOTAL_STEPS_PER_REV * BELT_RATIO) / 360.0;
  if (steps_per_sec > 0.001) {
    step_interval_us_ = max(static_cast<unsigned long>(1000000.0 / steps_per_sec), 
                           static_cast<unsigned long>(MIN_STEP_DELAY_US));
  } else {
    step_interval_us_ = STEP_DELAY_DEFAULT_US;
  }
  if (step_timer_) {
    timerStop(step_timer_);
    timerAlarm(step_timer_, step_interval_us_, true, 0);
    timerStart(step_timer_);
  }
  setDirection(current_velocity_deg_per_sec_ > 0);
}

void StepperControl::setDirection(bool clockwise) {
  direction_ = clockwise;
  //positive=CCW (DIR_PIN=LOW), negative=CW (DIR_PIN=HIGH)
  digitalWrite(DIR_PIN, clockwise ? LOW : HIGH);
}

double StepperControl::calculateShortestPath(double current, double target) {
  double error = target - current;
  if (error > 180.0) error -= 360.0;
  else if (error < -180.0) error += 360.0;
  
  return error;
}

void StepperControl::setTargetPosition(double position_deg) {
  target_position_deg_ = position_deg;
  current_state_ = MOVING_TO_TARGET;
}

void StepperControl::setVelocity(double velocity_deg_per_sec) {
  current_velocity_deg_per_sec_ = constrain(velocity_deg_per_sec, 
                                           -max_velocity_, 
                                           max_velocity_);
  if (fabs(current_velocity_deg_per_sec_) > 0.1) {
    current_state_ = MOVING_TO_TARGET;
  }
}

void StepperControl::updatePositionControl(double current_position_deg) {
  if (current_state_ != MOVING_TO_TARGET) return;
  double error = calculateShortestPath(current_position_deg, target_position_deg_);
  if (fabs(error) <= POSITION_TOLERANCE_DEG) {
    current_velocity_deg_per_sec_ = 0.0;
    current_state_ = STOPPED;
    return;
  }
  double desired_velocity = error * PROPORTIONAL_GAIN;
  desired_velocity = constrain(desired_velocity, -max_velocity_, max_velocity_);
  setVelocity(desired_velocity);
}

void StepperControl::stop() {
  current_velocity_deg_per_sec_ = 0.0;
  current_state_ = STOPPED;
  if (step_timer_) {
    timerStop(step_timer_);
  }
}
void StepperControl::enable(bool enabled) {
  digitalWrite(ENABLE_PIN, enabled ? LOW : HIGH);  // A4988: LOW = enabled
}