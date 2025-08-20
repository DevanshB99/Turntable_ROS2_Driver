#include "stepper_control.h"

// Static instance for ISR access
static StepperControl* stepper_instance = nullptr;

StepperControl::StepperControl() 
  : step_state_(false)
  , step_interval_us_(STEP_DELAY_DEFAULT_US)
  , last_step_time_(0)
  , direction_(true)
  , current_state_(STOPPED)
  , target_position_deg_(0.0)
  , current_velocity_deg_per_sec_(0.0)
  , max_velocity_(MAX_VELOCITY_DEG_PER_SEC)
  , acceleration_(ACCELERATION_DEG_PER_SEC2)
  , executing_trajectory_(false)
  , trajectory_point_index_(0)
  , trajectory_start_time_(0)
  , step_timer_(nullptr)
{
  stepper_instance = this;
}

void StepperControl::begin() {
  // Configure GPIO pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  
  // Set initial states
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);  // Default direction
  digitalWrite(ENABLE_PIN, LOW); // Enable motor (LOW = enabled for A4988)
  
  // Setup microstepping
  setupMicrostepping();
  
  // Setup hardware timer for step generation
  step_timer_ = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1MHz)
  timerAttachInterrupt(step_timer_, &StepperControl::stepTimerISR, true);
  timerAlarmWrite(step_timer_, step_interval_us_, true);  // Repeat mode
  timerAlarmEnable(step_timer_);
  
  Serial.println("Stepper controller initialized");
}

void StepperControl::setupMicrostepping() {
  // Set 1/16 microstepping (MS1=HIGH, MS2=HIGH, MS3=HIGH)
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, HIGH);
  
  Serial.println("Microstepping set to 1/16");
}

void IRAM_ATTR StepperControl::stepTimerISR() {
  if (stepper_instance && stepper_instance->current_state_ != STOPPED) {
    digitalWrite(STEP_PIN, stepper_instance->step_state_);
    stepper_instance->step_state_ = !stepper_instance->step_state_;
  }
}

void StepperControl::update() {
  // This method is called from the real-time task
  // Update step timing based on current velocity
  updateStepTiming();
}

void StepperControl::updateStepTiming() {
  if (current_velocity_deg_per_sec_ == 0.0) {
    current_state_ = STOPPED;
    return;
  }
  
  // Calculate step interval based on velocity
  double steps_per_sec = (fabs(current_velocity_deg_per_sec_) * TOTAL_SYSTEM_STEPS) / 360.0;
  
  if (steps_per_sec > 0.001) {
    step_interval_us_ = max(static_cast<unsigned long>(1000000.0 / steps_per_sec), 
                           static_cast<unsigned long>(MIN_STEP_DELAY_US));
  } else {
    step_interval_us_ = STEP_DELAY_DEFAULT_US;
  }
  
  // Update timer
  timerAlarmWrite(step_timer_, step_interval_us_, true);
  
  // Set direction
  setDirection(current_velocity_deg_per_sec_ > 0);
}

void StepperControl::setDirection(bool clockwise) {
  direction_ = clockwise;
  // Note: Your proven direction logic from turntable.py
  // positive=CCW (DIR_PIN=LOW), negative=CW (DIR_PIN=HIGH)
  digitalWrite(DIR_PIN, clockwise ? LOW : HIGH);
}

void StepperControl::setTargetPosition(double position_deg) {
  target_position_deg_ = position_deg;
  current_state_ = MOVING_TO_TARGET;
  executing_trajectory_ = false;
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
  
  // Calculate error with wrap-around handling (matching turntable.py logic)
  double error = target_position_deg_ - current_position_deg;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  
  // Check if we've reached the target
  if (fabs(error) <= POSITION_TOLERANCE_DEG) {
    current_velocity_deg_per_sec_ = 0.0;
    current_state_ = STOPPED;
    return;
  }
  
  // Simple proportional control with velocity limiting
  double proportional_gain = 2.0;  // Adjust as needed
  double desired_velocity = error * proportional_gain;
  
  // Limit velocity
  desired_velocity = constrain(desired_velocity, -max_velocity_, max_velocity_);
  
  setVelocity(desired_velocity);
}

void StepperControl::executeTrajectory() {
  // Placeholder for trajectory execution
  // This would implement the trajectory following logic from turntable.py
  if (!executing_trajectory_) return;
  
  // Implementation would go here based on received trajectory points
}

void StepperControl::stop() {
  current_velocity_deg_per_sec_ = 0.0;
  current_state_ = STOPPED;
  executing_trajectory_ = false;
}

void StepperControl::enable(bool enabled) {
  digitalWrite(ENABLE_PIN, enabled ? LOW : HIGH);  // A4988: LOW = enabled
}