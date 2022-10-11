#pragma once

#define REFL_L 22 
#define REFL_R 20
#define TAPE_THRESHOLD 400  // vals over are considered on tape
#define WHITE_THRESHOLD 40  // vals under are considered on white
#define MIN_MOTOR_EFFORT 80

/// @brief 
/// @param val reflectance sensor value
/// @return if the sensor is on the tape
bool on_tape(int val){
  return val >= TAPE_THRESHOLD;
}

/// @brief 
/// @param val reflectance sensor value
/// @return if the sensor is on the white
bool on_white(int val){
  return val < WHITE_THRESHOLD;
}


/// @brief 
/// @return if the sensor is at an intersection 
bool at_intersection(){
  int l = analogRead(REFL_L);
  int r = analogRead(REFL_R);
  return on_tape(l) && on_tape(r);
}


/// @brief gets distance sensor reading
/// @return distance reading in cm
float read_dst_sensor(){
  return rangefinder.getDistance();
}


/// @brief provides porportional control for motors to reach distance sensor reading
///        values need to be manually assigned to motors, should be called in a loop
///        until distance reading is within tolerance range.
/// @param base_effort max speed to run the motors at
/// @param p p value
/// @param distance target distance reading cm
/// @return adjusted motor effort based on current state, may be negative, but magnitude <= base_effort
int dst_sensor_p_control(int base_effort, float p, float current_distance, float distance){
  int new_effort = (float)base_effort * constrain((current_distance - distance) * p, -1.0, 1.0);
  if (new_effort > 0 && new_effort < MIN_MOTOR_EFFORT) {
    new_effort = MIN_MOTOR_EFFORT;
  } else if (new_effort < 0 && new_effort > -MIN_MOTOR_EFFORT){
    new_effort = -MIN_MOTOR_EFFORT;
  }
  return new_effort;
}


/// @brief finds steering correction value to stay on tape line
/// @param p porportional constant
/// @return float (0-1) steering power ratio for correction
///         0 being full left, 1 full right
float get_line_following_steering_factor(float p){
  int l = analogRead(REFL_L);
  int r = analogRead(REFL_R);
  // both or off tape
  if ((on_tape(l) && on_tape(r)) || (on_white(l) && on_white(r))){
    return 0.5;  // go straight

  } else {
    // too far pointed right
    if (!on_white(l)) {
      int dist = l - WHITE_THRESHOLD;
      return 0.5 + dist * p;

    // too far pointed left
    } else if (!on_white(r)) {
      int dist = r - WHITE_THRESHOLD;
      return 0.5 - dist * p;
    }
  }
  return 0.5;  // shouldnt get here
}


/// @brief gets steering corrected motor speed for left motor
///        takes account for base_effort being negative
/// @param base_effort base speed for the motor
/// @param steering_factor steeringfactor. see get_line_following_steering_factor
/// @return motor effort with steering factor applied
int get_left_steering_effort(int base_effort, float steering_factor){
  if (base_effort >= 0){
    return base_effort * (1-steering_factor);
  } else{
    return base_effort * steering_factor;
  }
}


/// @brief gets steering corrected motor speed for right motor
/// @param base_effort base speed for the motor
/// @param steering_factor steering_factor factor. see get_line_following_steering_factor
/// @return motor effort with steering factor applied
int get_right_steering_effort(int base_effort, float steering_factor){
  if (base_effort >= 0){
    return base_effort * steering_factor;
  } else {
    return base_effort * (1-steering_factor);
  }
}


/// @brief follows a line to the next intersection
/// @param base_effort speed to run the motors at
/// @param p p factor
/// @return if the movement is completed
bool follow_line_to_intersection_nb(int base_effort, float p){
  float steering_factor;

  if (!at_intersection()){
    steering_factor = get_line_following_steering_factor(p);
    // Serial.print("steering factor: ");
    // Serial.println(steering_factor);
    // Serial.print("left steering effort: ");
    // Serial.println(get_left_steering_effort(base_effort, steering_factor));
    // Serial.print("right steering effort: ");
    // Serial.println(get_right_steering_effort(base_effort, steering_factor));
    chassis.setMotorEfforts(get_left_steering_effort(base_effort, steering_factor),
      get_right_steering_effort(base_effort, steering_factor));
    return (false);
  }
  return (true);
}


/// @brief follows a line to a distance sensor reading
///        uses PID control from line sensor to control steering
///        uses PID control from distance sensor to control overall speed
/// @param base_effort speed to run the motors at
/// @param p p factor
/// @param distance distance in cm
/// @param tolerance distance tolerance
/// @return if the movement is completed
bool follow_line_to_distance_reading_nb(int base_effort, float lf_p, float dst_p, float distance, float tolerance){
  float steering_factor;
  int effort;
  float current_distance = read_dst_sensor();

  // effort controls the speed of the robot
  // steering factor controls the direction of the robot
  // by controling steering factor with the line following code
  // and effort with the distance sensor PID code
  // the robot is able to line follow to a specific distance sensor reading
  if (abs(current_distance - distance) >= tolerance){
    effort = dst_sensor_p_control(base_effort, dst_p, current_distance, distance);
    steering_factor = get_line_following_steering_factor(lf_p);
    // Serial.println(effort);
    // Serial.println(steering_factor);
    // Serial.println();
    // Serial.println();
    chassis.setMotorEfforts(get_left_steering_effort(effort, steering_factor),
      get_right_steering_effort(effort, steering_factor));
    current_distance = read_dst_sensor();
    return (false);
  } else {
    return (true);
  }
}


/// @brief follows the line a specific distance
/// @param base_effort speed to run the motors at
/// @param p p factor
/// @param distance distance to go (cm)
/// @param reset resets internal counter variables. use between movements
/// @return if the movement is completed
bool follow_line_distance_nb(int base_effort, float p, float distance, bool reset){
  static float traveled_distance = 0;
  static int l_start_enc_cnt = chassis.getLeftEncoderCount();
  static int r_start_enc_cnt = chassis.getLeftEncoderCount();
  if (reset) {
    traveled_distance = 0;
    l_start_enc_cnt = chassis.getLeftEncoderCount();
    r_start_enc_cnt = chassis.getLeftEncoderCount();
  }
  float correction;
  int l_enc_delta;
  int r_enc_delta;

  if (traveled_distance <= distance){
    correction = get_line_following_steering_factor(p);
    chassis.setMotorEfforts(get_left_steering_effort(base_effort, correction),
      get_right_steering_effort(base_effort, correction));

    l_enc_delta = chassis.getLeftEncoderCount() - l_start_enc_cnt;
    r_enc_delta = chassis.getLeftEncoderCount() - r_start_enc_cnt;

    traveled_distance = (l_enc_delta + r_enc_delta) / 2 * chassis.cmPerEncoderTick;
    return false;
  } else {
    return true;
  }
}