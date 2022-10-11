#define ROMI_RADIUS 8  // cm

/// @brief converts linear wheel distance to radians
/// @param wheel_dst distance the wheel has moved
/// @return radians
float wheel_dst_to_rad(float wheel_dst){
  return (wheel_dst / ROMI_RADIUS);
}

/// @brief converts radians to wheel linear distance
/// @param wheel_dst distance the wheel has moved
/// @return wheel linear distance
float rad_to_wheel_dst(float rad){
  return rad * ROMI_RADIUS / chassis.cmPerEncoderTick;
}

/// @brief turns some ammount of radians clockwise
/// @param base_effort base effort to use for movement
/// @param p p val
/// @param angle_rad angle to turn in rad 
/// @param tolerance tolerance to hold the turn to
/// @param reset resets internal counter variables. use between movements
/// @return if the movement is completed
bool turn_rad_nb(int base_effort, float p, float angle_rad, float tolerance, bool reset){  // TODO: fix logic
  static float l_enc_zero = chassis.getLeftEncoderCount();
  static float r_enc_zero = chassis.getRightEncoderCount();

  if (reset) {
    l_enc_zero = chassis.getLeftEncoderCount();
    r_enc_zero = chassis.getRightEncoderCount();
  }
  
  float target_pos = rad_to_wheel_dst(angle_rad);
  float l_pos = chassis.getLeftEncoderCount() - l_enc_zero;
  float r_pos = chassis.getRightEncoderCount() - r_enc_zero;
  float l_delta = target_pos - abs(l_pos);
  float r_delta = target_pos - abs(r_pos);
  float avg_delta = (abs(l_delta) + abs(r_delta)) / 2;
  float l_effort = l_delta*p;
  float r_effort = r_delta*p;

  l_effort = constrain(l_effort, -base_effort, base_effort);
  r_effort = constrain(r_effort, -base_effort, base_effort);

  Serial.println();
  Serial.print("target_pos ");
  Serial.println(target_pos);
  Serial.print("l_pos ");
  Serial.println(l_pos);
  Serial.print("r_pos ");
  Serial.println(r_pos);
  Serial.print("l_delta ");
  Serial.println(l_delta);
  Serial.print("r_delta ");
  Serial.println(r_delta);
  Serial.print("l_effort ");
  Serial.println(l_effort);
  Serial.print("r_effort ");
  Serial.println(r_effort);
  Serial.print("avg_delta ");
  Serial.println(avg_delta);
  Serial.println();
  delay(20);

  if (avg_delta <= tolerance){
    chassis.setMotorEfforts(0,0);
    return true;
  }

  if (angle_rad <= PI){  // turn clockwise
    chassis.setMotorEfforts(l_effort, -r_effort);
  } else {  // turn counterclockwise
    chassis.setMotorEfforts(-l_effort, r_effort);
  }

  return false;
}