#include "servo32u4.h"
extern Servo32U4 servo;

/// @brief savely moves the servo to a position
/// @param pos servo position to go to
/// @param reset resets internal counter variables. use between movements
/// @return grabber_move_state
Servo32U4Base::grabber_move_state servo_goto_nb(int pos, bool reset){  // TODO: check logic, not working rn
  int curr_pos = analogRead(servo.Servo_sense);
  int tar_pos = servo.microsec_to_pos(pos);  //converts position in microseconds to resistor value
  static int prev_pos = curr_pos;    //sets prev position a little different to avoid exiting while loop
  static int stall_counter = 0;
  if (reset) {
    prev_pos = curr_pos;
    stall_counter = 0;
  }

  Serial.print("curr_pos: ");
  Serial.println(curr_pos);
  Serial.print("tar_pos: ");
  Serial.println(tar_pos);
  Serial.print("prev_pos: ");
  Serial.println(prev_pos);

  servo.writeMicroseconds(pos);
  if(abs(curr_pos - prev_pos)>5 && abs(curr_pos - tar_pos)>10){  // servo still moving
    prev_pos = curr_pos;
    return (Servo32U4Base::grabber_move_state::in_progress);
  } else if (abs(curr_pos - prev_pos)<5){  // failure, servo not moving
    if (stall_counter++ > 20) {
      return (Servo32U4Base::grabber_move_state::failure);
    } else {
      return (Servo32U4Base::grabber_move_state::in_progress);
    }
  }else { //if (abs(curr_pos - tar_pos)<10) move done
    prev_pos = curr_pos;
    Serial.print("curr_pos: ");
    Serial.println(curr_pos);
    Serial.print("tar_pos: ");
    Serial.println(tar_pos);
    Serial.print("prev_pos: ");
    Serial.println(prev_pos);
    return (Servo32U4Base::grabber_move_state::success);
  }
  // delay(20);
}