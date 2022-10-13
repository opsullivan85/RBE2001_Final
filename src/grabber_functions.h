#include "servo32u4.h"
extern Servo32U4 servo;

/// @brief savely moves the servo to a position
/// @param pos servo position to go to
/// @param reset resets internal counter variables. use between movements
/// @return grabber_move_state
Servo32U4Base::grabber_move_state servo_goto_nb(int pos, bool reset){  // TODO: check logic, not working rn
  const static int stall_tolerance = 2;
  const static int position_tolerance = 10;
  int curr_pos = analogRead(servo.Servo_sense);
  int tar_pos = servo.microsec_to_pos(pos);  //converts position in microseconds to resistor value
  static int prev_pos = curr_pos + stall_tolerance + 1;    //sets prev position a little different to avoid exiting while loop
  if (reset) {
    prev_pos = curr_pos + stall_tolerance + 1;
  }

  servo.writeMicroseconds(pos);
  
  if(abs(curr_pos - prev_pos)>stall_tolerance && abs(curr_pos - tar_pos)>position_tolerance){  // servo still moving
    prev_pos = curr_pos;
    return (Servo32U4Base::grabber_move_state::in_progress);
  } else if (abs(curr_pos - prev_pos)<stall_tolerance && abs(curr_pos - tar_pos)>position_tolerance){  // failure, servo not moving
    return (Servo32U4Base::grabber_move_state::failure);
  }else { //if (abs(curr_pos - tar_pos)<10) move done
    return (Servo32U4Base::grabber_move_state::success);
  }
}