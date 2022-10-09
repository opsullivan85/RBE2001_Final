#include <Arduino.h>
#include <Romi32U4.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include "StackArray.h"
#include "BlueMotor.h"
#include "servo32u4.h"

#define REFL_L 22
#define REFL_R 20
#define TAPE_THRESHOLD 400  // vals over are considered on tape
#define WHITE_THRESHOLD 40  // vals under are considered on white
#define ROMI_RADIUS 8  // cm
#define LF_P 0.0005
#define DST_P 0.17
#define MIN_MOTOR_EFFORT 80
#define pos_0_enc_cnt 0
#define pos_0_distance 5.0
#define pos_45_enc_cnt 3892
#define pos_45_distance 5.0
#define pos_25_enc_cnt 8234
#define pos_25_distance 5.0

BlueMotor motor;
Rangefinder rangefinder(17,12);
Servo32U4 servo;
Chassis chassis;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

void setup()
{
  Serial.begin(9600);
  chassis.init();
  rangefinder.init();
  pinMode(REFL_L, INPUT);
  motor.setup();
  servo.setMinMaxMicroseconds(500, 2500);   //1035 closed, 1900 open
  pinMode(REFL_R, INPUT);
  buttonB.waitForButton();
  delay(500);
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
  if (reset) {traveled_distance = 0;}
  float correction;
  int l_start_enc_cnt = chassis.getLeftEncoderCount();
  int r_start_enc_cnt = chassis.getLeftEncoderCount();
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
  return rad * ROMI_RADIUS;
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
  float l_delta = target_pos - l_pos;
  float r_delta = target_pos - r_pos;
  float avg_delta = (l_delta + r_delta) / 2;
  float l_effort = constrain(l_delta*p, 0, base_effort);
  float r_effort = constrain(r_delta*p, 0, base_effort);

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


/// @brief savely moves the servo to a position
/// @param pos servo position to go to
/// @param reset resets internal counter variables. use between movements
/// @return grabber_move_state
Servo32U4Base::grabber_move_state servo_goto_nb(int pos, bool reset){
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
    if (stall_counter++ > 10) {
      return (Servo32U4Base::grabber_move_state::failure);
    } else {
      return (Servo32U4Base::grabber_move_state::in_progress);
    }
  }else { //if (abs(curr_pos - tar_pos)<10) move done
    prev_pos = curr_pos;
    return (Servo32U4Base::grabber_move_state::success);
  }
  // delay(20);
}


enum states {
  initilize,
  estop,
  next_state,
  turn_to_line,  // expects data for turn direction (0 left, 1 right)
  turn_rad,  // expects an angle to turn counterclockwise (deg 0-359)
  orient_to_intersection,  // turns and goes to intersection. 
                           // expects data for direction to turn at intersection (0 left, 1 right)
  follow_line_to_distance_reading,  // expects data for distance reading (cm)
  follow_line_to_over_intersection,
  follow_line_distance,  // expects data for distance to move (cm)
  wait_for_confirmation,
  wait,  // expects data for time to wait, (~ms)
  grab,
  release,
  grab_pos,  // expects data for four bar position (deg)
  release_pos,  // expects data for four bar position (deg)
  pos_four_bar,  // expects data for four bar position (deg)
};

struct packet {
  states state;
  float data;
};

void debug_printer(packet instruction, int counter){
  static const String decoder[] = {
    "initilize",
    "estop",
    "next_state",
    "turn_to_line",  // expects data for turn direction (0 left, 1 right)
    "turn_rad",  // expects an angle to turn counterclockwise (deg 0-359)
    "orient_to_intersection",  // turns and goes to intersection. 
                            // expects data for direction to turn at intersection (0 left, 1 right)
    "follow_line_to_distance_reading",  // expects data for distance reading (cm)
    "follow_line_to_over_intersection",
    "follow_line_distance",  // expects data for distance to move (cm)
    "wait_for_confirmation",
    "wait",  // expects data for time to wait, (~ms)
    "grab",
    "release",
    "grab_pos",  // expects data for four bar position (deg)
    "release_pos",  // expects data for four bar position (deg)
    "pos_four_bar",  // expects data for four bar position (deg)
  };
  Serial.println();
  Serial.print(decoder[instruction.state]);
  Serial.print(" ");
  Serial.print(instruction.data);
  Serial.print(" ");
  Serial.println(counter);
}

void loop(){
  #define debug
  StackArray<packet> instruction_stack;
  // instruction_stack.push((packet){initilize, -1});
  instruction_stack.push((packet){grab, -1});
  packet instruction = (packet){next_state, -1};
  int counter = 0;  // resets between states

  while (true){
    #ifdef debug
    debug_printer(instruction, counter);
    #endif

    if(buttonB.isPressed()){
      instruction = (packet){estop, -1};
    }

    switch (instruction.state) {
      case initilize:
        // this is a stack so instructions should be read bottom up
        instruction_stack.push((packet){release_pos, 25});  // replace plate for 25 pos
        instruction_stack.push((packet){orient_to_intersection, 0}); // orient for 25 pos

        instruction_stack.push((packet){grab_pos, 0});  // grab new plate from 0 deg pos
        instruction_stack.push((packet){release_pos, 0});  // release old plate at 0 deg pos

        instruction_stack.push((packet){orient_to_intersection, 0});  // orient for 0 pos
        instruction_stack.push((packet){grab_pos, 25});  // grab from 45 deg pos
        instruction_stack.push((packet){orient_to_intersection, 1});  // orient for 45 pos

        instruction_stack.push((packet){follow_line_to_over_intersection, -1});  // cross field
        instruction_stack.push((packet){turn_rad, 270});  // turn to face other side of field
        instruction_stack.push((packet){follow_line_to_distance_reading, 10});  // go to position for crossing
        instruction_stack.push((packet){orient_to_intersection, 1});  // orient for 0 pos

        instruction_stack.push((packet){release_pos, 45});  // replace plate for 45 pos
        instruction_stack.push((packet){orient_to_intersection, 0}); // orient for 45 pos

        instruction_stack.push((packet){grab_pos, 0});  // grab new plate from 0 deg pos
        instruction_stack.push((packet){release_pos, 0});  // release old plate at 0 deg pos

        instruction_stack.push((packet){orient_to_intersection, 1});  // orient for 0 pos
        instruction_stack.push((packet){grab_pos, 45});  // grab from 45 deg pos

        instruction = (packet){next_state, -1};
        break;

      case orient_to_intersection:
        instruction_stack.push((packet){turn_to_line, instruction.data});  // turn specified direction
        instruction_stack.push((packet){follow_line_to_over_intersection, -1});  // go to intersection
        instruction_stack.push((packet){turn_to_line, 1});  // full turn on single line
        instruction = (packet){next_state, -1};
        break;

      case estop:  // TODO: Finish
        motor.setEffort(0);
        chassis.idle();

        while (true) {}
        break;

      case next_state:
        counter = 0;  // reset the counter so the next state can use it
        if (!instruction_stack.isEmpty()) {
          instruction = instruction_stack.pop();
        } else {
          instruction = (packet){estop, -1};
        }
        break;

      case turn_to_line:  // TODO: Finish
        break;
      case turn_rad:  // TODO: Finish
        break;

      case follow_line_to_distance_reading:
        if (follow_line_to_distance_reading_nb(300, LF_P, DST_P, 120, 0.1)){
          instruction = (packet){next_state, -1};
        }
        break;

      case follow_line_to_over_intersection:
        if (follow_line_to_intersection_nb(300, LF_P)){
          instruction_stack.push((packet){follow_line_distance, ROMI_RADIUS});
          instruction = (packet){next_state, -1};
        }
        break;

      case follow_line_distance:
        if (counter++ == 0) {  // first nb call of this move
          if (follow_line_distance_nb(300, LF_P, instruction.data, true)){  // reset fn
            instruction = (packet){next_state, -1};
          }
        } else {  // subsequent calls
          if (follow_line_distance_nb(300, LF_P, instruction.data, false)){  // dont reset fn
            instruction = (packet){next_state, -1};
          }
        }

        break;

      case wait_for_confirmation:  // TODO: Finish
        break;

      /// @brief waits for 1ms at a time, basically non blocking
      case wait:
        if (counter++ < instruction.data) {
          delay(1);
        } else {  // done waiting
          instruction = (packet){next_state, -1};
        }
        break;

      case grab:  // this will just keep trying to grab on failure
        {  // seperate scope so I don't have to deal with redefining vars
          Servo32U4Base::grabber_move_state servo_state;

          if (counter++ == 0) {  // first nb call of this move
            servo_state = servo_goto_nb(Servo32U4Base::grabber_closed_pos, true);
          } else {  // subsequent calls
            servo_state = servo_goto_nb(Servo32U4Base::grabber_closed_pos, false);
          }

          if (servo_state == Servo32U4Base::grabber_move_state::success){
            instruction = (packet){next_state, -1};
            Serial.println("success");
          } else if (servo_state == Servo32U4Base::grabber_move_state::failure){  // just try again
            instruction_stack.push((packet){grab, -1});
            instruction_stack.push((packet){release, -1});
            instruction = (packet){next_state, -1};
            Serial.println("failure");
          } else {  // servo_state == Servo32U4Base::grabber_move_state::in_progress
            Serial.println("in_progress");
          }
        }
        break;

      case release:
        {  // seperate scope so I don't have to deal with redefining vars
          Servo32U4Base::grabber_move_state servo_state;

          if (counter++ == 0) {  // first nb call of this move
            servo_state = servo_goto_nb(Servo32U4Base::grabber_open_pos, true);
          } else {  // subsequent calls
            servo_state = servo_goto_nb(Servo32U4Base::grabber_open_pos, false);
          }

          if (servo_state == Servo32U4Base::grabber_move_state::success){
            instruction = (packet){next_state, -1};
          } else if (servo_state == Servo32U4Base::grabber_move_state::failure){
            instruction = (packet){estop, -1};
          } else {  // servo_state == Servo32U4Base::grabber_move_state::in_progress
            break;
          }
        }
        break;

      case grab_pos:
        {  // seperate scope so I don't have to deal with redefining vars
          float distance;
          float angle_enc_count;
          float safe_angle_enc_count;
          if (instruction.data == 0){
            distance = pos_0_distance;
            angle_enc_count = pos_0_enc_cnt;
            safe_angle_enc_count = pos_0_enc_cnt+50;
          } else if (instruction.data == 25){
            distance = pos_25_distance;
            angle_enc_count = pos_25_enc_cnt;
            safe_angle_enc_count = pos_0_enc_cnt+100;
          } else if (instruction.data == 45){
            distance = pos_45_distance;
            angle_enc_count = pos_45_enc_cnt;
            safe_angle_enc_count = pos_0_enc_cnt+100;
          } else {
            instruction = (packet){estop, -1};
            break;
          }

          instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count});
          instruction_stack.push((packet){wait_for_confirmation, -1});
          instruction_stack.push((packet){grab, -1});
          instruction_stack.push((packet){pos_four_bar, angle_enc_count});
          instruction_stack.push((packet){follow_line_to_distance_reading, distance});
          instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count});
          
          instruction = (packet){next_state, -1};
        }
        break;

      case release_pos:
        {  // seperate scope so I don't have to deal with redefining vars
          float distance;
          float angle_enc_count;
          float safe_angle_enc_count;
          if (instruction.data == 0){
            distance = pos_0_distance;
            angle_enc_count = pos_0_enc_cnt;
            safe_angle_enc_count = pos_0_enc_cnt+50;
          } else if (instruction.data == 25){
            distance = pos_25_distance;
            angle_enc_count = pos_25_enc_cnt;
            safe_angle_enc_count = pos_0_enc_cnt+100;
          } else if (instruction.data == 45){
            distance = pos_45_distance;
            angle_enc_count = pos_45_enc_cnt;
            safe_angle_enc_count = pos_0_enc_cnt+100;
          } else {
            instruction = (packet){estop, -1};
            break;
          }

          instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count});
          instruction_stack.push((packet){wait_for_confirmation, -1});
          instruction_stack.push((packet){release, -1});
          instruction_stack.push((packet){pos_four_bar, angle_enc_count});
          instruction_stack.push((packet){follow_line_to_distance_reading, distance});
          instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count});
          
          instruction = (packet){next_state, -1};
        }
        break;

      case pos_four_bar:
        if(motor.moveToNB(instruction.data)) {
          instruction = (packet){next_state, -1};
        }
        break;
    }
  }
}