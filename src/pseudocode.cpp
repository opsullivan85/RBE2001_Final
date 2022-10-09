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
  // Serial.print("left: ");
  // Serial.println(l);
  // Serial.print("right: ");
  // Serial.println(r);
  // Serial.println();
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


// /// @brief turns until robot is off of current line, and until it is on next line in direction.
// ///        assumes intersection to be behind sensor
// /// @param effort 
// /// @param precision_effort effort used once one sensor detects the final line
// /// @param direction -1 is left 1 is right
// void turn_in_place_to_line(int effort, int precision_effort, int direction){
//   int l = analogRead(REFL_L);
//   int r = analogRead(REFL_R);
  
//   chassis.setMotorEfforts(effort * direction, -effort * direction);

//   if (direction == -1) {  // turning left
//     while (on_tape(l)) {  // wait for left sensor to clear tape
//       l = analogRead(REFL_L);
//     }

//     r = analogRead(REFL_R);
//     while (!on_tape(r)) {  // wait for right sensor to hit tape, left should be clear now
//       r = analogRead(REFL_R);
//     }
    
//     while (!on_tape(l)) {  // wait for left sensor to hit final tape
//       l = analogRead(REFL_L);
//     }

//     // slow down
//     chassis.setMotorEfforts(precision_effort * direction, -precision_effort * direction);

//     while (!on_white(l) && !on_white(r)) {  // wait for sensors to be straddling tape
//       l = analogRead(REFL_L);
//       r = analogRead(REFL_R);
//     }

//   } else if (direction == 1) {  // turning right
//     while (on_tape(r)) {  // wait for right sensor to clear tape
//       r = analogRead(REFL_R);
//     }

//     l = analogRead(REFL_L);
//     while (!on_tape(l)) {  // wait for left sensor to hit tape, right should be clear now
//       l = analogRead(REFL_L);
//     }
    
//     while (!on_tape(r)) {  // wait for right sensor to hit final tape
//       r = analogRead(REFL_R);
//     }

//     // slow down
//     chassis.setMotorEfforts(precision_effort * direction, -precision_effort * direction);

//     while (!on_white(l) && !on_white(r)) {  // wait for sensors to be straddling tape
//       l = analogRead(REFL_L);
//       r = analogRead(REFL_R);
//     }
//   }

//   chassis.setMotorEfforts(0, 0);
// }


/// @brief follows a line to the next intersection
/// @param base_effort speed to run the motors at
/// @param p p factor
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


bool check_for_estop(){
  return(false);
}















enum states {
  initilize,
  estop,
  next_state,
  turn_to_line,  // expects data for turn direction (0 left, 1 right)
  turn_deg,  // expects an angle to turn counterclockwise (deg 0-359)
  orient_to_intersection,  // turns and goes to intersection. 
                           // expects data for direction to turn at intersection (0 left, 1 right)
  follow_line_to_distance_reading,  // expects data for distance reading (cm)
  follow_line_to_over_intersection,
  follow_line_distance,  // expects data for distance to move (cm)
  wait_for_confirmation,
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

void loop(){
  StackArray<packet> instruction_stack;
  instruction_stack.push((packet){initilize, -1});
  packet instruction = (packet){next_state, -1};
  int counter;

  while (true){
    if(check_for_estop){
      instruction.state = estop;
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
        instruction_stack.push((packet){turn_deg, 270});  // turn to face other side of field
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

      case next_state:
        counter = 0;  // reset the counter so the next state can use it
        if (!instruction_stack.isEmpty()) {
          instruction = instruction_stack.pop();
        } else {
          instruction.state = estop;
        }
        break;

      case grab_pos:
        float distance;
        int angle_enc_count;
        int safe_angle_enc_count;
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
          instruction.state = estop;
          break;
        }

        instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count});
        instruction_stack.push((packet){wait_for_confirmation, -1});
        instruction_stack.push((packet){grab, -1});
        instruction_stack.push((packet){pos_four_bar, angle_enc_count});
        instruction_stack.push((packet){follow_line_to_distance_reading, distance});
        instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count});
        
        instruction = (packet){next_state, -1};
        break;

      case orient_to_intersection:
        instruction_stack.push((packet){turn_to_line, instruction.data});  // turn specified direction
        instruction_stack.push((packet){follow_line_to_over_intersection, -1});  // go to intersection
        instruction_stack.push((packet){turn_to_line, 1});  // full turn on single line

        instruction = (packet){next_state, -1};
        break;

      case estop:
      case turn_to_line:
      case follow_line_to_distance_reading:
      case follow_line_to_over_intersection:
      case wait_for_confirmation:
      case grab:
      case release:
      case release_pos:
      case pos_four_bar:
    }
  }
}