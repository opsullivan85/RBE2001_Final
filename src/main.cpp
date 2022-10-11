#include <Arduino.h>
#include <Romi32U4.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include "StackArray.h"
#include "BlueMotor.h"
#include "servo32u4.h"
#include "line_following_functions.h"
#include "grabber_functions.h"
#include "turning_functions.h"

#define LF_P 0.0002
#define LINE_FOLLOWING_EFFORT 150
#define DST_P 0.17
#define TURN_P 0.4
#define TURN_EFFORT 100
#define TURN_TOLERANCE 10
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
IRDecoder decoder(14);

void setup()
{
  Serial.begin(9600);
  chassis.init();
  decoder.init();
  rangefinder.init();
  pinMode(REFL_L, INPUT);
  motor.setup();
  servo.setMinMaxMicroseconds(500, 2500);   //1035 closed, 1900 open
  pinMode(REFL_R, INPUT);
  buttonB.waitForButton();
  delay(500);
  
  // int counter = 0;
  // while(true){
  //   servo.writeMicroseconds(1000);
  //   Serial.println("alive");
  //   Serial.println(counter++);
  // }
}


enum states {
  /// @brief sets up the state machine
  /// @param data unused
  initilize,

  /// @brief safely stops the robot
  /// @param data unused
  estop,
  
  /// @brief advances the state machine
  /// @param data unused
  next_state,

  /// @brief turns to the closest line in the specified direction
  ///        won't do anything if already on line
  /// @param data turn direction (0 left, 1 right)
  turn_to_line,

  /// @brief turns some ammount of radians
  /// @param data angle to turn counterclockwise (left) (rad)
  turn_rad,

  /// @brief turns to the next closest line in the specified direction
  /// @param data turn direction (0 left, 1 right)
  turn_to_next_line,

  /// @brief turns and goes to intersection.
  /// @param data direction to turn at intersection (0 left, 1 right)
  orient_to_intersection,

  /// @brief follows line until specified distance sensor reading
  /// @param data distance reading (cm)
  follow_line_to_distance_reading,

  /// @brief follows line until romi is centered over intersection
  /// @param data unused
  follow_line_to_over_intersection,

  /// @brief follows line for a specified distance
  /// @param data distance to move (cm)
  follow_line_distance,

  /// @brief wait for confirmation on ir remote
  /// @param data unused
  wait_for_confirmation,

  /// @brief waits for some time
  /// @param data time to wait, (~ms)
  wait,

  /// @brief closes the grabber
  /// @param data unused
  grab,

  /// @brief opens the grabber
  /// @param data unused
  release,

  /// @brief follows line to and grabs plate from platform
  ///        at specified angle
  /// @param data four bar position (deg)
  grab_pos,

  /// @brief follows line to and releases plate onto platform
  ///        at specified angle
  /// @param data four bar position (deg)
  release_pos,

  /// @brief positions the four bar at specified angle
  /// @param data four bar position (encoder counts)
  pos_four_bar,

  /// @brief handles ir remote input.
  ///        treats everything but ENTER_SAVE as estop.
  ///        temporarily interrupts instructions, resets counter.
  /// @param data unused
  handle_ir_remote,  
};

struct packet {
  states state;
  float data;
  int counter_init_val;  // I thought there was a way to default initilize this to zero, but I cant find it
};


String state_to_string(states state){
    static const String decoder[] = {
    "initilize",
    "estop",
    "next_state",
    "turn_to_line",
    "turn_rad",
    "turn_to_next_line",
    "orient_to_intersection",  //TODO: verify
    "follow_line_to_distance_reading",  //TODO: verify
    "follow_line_to_over_intersection",
    "follow_line_distance",
    "wait_for_confirmation",
    "wait",
    "grab",
    "release",
    "grab_pos",  //TODO: verify
    "release_pos",  //TODO: verify
    "pos_four_bar",
    "handle_ir_remote",
  };
  return decoder[state];
}

void print_packet(packet p){
  Serial.print(state_to_string(p.state));
  Serial.print(" ");
  Serial.print(p.data);
  Serial.print(" ");
  Serial.print(p.counter_init_val);
}

void debug_printer(packet p, int counter){
  if(p.state != wait || counter == 0){  // only print the first wait, cut down on noise
    Serial.println();
    if(p.state == next_state){Serial.print(" ");}
    print_packet(p);
    Serial.print(" ");
    Serial.print(counter);
  }
}

void print_instruction_stack(StackArray<packet> &instruction_stack){
  StackArray<packet> tmp_stack;
  packet tmp_packet;

  Serial.println();
  while (!instruction_stack.isEmpty()){
    tmp_packet = instruction_stack.pop();
    tmp_stack.push(tmp_packet);
    print_packet(tmp_packet);
    Serial.println();
  }
  Serial.println();

  while (!tmp_stack.isEmpty()){
    instruction_stack.push(tmp_stack.pop());
  }

}

void loop(){
  #define debug
  StackArray<packet> instruction_stack;
  // instruction_stack.push((packet){initilize, -1, 0});
  instruction_stack.push((packet){follow_line_to_distance_reading, 12, 0});
  packet instruction = (packet){next_state, -1, 0};

  /// @brief used to track data between state calls.
  int counter = 0;
  int ir_remote_code;
  unsigned long wait_timer;

  while (true){
    #ifdef debug
    debug_printer(instruction, counter);
    // print_instruction_stack(instruction_stack);
    #endif

    ir_remote_code = decoder.getKeyCode();  // read ir remote
    switch (instruction.state) {
      case initilize:
        // this is a stack so instructions should be read bottom up
        instruction_stack.push((packet){release_pos, 25, 0});  // replace plate for 25 pos
        instruction_stack.push((packet){orient_to_intersection, 0, 0}); // orient for 25 pos

        instruction_stack.push((packet){grab_pos, 0, 0});  // grab new plate from 0 deg pos
        instruction_stack.push((packet){release_pos, 0, 0});  // release old plate at 0 deg pos

        instruction_stack.push((packet){orient_to_intersection, 0, 0});  // orient for 0 pos
        instruction_stack.push((packet){grab_pos, 25, 0});  // grab from 45 deg pos
        instruction_stack.push((packet){orient_to_intersection, 1, 0});  // orient for 45 pos

        instruction_stack.push((packet){follow_line_to_over_intersection, -1, 0});  // cross field
        instruction_stack.push((packet){turn_rad, 270, 0});  // turn to face other side of field
        instruction_stack.push((packet){follow_line_to_distance_reading, 10, 0});  // go to position for crossing
        instruction_stack.push((packet){orient_to_intersection, 1, 0});  // orient for 0 pos

        instruction_stack.push((packet){release_pos, 45, 0});  // replace plate for 45 pos
        instruction_stack.push((packet){orient_to_intersection, 0, 0}); // orient for 45 pos

        instruction_stack.push((packet){grab_pos, 0, 0});  // grab new plate from 0 deg pos
        instruction_stack.push((packet){release_pos, 0, 0});  // release old plate at 0 deg pos

        instruction_stack.push((packet){orient_to_intersection, 1, 0});  // orient for 0 pos
        instruction_stack.push((packet){grab_pos, 45, 0});  // grab from 4F5 deg pos

        instruction = (packet){next_state, -1, 0};
        break;

      case orient_to_intersection:
        instruction_stack.push((packet){turn_to_line, instruction.data, 0});  // turn specified direction
        instruction_stack.push((packet){follow_line_to_over_intersection, -1, 0});  // go to intersection
        instruction_stack.push((packet){turn_to_line, 1, 0});  // full turn on single line
        instruction = (packet){next_state, -1, 0};
        break;

      case estop:
        motor.setEffort(0);
        chassis.idle();
        servo_goto_nb(Servo32U4Base::grabber_open_pos, true);

        while(true) {delay(10);}
        break;

      case next_state:
        if (!instruction_stack.isEmpty()) {
          instruction = instruction_stack.pop();
          counter = instruction.counter_init_val;  // reset the counter so the next state can use it
        } else {
          instruction = (packet){estop, -1, 0};
        }
        break;

      case turn_to_line:
        if (instruction.data == 0) {  // left
          chassis.setMotorEfforts(-TURN_EFFORT, TURN_EFFORT);
          if (on_tape(analogRead(REFL_R))){
            chassis.setMotorEfforts(0,0);
            instruction = (packet){next_state, -1, 0};
          }
        } else {  // instruction.data == 1  // right
          chassis.setMotorEfforts(TURN_EFFORT, -TURN_EFFORT);
          if (on_tape(analogRead(REFL_L))){
            chassis.setMotorEfforts(0,0);
            instruction = (packet){next_state, -1, 0};
          }
        }

        break;

      case turn_rad:
        if(counter++ == 0){
          if (turn_rad_nb(TURN_EFFORT, TURN_P, instruction.data, TURN_TOLERANCE, true)) {
            instruction = (packet){next_state, -1, 0};
          }
        } else {
          if (turn_rad_nb(TURN_EFFORT, TURN_P, instruction.data, TURN_TOLERANCE, false)) {
            instruction = (packet){next_state, -1, 0};
          }
        }
        break;

      case turn_to_next_line:
        instruction_stack.push((packet){turn_to_line, instruction.data, 0});
        if(instruction.data == 0){
          instruction_stack.push((packet){turn_rad, -PI/8, 0});
        } else {  // instruction.data == 1
          instruction_stack.push((packet){turn_rad, PI/8, 0});
        }
        instruction = (packet){next_state, -1, 0};
        break;

      case follow_line_to_distance_reading:
        if (follow_line_to_distance_reading_nb(LINE_FOLLOWING_EFFORT, LF_P, DST_P, 120, 0.1)){
          instruction = (packet){next_state, -1, 0};
        }
        break;

      case follow_line_to_over_intersection:
        if (follow_line_to_intersection_nb(LINE_FOLLOWING_EFFORT, LF_P)){
          instruction_stack.push((packet){follow_line_distance, ROMI_RADIUS, 0});
          instruction = (packet){next_state, -1, 0};
        }
        break;

      case follow_line_distance:
        if (counter++ == 0) {  // first nb call of this move
          if (follow_line_distance_nb(LINE_FOLLOWING_EFFORT, LF_P, instruction.data, true)){  // reset fn
            instruction = (packet){next_state, -1, 0};
          }
        } else {  // subsequent calls
          if (follow_line_distance_nb(LINE_FOLLOWING_EFFORT, LF_P, instruction.data, false)){  // dont reset fn
            instruction = (packet){next_state, -1, 0};
          }
        }

        break;

      case wait_for_confirmation:
        if (ir_remote_code == ENTER_SAVE) {
          instruction = (packet){next_state, -1, 0};
        }
        break;

      /// @brief waits for 0.5ms at a time, basically non blocking
      case wait:
        if (counter == 0) {
          counter++;
          wait_timer = millis();
        }
        
        if(wait_timer + instruction.data <= millis()) {  // done waiting
          instruction = (packet){next_state, -1, 0};
        } else {  // keep waiting
          counter = constrain((int)(wait_timer + instruction.data - millis()),1, __INT_MAX__);
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
            instruction = (packet){next_state, -1, 0};
          } else if (servo_state == Servo32U4Base::grabber_move_state::failure){  // just try again
            instruction_stack.push((packet){grab, -1, 0});
            instruction_stack.push((packet){release, -1, 0});
            instruction = (packet){next_state, -1, 0};
          } else {  // servo_state == Servo32U4Base::grabber_move_state::in_progress
            instruction_stack.push((packet){grab, -1, counter});
            instruction_stack.push((packet){wait, 20, 0});
            instruction = (packet){next_state, -1, 0};
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
            instruction = (packet){next_state, -1, 0};
          } else if (servo_state == Servo32U4Base::grabber_move_state::failure){  // give up
            instruction = (packet){estop, -1, 0};
          } else {  // servo_state == Servo32U4Base::grabber_move_state::in_progress
            instruction_stack.push((packet){release, -1, counter});
            instruction_stack.push((packet){wait, 30, 0});
            instruction = (packet){next_state, -1, 0};
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
            instruction = (packet){estop, -1, 0};
            break;
          }

          instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count, 0});
          instruction_stack.push((packet){wait_for_confirmation, -1, 0});
          instruction_stack.push((packet){grab, -1, 0});
          instruction_stack.push((packet){pos_four_bar, angle_enc_count, 0});
          instruction_stack.push((packet){follow_line_to_distance_reading, distance, 0});
          instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count, 0});
          
          instruction = (packet){next_state, -1, 0};
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
            instruction = (packet){estop, -1, 0};
            break;
          }

          instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count, 0});
          instruction_stack.push((packet){wait_for_confirmation, -1, 0});
          instruction_stack.push((packet){release, -1, 0});
          instruction_stack.push((packet){pos_four_bar, angle_enc_count, 0});
          instruction_stack.push((packet){follow_line_to_distance_reading, distance, 0});
          instruction_stack.push((packet){pos_four_bar, safe_angle_enc_count, 0});
          
          instruction = (packet){next_state, -1, 0};
        }
        break;

      case pos_four_bar:
        if(motor.moveToNB(instruction.data)) {
          instruction = (packet){next_state, -1, 0};
        }
        break;

      case handle_ir_remote:
        if (instruction.data != ENTER_SAVE){
          instruction = (packet){estop, -1, 0};
        } else {
          instruction = (packet){next_state, -1, 0};
        }
        break;
    }

    // handle un-caught ir codes
    if(ir_remote_code >= 0){
      instruction_stack.push((packet){instruction.state, instruction.data, counter});  // maintains current counter val
      instruction_stack.push((packet){handle_ir_remote, ir_remote_code, 0});
      instruction = (packet){next_state, -1, 0};
    }
  }
}