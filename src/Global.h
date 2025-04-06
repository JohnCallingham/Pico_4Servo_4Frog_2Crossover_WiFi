#ifndef Global_h
#define Global_h

#include "JCServoEasing.h"
#include <Servo.h>

// WiFi uses one PIO so only seven servos are available. Only four are needed for this sketch.
#define NUM_SERVOS 4
#define NUM_POS 3

#define NUM_OUTPUTS 8

#define NUM_CROSSOVERS 2

const int NUM_SERVO_EVENTS = (NUM_SERVOS * NUM_POS * 3);
const int NUM_OUTPUT_EVENTS = (NUM_OUTPUTS * 2);
const int NUM_CROSSOVER_EVENTS = (NUM_CROSSOVERS * NUM_POS * 3);

#define NUM_EVENT NUM_SERVO_EVENTS + NUM_OUTPUT_EVENTS + NUM_CROSSOVER_EVENTS

// Used to determine what type of event has been received.
const int EVENT_INDEX_SERVO_START = 0;
const int EVENT_INDEX_SERVO_END = ((EVENT_INDEX_SERVO_START) + NUM_SERVO_EVENTS - 1);
const int EVENT_INDEX_OUTPUT_START = ((EVENT_INDEX_SERVO_END) + 1);
const int EVENT_INDEX_OUTPUT_END = ((EVENT_INDEX_OUTPUT_START) + NUM_OUTPUT_EVENTS - 1);
const int EVENT_INDEX_CROSSOVER_START = ((EVENT_INDEX_OUTPUT_END) + 1);
const int EVENT_INDEX_CROSSOVER_END = ((EVENT_INDEX_CROSSOVER_START) + NUM_CROSSOVER_EVENTS - 1);
#define INDEX_IS_SERVO_EVENT(index) ((index >= EVENT_INDEX_SERVO_START) && (index <= EVENT_INDEX_SERVO_END))
#define INDEX_IS_OUTPUT_EVENT(index) ((index >= EVENT_INDEX_OUTPUT_START) && (index <= EVENT_INDEX_OUTPUT_END))
#define INDEX_IS_CROSSOVER_EVENT(index) ((index >= EVENT_INDEX_CROSSOVER_START) && (index <= EVENT_INDEX_CROSSOVER_END))

#define DESCRIPTION_LENGTH 16 // Used for all description char arrays.

#define SERVO_PWM_DEG_0    540 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_PWM_DEG_180  2400 // this is the 'maximum' pulse length count (out of 4096)

// #define SERVO_SPEED 45 // servo speed in degrees per second.
#define SERVO_SPEED 30 // servo speed in degrees per second.

// Moved from ServoStateMachine.h
// States and Actions required to maintain the state of each servo.
enum State {
    UNKNOWN,
    AT_POSITION_1,
    AT_POSITION_2,
    AT_POSITION_3,
    MOVING_TO_POSITION_1,
    MOVING_TO_POSITION_2,
    MOVING_TO_POSITION_3,
    MOVING_FROM_POSITION_1_TO_POSITION_3,
    MOVING_FROM_POSITION_3_TO_POSITION_1
  };
  
  enum Action {
    MOVE_TO_POSITION_1,
    MOVE_TO_POSITION_2,
    MOVE_TO_POSITION_3,
    MOVE_COMPLETED
  };

#endif
