#ifndef ServoStateMachine_h
#define ServoStateMachine_h

#include <Arduino.h>
#include "Global.h"
#include "JCServoEasing.h"
#include "StateMachine.h"

struct ServoPosition {
  char description[DESCRIPTION_LENGTH];
  uint8_t positionDegrees;
  uint16_t eventIndexPositionLeaving;
  uint16_t eventIndexPositionReached;
};

struct ServoArray {
  struct {
    ServoPosition position[NUM_POS];
    State currentState;
    uint8_t speedDegreesPerSecond;
    JCServoEasing servoEasing;
  } servo[NUM_SERVOS];
};

struct CrossoverArray {
  uint8_t firstServo;
  uint8_t secondServo;
  ServoPosition position[NUM_POS]; // only used for event indexes.
  State currentPosition; // so we know which leaving event to send.
  State waitingForPosition; // the position that this crossover is waiting for both its servos to reach, or UNKNOWN if not waiting.
};

class ServoStateMachine {
  public:
    /**
     * The class's constructor.
     * Initialises all the state transitions.
     */
    ServoStateMachine();


    /**
     * Copies servoArray to the class's internal copy.
     */
    void initialiseServos(ServoArray servoArray);

    /**
     * Initialises crossoverArray.
     */
    void initialiseCrossovers();

    /**
     * Checks to see if servoNumber needs moving and if it has reached its target position.
     * Returns the index of any event to send, or -1 if there is no event to send.
     * Also checks to see if any crossovers are waiting for their servos to finish moving.
     */
    int update(uint8_t servoNumber);

    /**
     * Returns the current state for servoNumber.
     */
    State currentServoState(uint8_t servoNumber);

    /**
     * Returns the current state for crossoverNumber.
     */
    State currentCrossoverPosition(uint8_t crossoverNumber);

    /**
     * Performs the following functions;-
     * - updates the current state for servoNumber.
     * - if required, causes the servo to start moving.
     * - if required, returns the index of the event to be sent, or -1 if there is no event to send.
     */
    int processStateTransition(uint8_t servoNumber, Action action);

    ServoArray servoArray;
    CrossoverArray crossoverArray[NUM_CROSSOVERS];

  private:
    const char *returnStateName(State state);
    const char *returnActionName(Action action);

    /**
     * Returns the index of the leaving event for this position of this servo.
     */
    int eventIndexLeaving(uint8_t servoNumber, uint8_t position);

    /**
     * Returns the index of the reached event for this position of this servo.
     */
    int eventIndexReached(uint8_t servoNumber, uint8_t position);

    /**
     * Contains all the valid state transitions.
     */
    StateMachine stateMachine;

};

#endif
