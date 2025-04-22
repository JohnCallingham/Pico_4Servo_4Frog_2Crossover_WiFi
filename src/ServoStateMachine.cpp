#include "ServoStateMachine.h"

#define POSITION_DESCRIPTION(p) servoArray.servo[servoNumber].position[p].description

ServoStateMachine::ServoStateMachine() {
  // Initialise the state machine by adding all the state transitions.
  // ================================= current state, action, event reached(1-3), event leaving(1-3), servo target position(1-3), new state
  stateMachine.addTransition(StateTransition(UNKNOWN, MOVE_TO_POSITION_1, 0, 0, 1, MOVING_TO_POSITION_1));
  stateMachine.addTransition(StateTransition(UNKNOWN, MOVE_TO_POSITION_2, 0, 0, 2, MOVING_TO_POSITION_2));
  stateMachine.addTransition(StateTransition(UNKNOWN, MOVE_TO_POSITION_3, 0, 0, 3, MOVING_TO_POSITION_3));
  stateMachine.addTransition(StateTransition(MOVING_TO_POSITION_1, MOVE_COMPLETED, 1, 0, 0, AT_POSITION_1));
  stateMachine.addTransition(StateTransition(MOVING_TO_POSITION_2, MOVE_COMPLETED, 2, 0, 0, AT_POSITION_2));
  stateMachine.addTransition(StateTransition(MOVING_TO_POSITION_3, MOVE_COMPLETED, 3, 0, 0, AT_POSITION_3));
  stateMachine.addTransition(StateTransition(MOVING_FROM_POSITION_1_TO_POSITION_3, MOVE_COMPLETED, 0, 2, 3, MOVING_TO_POSITION_3));
  stateMachine.addTransition(StateTransition(MOVING_FROM_POSITION_3_TO_POSITION_1, MOVE_COMPLETED, 2, 0, 1, MOVING_TO_POSITION_1));
  stateMachine.addTransition(StateTransition(AT_POSITION_1, MOVE_TO_POSITION_1, 1, 0, 0, AT_POSITION_1));
  stateMachine.addTransition(StateTransition(AT_POSITION_1, MOVE_TO_POSITION_2, 0, 1, 2, MOVING_TO_POSITION_2));
  stateMachine.addTransition(StateTransition(AT_POSITION_1, MOVE_TO_POSITION_3, 0, 1, 2, MOVING_FROM_POSITION_1_TO_POSITION_3));
  // stateMachine.addTransition(StateTransition(AT_POSITION_2, MOVE_TO_POSITION_1, 0, 1, 1, MOVING_TO_POSITION_1));
  stateMachine.addTransition(StateTransition(AT_POSITION_2, MOVE_TO_POSITION_1, 0, 2, 1, MOVING_TO_POSITION_1));
  // stateMachine.addTransition(StateTransition(AT_POSITION_2, MOVE_TO_POSITION_2, 1, 0, 0, AT_POSITION_2));
  stateMachine.addTransition(StateTransition(AT_POSITION_2, MOVE_TO_POSITION_2, 2, 0, 0, AT_POSITION_2));
  stateMachine.addTransition(StateTransition(AT_POSITION_2, MOVE_TO_POSITION_3, 0, 2, 3, MOVING_TO_POSITION_3));
  stateMachine.addTransition(StateTransition(AT_POSITION_3, MOVE_TO_POSITION_1, 0, 3, 2, MOVING_FROM_POSITION_3_TO_POSITION_1));
  stateMachine.addTransition(StateTransition(AT_POSITION_3, MOVE_TO_POSITION_2, 0, 3, 2, MOVING_TO_POSITION_2));
  stateMachine.addTransition(StateTransition(AT_POSITION_3, MOVE_TO_POSITION_3, 3, 0, 0, AT_POSITION_3));
}

const char * ServoStateMachine::returnStateName(State state) {
  switch (state) {
    case UNKNOWN: return (const char *) "UNKNOWN";
    case AT_POSITION_1: return (const char *) "AT_POSITION_1 (THROWN)";
    case AT_POSITION_2: return (const char *) "AT_POSITION_2 (MID_POINT)";
    case AT_POSITION_3: return (const char *) "AT_POSITION_3 (CLOSED)";
    case MOVING_TO_POSITION_1: return (const char *) "MOVING_TO_POSITION_1 (THROWN)";
    case MOVING_TO_POSITION_2: return (const char *) "MOVING_TO_POSITION_2 (MID_POINT)";
    case MOVING_TO_POSITION_3: return (const char *) "MOVING_TO_POSITION_3 (CLOSED)";
    case MOVING_FROM_POSITION_1_TO_POSITION_3: return (const char *) "MOVING_FROM_POSITION_1_(THROWN)_TO_POSITION_3_(CLOSED)";
    case MOVING_FROM_POSITION_3_TO_POSITION_1: return (const char *) "MOVING_FROM_POSITION_3_(CLOSED)_TO_POSITION_1_(THROWN)";
    default: return (const char *) "Invalid state";
  }
}

const char * ServoStateMachine::returnActionName(Action action) {
  switch (action) {
    case MOVE_TO_POSITION_1: return (const char *) "MOVE_TO_THROWN";
    case MOVE_TO_POSITION_2: return (const char *) "MOVE_TO_MID_POINT";
    case MOVE_TO_POSITION_3: return (const char *) "MOVE_TO_CLOSED";
    case MOVE_COMPLETED: return (const char *) "MOVE_COMPLETED";
    default: return (const char *) "Invalid action";
  }
}

void ServoStateMachine::initialiseServos(ServoArray servoArray) {
  // Copy the supplied array to our private object.
  this->servoArray = servoArray;

  // For all servos convert their speed to a delay in mS for the servo easing to work.
  // Also set the servo's current and target angles to the servo's configured mid position.
  for (int i=0; i<NUM_SERVOS; i++) {
    this->servoArray.servo[i].servoEasing.setDelaymS(300/this->servoArray.servo[i].speedDegreesPerSecond); // Changed 1000 to 300 to cope with inaccurate millis().

    // Set the target and current angles to stop the servo moving when initialised.
    this->servoArray.servo[i].servoEasing.setCurrentAngle(this->servoArray.servo[i].position[1].positionDegrees);
    this->servoArray.servo[i].servoEasing.setTargetAngle(this->servoArray.servo[i].position[1].positionDegrees);

    // Attach the servo to its pin and set its initial angle.
    this->servoArray.servo[i].servoEasing.initialise(this->servoArray.servo[i].position[1].positionDegrees);

    // Set the servo's initial state to mid point.
    this->servoArray.servo[i].currentState = State::AT_POSITION_2;
  }
}

void ServoStateMachine::initialiseCrossovers() {

  // Initialise the CrossoverArray.
  // Determine the servos for the crossovers.
  // Need an algorithm that gives servo=0 for crossover=0, servo=2 for crossover=1, servo=4 for crossover=2, etc.
  crossoverArray[0].firstServo = 0;
  crossoverArray[0].secondServo = 1;
  crossoverArray[1].firstServo = 2;
  crossoverArray[1].secondServo = 3;

  for (int i=0; i<NUM_CROSSOVERS; i++) {
    // Calculate the event indexes.
    for (int j=0; j<NUM_POS; j++) {
      crossoverArray[i].position[j].eventIndexPositionReached = EVENT_INDEX_CROSSOVER_START + (i * 9) + (j * 3) + 1;
      crossoverArray[i].position[j].eventIndexPositionLeaving = EVENT_INDEX_CROSSOVER_START + (i * 9) + (j * 3) + 2;
    }

    // Set all crossovers to be not waiting for any servos to finish moving.
    crossoverArray[i].waitingForPosition = UNKNOWN;

    // // Set their current position to UNKNOWN so they don't send a leaving event when first moved.
    // // This prevents the crossover from moving when toggled immediately after reset!!
    // crossoverArray[i].currentPosition = UNKNOWN;

    // Set their current position to AT_POSITION_2 (Mid) so they can be moved after reset.
    crossoverArray[i].currentPosition = AT_POSITION_2;
  }
}

int ServoStateMachine::update(uint8_t servoNumber) {
  // Check for servo movements.
  if (servoArray.servo[servoNumber].servoEasing.update()) {
    // The servo has reached its target position.
    return processStateTransition(servoNumber, MOVE_COMPLETED);
  }

  // Check if any crossovers are waiting for their servos to finish moving.
  for (int i=0; i<NUM_CROSSOVERS; i++) {
    if (crossoverArray[i].waitingForPosition != UNKNOWN) {
      // This crossover is waiting for both its servos to finish moving.
      if ((crossoverArray[i].waitingForPosition == servoArray.servo[crossoverArray[i].firstServo].currentState) && (crossoverArray[i].waitingForPosition == servoArray.servo[crossoverArray[i].secondServo].currentState)) {

        // Both servos have finished moving.
        crossoverArray[i].currentPosition = crossoverArray[i].waitingForPosition; // Set the new state for this crossover.

        // Send the crossover's reached event for the position reached.
        switch (crossoverArray[i].waitingForPosition) {
          case AT_POSITION_1:
            crossoverArray[i].waitingForPosition = UNKNOWN; // Stop checking for both servos finished moving.
            Serial.printf("\nCrossover %d sending reached event for position 1");
            return crossoverArray[i].position[0].eventIndexPositionReached;
            break;
          case AT_POSITION_2:
            crossoverArray[i].waitingForPosition = UNKNOWN; // Stop checking for both servos finished moving.
            Serial.printf("\nCrossover %d sending reached event for position 2");
            return crossoverArray[i].position[1].eventIndexPositionReached;
            break;
          case AT_POSITION_3:
            crossoverArray[i].waitingForPosition = UNKNOWN; // Stop checking for both servos finished moving.
            Serial.printf("\nCrossover %d sending reached event for position 3");
            return crossoverArray[i].position[2].eventIndexPositionReached;
            break;
        }
      }
    }
  }

  return -1;
}

int ServoStateMachine::processStateTransition(uint8_t servoNumber, Action action) {
  State currentState = servoArray.servo[servoNumber].currentState;

  Serial.printf("\nServo %d", servoNumber+1);
  Serial.printf(" current state is %s", returnStateName(currentState));
  Serial.printf(" action is %s", returnActionName(action));

  // Find the transition which matches this servo's current state and action.
  StateTransition foundTransition = stateMachine.findTransition(currentState, action);

  // Check for an invalid transition.
  if (foundTransition.newState == UNKNOWN) {
    // // Give an error message and set the current state to UNKNOWN so that we may recover.
    // Serial.printf("\nServo %d invalid transition", servoNumber+1);
    // servoArray.servo[servoNumber].currentState = UNKNOWN;

    // Give an error message and ignore.
    Serial.printf("\nServo %d invalid transition ignored", servoNumber+1);
    //servoArray.servo[servoNumber].currentState = UNKNOWN;

    // No event to send.
    return -1;
  }

  // This must be a valid transition.

  // If required set the servo's target angle.
  int targetPosition = foundTransition.setServoTargetPosition;
  //Serial.printf("\ntargetPosition=%d", targetPosition);
  uint8_t targetPositionAngle;
  if (targetPosition != -1) {
    targetPositionAngle = servoArray.servo[servoNumber].position[targetPosition].positionDegrees;

    Serial.printf("\nServo %d moving to position %d (%s) (%d degrees)", servoNumber+1, targetPosition, POSITION_DESCRIPTION(targetPosition), targetPositionAngle);
    servoArray.servo[servoNumber].servoEasing.setTargetAngle(targetPositionAngle);
  }

  // Update the current state to be the new state.
  servoArray.servo[servoNumber].currentState = foundTransition.newState;
  Serial.printf("\nServo %d new state is %s", servoNumber+1, returnStateName(servoArray.servo[servoNumber].currentState));

  // If required return the leaving event.
  if (foundTransition.eventLeaving != -1) return eventIndexLeaving(servoNumber, foundTransition.eventLeaving);

  // If required return the reached event.
  if (foundTransition.eventReached != -1) return eventIndexReached(servoNumber, foundTransition.eventReached);

  return -1;
}

int ServoStateMachine::eventIndexLeaving(uint8_t servoNumber, uint8_t position) {
  Serial.printf("\nServo %d sending event leaving position %d (%s)", servoNumber+1, position+1, POSITION_DESCRIPTION(position));
  return servoArray.servo[servoNumber].position[position].eventIndexPositionLeaving;
}

int ServoStateMachine::eventIndexReached(uint8_t servoNumber, uint8_t position) {
  Serial.printf("\nServo %d sending event reached position %d (%s)", servoNumber+1, position+1, POSITION_DESCRIPTION(position));
  return servoArray.servo[servoNumber].position[position].eventIndexPositionReached;
}

State ServoStateMachine::currentServoState(uint8_t servoNumber) {
  return servoArray.servo[servoNumber].currentState;
}

State ServoStateMachine::currentCrossoverPosition(uint8_t crossoverNumber) {
  return crossoverArray[crossoverNumber].currentPosition;
}
