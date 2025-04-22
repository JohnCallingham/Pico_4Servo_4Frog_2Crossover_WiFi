#include "ConsumedEvents.h"

// Define the pins for the outputs.
uint8_t outputPin[] = { 26, 22, 21, 20, 19, 18, 17, 16 };

void initialiseOutputPins() {
  // Set all output pins to outputs.
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    pinMode(outputPin[i], OUTPUT);
    digitalWrite(outputPin[i], LOW);
  }
}

EventsToSend handleConsumedEvent(uint16_t index, ServoStateMachine* servoStateMachine) {

  Serial.printf("\n\nevent received: index=0x%02X", index);

  if (INDEX_IS_SERVO_EVENT(index)) {
    return handleServoEvent(index, servoStateMachine);
  } else if (INDEX_IS_OUTPUT_EVENT(index)) {
    return handleOutputEvent(index, servoStateMachine);
  } else if (INDEX_IS_CROSSOVER_EVENT(index)) {
    return handleCrossoverEvent(index, servoStateMachine);
  } else {
    return EventsToSend {-1, -1, -1}; // To keep the compiler happy!!
  }
}

EventsToSend handleServoEvent(uint16_t index, ServoStateMachine* servoStateMachine) {
  // Update the servo state according to the servo number and position requested.
  // Also handle a toggle event.
  EventsToSend eventsToSend;
  eventsToSend.event[0] = -1;
  eventsToSend.event[1] = -1;
  eventsToSend.event[2] = -1;

  uint8_t servoNumber = index / NUM_EVENTS_PER_SERVO; // servoNumber is 0 - 3. 3 positions per servo, 3 events per position, toggle event.
  uint8_t servoPosition = (index % NUM_EVENTS_PER_SERVO) / NUM_POS; // servoPosition is 0 - 2.
  Serial.printf("\nservoNumber=%d, servoPosition=%d", servoNumber, servoPosition);
  //int eventToSend = -1;

  if (servoPosition == 3) {
    // This is a toggle event.

    // Get the current state for this servo and move to the opposite end.
    switch (servoStateMachine->currentServoState(servoNumber)) {
      case AT_POSITION_1:
      eventsToSend.event[0] = servoStateMachine->processStateTransition(servoNumber, MOVE_TO_POSITION_3);
        break;
      case AT_POSITION_2:
        // what to do if the current position is mid??
        // poss do nothing??
        // OR send the reached mid position event ??
        break;
      case AT_POSITION_3:
      eventsToSend.event[0] = servoStateMachine->processStateTransition(servoNumber, MOVE_TO_POSITION_1);
        break;
      default:
        break;
    }
  } else {
    // This is a move to position event.
    switch (servoPosition) {
      case 0:
        eventsToSend.event[0] = servoStateMachine->processStateTransition(servoNumber, MOVE_TO_POSITION_1);
        break;
      case 1:
        eventsToSend.event[0] = servoStateMachine->processStateTransition(servoNumber, MOVE_TO_POSITION_2);
        break;
      case 2:
        eventsToSend.event[0] = servoStateMachine->processStateTransition(servoNumber, MOVE_TO_POSITION_3);
        break;
    }
  }
  // if (eventToSend != -1) {
  //   OpenLcb.produce(eventToSend);
  // }
  return eventsToSend;
}

EventsToSend handleOutputEvent(uint16_t index, ServoStateMachine* servoStateMachine) {
  // Initialise the default return value.
  EventsToSend eventsToSend;
  eventsToSend.event[0] = -1;
  eventsToSend.event[1] = -1;
  eventsToSend.event[2] = -1;

  // uint8_t outputEvent = index - NUM_SERVO_EVENTS; // there are 16 output events. 4 frogs, each frog has J low, J high, K low and K high.
  uint8_t outputEvent = index - EVENT_INDEX_OUTPUT_START;
  Serial.printf("\n\noutputEvent = %d", outputEvent);

  uint8_t outputNumber = (outputEvent) / 2; // outputNumber is 0 - 7.
  uint8_t frogNumber = outputNumber / 2; // frogNumber is 0 - 3.
  uint8_t outputState = (outputEvent) % 2; // outputState: 0 is low, 1 is high.

  if (outputState == 1) {
    // Turn both frog outputs low to ensure that a frog cannot have both J and K outputs high at the same time.
    uint8_t outputPinJ = (frogNumber * 2) + 0; // the output pin for the J connection for this frog.
    digitalWrite(outputPin[outputPinJ], LOW);
    uint8_t outputPinK = (frogNumber * 2) + 1; // the output pin for the K connection for this frog.
    digitalWrite(outputPin[outputPinK], LOW);

    // Turn the required output high.
    digitalWrite(outputPin[outputNumber], HIGH);
  } else {
    digitalWrite(outputPin[outputNumber], LOW);
  }

  // No events to send.
  return eventsToSend;
}

EventsToSend handleCrossoverEvent(uint16_t index, ServoStateMachine* servoStateMachine) {
  EventsToSend eventsToSend;
  // eventsToSend.event[0] = -1;
  // eventsToSend.event[1] = -1;
  // eventsToSend.event[2] = -1;

  Serial.printf("\nCrossover event received");

  uint8_t crossoverEvent = index - EVENT_INDEX_CROSSOVER_START;
  Serial.printf("\n\ncrossoverEvent = %d", crossoverEvent);

  // Determine crossover number and target position.
  // uint8_t crossoverNumber = crossoverEvent / 9; 
  uint8_t crossoverNumber = crossoverEvent / NUM_EVENTS_PER_CROSSOVER; 
  // uint8_t crossoverPosition = (crossoverEvent % 9) / 3;
  uint8_t crossoverTargetPosition = (crossoverEvent % NUM_EVENTS_PER_CROSSOVER) / NUM_POS;
  Serial.printf("\nCrossover %d, target position %d", crossoverNumber, crossoverTargetPosition);

  if (crossoverTargetPosition == 3) {
    // This is a toggle event.

    // Get the current state for this crossover and move to the opposite end.
    switch (servoStateMachine->currentCrossoverPosition(crossoverNumber)) {
      case AT_POSITION_1:
        Serial.printf("\ncurrent position AT_POSITION_1");
        eventsToSend = moveCrossoverToTargetPosition(crossoverNumber, 2, servoStateMachine);
        break;
      case AT_POSITION_2:
        // What to do if the current position is mid??
        // First thought was to do nothing.
        // BUT, if the crossover is set to mid after a reset, it will never be able to move by toggling.
        // So, it is better to move to one end.
        Serial.printf("\ncurrent position AT_POSITION_2");
        eventsToSend = moveCrossoverToTargetPosition(crossoverNumber, 0, servoStateMachine);
        break;
      case AT_POSITION_3:
      Serial.printf("\ncurrent position AT_POSITION_3");
      eventsToSend = moveCrossoverToTargetPosition(crossoverNumber, 0, servoStateMachine);
        break;
      default:
        break;
    }
  } else {
    // This is a move to position event.
    eventsToSend = moveCrossoverToTargetPosition(crossoverNumber, crossoverTargetPosition, servoStateMachine);
  }

  return eventsToSend;
}

EventsToSend moveCrossoverToTargetPosition(uint8_t crossoverNumber, uint8_t crossoverTargetPosition, ServoStateMachine* servoStateMachine) {
  EventsToSend eventsToSend;
  eventsToSend.event[0] = -1;
  eventsToSend.event[1] = -1;
  eventsToSend.event[2] = -1;

  // Send the crossover's leaving event for the current position.
  switch (servoStateMachine->crossoverArray[crossoverNumber].currentPosition) {
    case UNKNOWN:
      // No event to send.
      break;
    case AT_POSITION_1:
      //OpenLcb.produce(servoStateMachine->crossoverArray[crossoverNumber].position[0].eventIndexPositionLeaving);
      Serial.printf("\nCrossover %d sending event leaving position 1", crossoverNumber+1);
      eventsToSend.event[0] = servoStateMachine->crossoverArray[crossoverNumber].position[0].eventIndexPositionLeaving;
      break;
    case AT_POSITION_2:
      //OpenLcb.produce(servoStateMachine->crossoverArray[crossoverNumber].position[1].eventIndexPositionLeaving);
      Serial.printf("\nCrossover %d sending event leaving position 2", crossoverNumber+1);
      eventsToSend.event[0] = servoStateMachine->crossoverArray[crossoverNumber].position[1].eventIndexPositionLeaving;
      break;
    case AT_POSITION_3:
      //OpenLcb.produce(servoStateMachine->crossoverArray[crossoverNumber].position[2].eventIndexPositionLeaving);
      Serial.printf("\nCrossover %d sending event leaving position 3", crossoverNumber+1);
      eventsToSend.event[0] = servoStateMachine->crossoverArray[crossoverNumber].position[2].eventIndexPositionLeaving;
      break;
  }

  // Start the first servo moving to the target position.
  int eventToSend;
  switch (crossoverTargetPosition) {
    case 0:
      eventsToSend.event[1] = servoStateMachine->processStateTransition(servoStateMachine->crossoverArray[crossoverNumber].firstServo, MOVE_TO_POSITION_1);
      break;
    case 1:
      eventsToSend.event[1]= servoStateMachine->processStateTransition(servoStateMachine->crossoverArray[crossoverNumber].firstServo, MOVE_TO_POSITION_2);
      break;
    case 2:
      eventsToSend.event[1] = servoStateMachine->processStateTransition(servoStateMachine->crossoverArray[crossoverNumber].firstServo, MOVE_TO_POSITION_3);
      break;
  }
  // Send any events generated by the individual servo movement. This ensure that frog switching is performed.
  // if (eventToSend != -1) {
  //   //OpenLcb.produce(eventToSend);
  // }

  // Start the second servo moving to the target position.
  switch (crossoverTargetPosition) {
    case 0:
      eventsToSend.event[2] = servoStateMachine->processStateTransition(servoStateMachine->crossoverArray[crossoverNumber].secondServo, MOVE_TO_POSITION_1);
      break;
    case 1:
      eventsToSend.event[2] = servoStateMachine->processStateTransition(servoStateMachine->crossoverArray[crossoverNumber].secondServo, MOVE_TO_POSITION_2);
      break;
    case 2:
      eventsToSend.event[2] = servoStateMachine->processStateTransition(servoStateMachine->crossoverArray[crossoverNumber].secondServo, MOVE_TO_POSITION_3);
      break;
  }
  // Send any events generated by the individual servo movement. This ensure that frog switching is performed.
  // if (eventToSend != -1) {
  //   //OpenLcb.produce(eventToSend);
  // }

    // Set a flag so that the move completed states are monitored for both servos reaching their target positions.
  // This is checked in the servoStateMachine.update() method.
  // When both servos have reached their target position the crossover's reached event will be sent.
  switch (crossoverTargetPosition) {
    case 0:
      servoStateMachine->crossoverArray[crossoverNumber].waitingForPosition = AT_POSITION_1;
      break;
    case 1:
      servoStateMachine->crossoverArray[crossoverNumber].waitingForPosition = AT_POSITION_2;
      break;
    case 2:
      servoStateMachine->crossoverArray[crossoverNumber].waitingForPosition = AT_POSITION_3;
      break;
  }

  return eventsToSend;
}
