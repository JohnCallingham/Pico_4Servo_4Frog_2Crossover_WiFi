#ifndef ConsumedEvents_h
#define ConsumedEvents_h
/**
 * Helper functions for handling consumed events.
 * The entry function is handleConsumedEvent() which is
 * called from pceCallback().
 */

#include <Arduino.h>
#include "Global.h"
#include "ServoStateMachine.h"

void initialiseOutputPins();

/***
 * Determines whether the event is for servos, outputs or crossovers and handles accordingly.
 * Returns the indexes of up to 3 events which are to be sent.
 */
EventsToSend handleConsumedEvent(uint16_t index, ServoStateMachine* servoStateMachine);

EventsToSend handleServoEvent(uint16_t index, ServoStateMachine* servoStateMachine);
EventsToSend handleOutputEvent(uint16_t index, ServoStateMachine* servoStateMachine);
EventsToSend handleCrossoverEvent(uint16_t index, ServoStateMachine* servoStateMachine);

EventsToSend moveCrossoverToTargetPosition(uint8_t crossoverNumber,
                                   uint8_t crossoverTargetPosition,
                                   ServoStateMachine* servoStateMachine);

#endif
