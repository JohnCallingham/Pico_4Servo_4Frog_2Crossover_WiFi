#include "StateMachine.h"

void StateMachine::addTransition(StateTransition transition) {
    // Allow positions to be entered as 1 to 3 but stored as 0 to 2.
    transitions.push_back(StateTransition(transition.currentState,
                                          transition.action,
                                          transition.eventReached-1,
                                          transition.eventLeaving-1,
                                          transition.setServoTargetPosition-1,
                                          transition.newState));


}

StateTransition StateMachine::findTransition(State currentState, Action action) {
    for (StateTransition transition : transitions) {
        if ((transition.currentState == currentState) && (transition.action == action)) {
            return transition;
        }
    }

    // This transition has not been found.
    // Return a transition whose newState is UNKNOWN to indicate an invalid transition.
    return StateTransition(currentState, action, 0, 0, 0, UNKNOWN);
}
