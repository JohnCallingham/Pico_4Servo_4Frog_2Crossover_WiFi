#ifndef StateMachine_h
#define StateMachine_h

#include "Global.h"

/**
 * Represents one transition.
 */
struct StateTransition {
    // Constructor to allow this struct to be populated.
    StateTransition(State currentState,
                    Action action,
                    int eventReached,
                    int eventLeaving,
                    int setServoTargetPosition,
                    State newState) {
        this->currentState = currentState;
        this->action = action;
        this->eventReached = eventReached;
        this->eventLeaving = eventLeaving;
        this->setServoTargetPosition = setServoTargetPosition;
        this->newState = newState;
    }

    // The elements of this struct.
    enum State currentState; // the current state for this transition
    enum Action action; // the action which causes the transition
    int eventReached; // the position (0 - 2) of the reached event to be sent, or -1 if none to be sent
    int eventLeaving; // the position (0 - 2) of the leaving event to be sent, or -1 if none to be sent
    int setServoTargetPosition; // the target position for the servo to start moving towards (0 - 2), or -1 if no movement required
    enum State newState; // the new state after this transition
};

/**
 * Represents one state machine with multiple state transitions.
 */
class StateMachine {
    public:
        /**
         * Adds one state transition to the state machine.
         * transition.eventReached is 1 to 3, or 0 if not needed.
         * transition.eventLeaving is 1 to 3, or 0 if not needed.
         * transition.setServoTargetPosition is 1 to 3, or 0 if not needed.
         */
        void addTransition(StateTransition transition);


        /**
         * Returns the StateTransition which matches currentState and action.
         * If no match then return a transition whose newState is UNKNOWN.
         */
        StateTransition findTransition(State currentState, Action action);
    
    private:
        /**
         * A vector which contains all valid transitions for this state machine.
         */
        std::vector<StateTransition> transitions;


};


#endif
