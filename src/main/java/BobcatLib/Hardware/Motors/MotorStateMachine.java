package BobcatLib.Hardware.Motors;

/**
 * A simple state machine for representing and managing the operational state of a motor.
 * States include IDLE, FORWARD, REVERSE, UNKNOWN, ERROR, and STALL.
 */
public class MotorStateMachine {

    /**
     * Enumeration of possible motor states.
     */
    public enum MotorState {
        IDLE,      // Motor is not moving
        FORWARD,   // Motor is moving in the forward direction
        REVERSE,   // Motor is moving in the reverse direction
        UNKNOWN,   // Initial or unrecognized state
        ERROR,     // Invalid transition or state error
        STALL      // Motor is stalled 
    }

    private MotorState currentState;

    /**
     * Creates a new MotorStateMachine with an initial state of {@code UNKNOWN}.
     */
    public MotorStateMachine() {
        this.currentState = MotorState.UNKNOWN;
    }

    /**
     * Gets the current state of the motor.
     *
     * @return The current {@link MotorState}.
     */
    public MotorState getCurrentState() {
        return currentState;
    }

    /**
     * Sets the state of the motor if the transition is valid.
     * Otherwise, the state is set to {@link MotorState#ERROR}.
     *
     * @param newState The desired new state.
     */
    public void setState(MotorState newState) {
        if (isValidTransition(newState)) {
            currentState = newState;
        } else {
            currentState = MotorState.ERROR;
        }
    }

    /**
     * Determines the motor state based on current draw and updates the state machine.
     *
     * @param current The measured motor current.
     * @return The updated {@link MotorState}.
     */
    public MotorState setMotorState(double current) {
        if (current == 0) {
            setState(MotorState.IDLE);
        } else if (current > 0) {
            setState(MotorState.FORWARD);
        } else if (current < 0) {
            setState(MotorState.REVERSE);
        } else {
            setState(MotorState.ERROR);
        }
        return getCurrentState();
    }

    /**
     * Validates whether a state transition is allowed from the current state.
     *
     * @param newState The state to transition to.
     * @return {@code true} if the transition is allowed, {@code false} otherwise.
     */
    private boolean isValidTransition(MotorState newState) {
        switch (currentState) {
            case UNKNOWN:
                return newState == MotorState.IDLE || newState == MotorState.ERROR;

            case IDLE:
                return newState == MotorState.FORWARD || newState == MotorState.REVERSE;

            case FORWARD:
                return newState == MotorState.IDLE || newState == MotorState.REVERSE;

            case REVERSE:
                return newState == MotorState.IDLE || newState == MotorState.FORWARD;

            case ERROR:
                return newState == MotorState.UNKNOWN;

            default:
                return false;
        }
    }
}
