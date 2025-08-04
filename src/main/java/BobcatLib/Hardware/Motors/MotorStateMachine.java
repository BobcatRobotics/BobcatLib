package BobcatLib.Hardware.Motors;

/**
 * A simple state machine for representing and managing the operational state of a motor.
 * <p>
 * This class tracks the state of the motor and allows transitions between predefined states:
 * {@code IDLE}, {@code FORWARD}, {@code REVERSE}, {@code UNKNOWN}, {@code ERROR}, and {@code STALL}.
 * <p>
 * It provides logic to validate state transitions and determine the current motor state based on current draw.
 */
public class MotorStateMachine {

    /**
     * Enumeration of possible motor operational states.
     */
    public enum MotorState {
        /** Motor is not moving or drawing current */
        IDLE,

        /** Motor is moving in the forward direction */
        FORWARD,

        /** Motor is moving in the reverse direction */
        REVERSE,

        /** Initial or undefined state */
        UNKNOWN,

        /** State machine encountered an invalid transition */
        ERROR,

        /** Motor is stalled (not currently settable via this implementation) */
        STALL
    }

    private MotorState currentState;

    /**
     * Constructs a new {@code MotorStateMachine} instance with an initial state of {@link MotorState#UNKNOWN}.
     */
    public MotorStateMachine() {
        this.currentState = MotorState.UNKNOWN;
    }

    /**
     * Returns the current state of the motor.
     *
     * @return The current {@link MotorState}.
     */
    public MotorState getCurrentState() {
        return currentState;
    }

    /**
     * Attempts to transition the motor to a new state.
     * <p>
     * If the transition is considered invalid based on the current state,
     * the motor state will be set to {@link MotorState#ERROR}.
     *
     * @param newState The desired next state.
     */
    public void setState(MotorState newState) {
        if (isValidTransition(newState)) {
            currentState = newState;
        } else {
            currentState = MotorState.ERROR;
        }
    }

    /**
     * Automatically updates the state of the motor based on the measured current draw.
     * <ul>
     *   <li>0 current {@link MotorState#IDLE}</li>
     *   <li>Positive current {@link MotorState#FORWARD}</li>
     *   <li>Negative current {@link MotorState#REVERSE}</li>
     * </ul>
     * Any invalid values result in {@link MotorState#ERROR}.
     *
     * @param current The measured motor current (amps).
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
     * Checks if a transition to the given state is valid from the current state.
     * <p>
     * This is a simple rule-based transition model designed to prevent invalid or unexpected changes.
     *
     * @param newState The target state.
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
