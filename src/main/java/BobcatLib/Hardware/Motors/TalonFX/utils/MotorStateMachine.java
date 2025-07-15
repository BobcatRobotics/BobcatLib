package BobcatLib.Hardware.Motors.TalonFX.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;

/**
 * Represents a state machine to monitor and track the operational state of a TalonFX motor.
 */
public class MotorStateMachine {

    /**
     * Represents the possible states of the motor.
     */
    public enum MotorState {
        /** Motor is not active */
        IDLE,
        /** Motor is spinning forward */
        FORWARD,
        /** Motor is spinning in reverse */
        REVERSE,
        /** State is indeterminate */
        UNKNOWN,
        /** An error or invalid transition occurred */
        ERROR,
        /** Motor is energized but not rotating as expected */
        STALL
    }

    private MotorState currentState;

    /**
     * Constructs a new {@code MotorStateMachine} starting in the {@code IDLE} state.
     */
    public MotorStateMachine() {
        this.currentState = MotorState.IDLE;
    }

    /**
     * Returns the current state of the motor.
     *
     * @return the current {@link MotorState}
     */
    public MotorState getCurrentState() {
        return currentState;
    }

    /**
     * Updates the state machine based on the provided TalonFX motor's output status and voltage
     * readings.
     *
     * @param motor the {@link TalonFX} motor to monitor
     */
    public void updateState(TalonFX motor) {
        MotorOutputStatusValue status = motor.getMotorOutputStatus().getValue();

        switch (status) {
            case Unknown:
                transitionTo(MotorState.UNKNOWN);
                break;

            case Off:
                transitionTo(MotorState.IDLE);
                break;

            case Motoring:
            case DiscordantMotoring:
                double voltage = motor.getMotorVoltage().getValueAsDouble();

                if (voltage == 0) {
                    transitionTo(MotorState.IDLE);
                } else if (voltage < 0) {
                    transitionTo(MotorState.REVERSE);
                } else if (voltage > 0) {
                    transitionTo(MotorState.FORWARD);
                } else {
                    transitionTo(MotorState.STALL);
                }
                break;

            default:
                transitionTo(MotorState.ERROR);
                break;
        }
    }

    /**
     * Transitions the state machine to the specified new state if the transition is valid. If the
     * transition is invalid, it enters the {@code ERROR} state.
     *
     * @param newState the {@link MotorState} to transition to
     */
    public void transitionTo(MotorState newState) {
        if (isValidTransition(currentState, newState)) {
            currentState = newState;
        } else {
            currentState = MotorState.ERROR;
        }
    }

    /**
     * Determines whether a transition between two states is allowed.
     *
     * @param from the current state
     * @param to the target state
     * @return {@code true} if the transition is valid; {@code false} otherwise
     */
    private boolean isValidTransition(MotorState from, MotorState to) {
        switch (from) {
            case IDLE:
                return to == MotorState.FORWARD || to == MotorState.REVERSE
                        || to == MotorState.UNKNOWN || to == MotorState.STALL;

            case FORWARD:
            case REVERSE:
            case STALL:
            case UNKNOWN:
                return to == MotorState.IDLE || to == MotorState.ERROR;

            case ERROR:
                return to == MotorState.IDLE;

            default:
                return false;
        }
    }
}
