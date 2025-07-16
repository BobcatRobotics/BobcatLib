package BobcatLib.Hardware;

public class MotorStateMachine {

    public enum MotorState {
        IDLE,
        FORWARD,
        REVERSE,
        UNKNOWN,
        ERROR
    }

    private MotorState currentState;

    public MotorStateMachine() {
        this.currentState = MotorState.UNKNOWN;
    }

    public MotorState getCurrentState() {
        return currentState;
    }

    public void setState(MotorState newState) {
        if (isValidTransition(newState)) {
            currentState = newState;
        } else {
            currentState = MotorState.ERROR;
        }
    }

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
