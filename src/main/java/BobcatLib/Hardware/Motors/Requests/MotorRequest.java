package BobcatLib.Hardware.Motors.Requests;

/**
 * Interface for defining motor control request behaviors. Provides default methods for optional
 * features like PID, FOC, torque control, and motion profiling.
 */
public interface MotorRequest {

    /**
     * Placeholder for applying PID control parameters.
     */
    public default void withPID() {
        // Not implemented
    }

    /**
     * Enables or disables Field-Oriented Control (FOC) for the motor request.
     *
     * @param isEnabled True to enable FOC, false to disable.
     * @return A new MotorRequest with the FOC configuration.
     */
    public default MotorRequest withFOC(boolean isEnabled) {
        return new MotorRequest() {
            // Anonymous implementation stub
        };
    }

    /**
     * Enables or disables torque-based FOC mode.
     *
     * @param isTorqueFOC True to use torque FOC, false to use voltage control.
     * @return A new MotorRequest with the torque FOC configuration.
     */
    public default MotorRequest withTorqueFOC(boolean isTorqueFOC) {
        return new MotorRequest() {
            // Anonymous implementation stub
        };
    }

    /**
     * Configures a motion profile using maximum velocity and acceleration, and target constraints.
     *
     * @param maxVelocity Maximum allowed velocity.
     * @param maxAccel Maximum allowed acceleration.
     * @param finalTargetPosition Final target position to reach.
     * @param finalTargetVelocity Final target velocity to reach.
     * @return A new MotorRequest with the motion profile configuration.
     */
    public default MotorRequest withMotionProfile(double maxVelocity, double maxAccel,
            double finalTargetPosition, double finalTargetVelocity) {
        return new MotorRequest() {
            // Anonymous implementation stub
        };
    }
}
