package BobcatLib.Hardware.Motors;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Represents a general-purpose base class for motor controllers,
 * handling logging, alerting, and access to closed-loop control methods.
 */
public class MotorBase {
    private MotorIO io;
    private MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
    private final Alert motorDisconnectedAlert =
            new Alert("motor disconnected!", AlertType.kWarning);
    private final String name;

    /**
     * Constructs a new MotorBase instance.
     *
     * @param io The {@link MotorIO} implementation for controlling hardware.
     * @param name The name of the motor (used for logging).
     */
    public MotorBase(MotorIO io, String name) {
        this.io = io;
        this.name = name;
    }

    /**
     * Periodically updates motor inputs and logs them,
     * while checking and displaying connection status alerts.
     */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        motorDisconnectedAlert.set(!inputs.connected);
    }

    /**
     * Runs the motor in closed-loop velocity control mode using the internal velocity PID
     * controller.
     *
     * @param velocityRadPerSec The desired velocity in radians per second.
     */
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        io.setVelocityClosedLoop(velocityRadPerSec);
    }

    /**
     * Runs the motor in closed-loop position control mode using the internal position PID
     * controller.
     *
     * @param rotation The desired target position as a {@link Rotation2d} object.
     */
    public void setPositionClosedLoop(Rotation2d rotation) {
        io.setPositionClosedLoop(rotation);
    }

    /**
     * Returns the current velocity of the motor in radians per second.
     *
     * @return The velocity in rad/s.
     */
    public double getVelocity() {
        return inputs.velocityRadPerSec;
    }

    /**
     * Returns the current position of the motor as a {@link Rotation2d} object.
     *
     * @return The motor position.
     */
    public Rotation2d getPosition() {
        return Rotation2d.fromRadians(inputs.positionRad);
    }
}
