package BobcatLib.Hardware.Motors;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class MotorBase {
    private MotorIO io;
    private MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
    private final Alert motorDisconnectedAlert =
            new Alert("motor disconnected!", AlertType.kWarning);
    private final String name;

    public MotorBase(MotorIO io, String name) {
        this.io = io;
        this.name = name;
    }

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

    public double getVelocity(){
        return inputs.velocityRadPerSec;
    }

    public Rotation2d getPosition(){
        return  Rotation2d.fromRadians(inputs.positionRad);
    }
}
