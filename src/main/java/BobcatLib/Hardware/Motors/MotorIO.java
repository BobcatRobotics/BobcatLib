package BobcatLib.Hardware.Motors;

import org.dyn4j.geometry.Rotation;
import org.littletonrobotics.junction.AutoLog;
import BobcatLib.Hardware.Motors.MotorStateMachine.MotorState;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * MotorIO defines a hardware abstraction interface for motor input-output operations.
 * It allows motor implementations to provide telemetry data through the {@link MotorIOInputs} class.
 */
public interface MotorIO {

  /**
   * Container class for motor telemetry and status inputs.
   * This class is automatically logged using the {@link AutoLog} annotation.
   */
  @AutoLog
  public static class MotorIOInputs {
    /** Whether the motor is connected and responsive. */
    public boolean connected = false;

    /** Position of the motor in radians. */
    public double positionRad = 0.0;

    /** Velocity of the motor in radians per second. */
    public double velocityRadPerSec = 0.0;

    /** Voltage applied to the motor in volts. */
    public double appliedVolts = 0.0;

    /** Current drawn by the motor in amps. */
    public double currentAmps = 0.0;

    /** Current state of the motor, as defined by {@link MotorState}. */
    public MotorState state = MotorState.IDLE;
  }

  /**
   * Update the motor input values. This method is called periodically to
   * refresh the telemetry data stored in {@link MotorIOInputs}.
   *
   * @param inputs The container to populate with the current motor telemetry data.
   */
  public default void updateInputs(MotorIOInputs inputs) {}

    /**
   * Runs the motor in closed-loop velocity control mode using the internal velocity PID controller.
   *
   * @param velocityRadPerSec The desired velocity in radians per second.
   */
  public default void setVelocityClosedLoop(double velocityRadPerSec) {}

    /**
   * Runs the motor in closed-loop position control mode using the internal position PID controller.
   *
   * @param rotation The desired target position as a {@link Rotation2d} object.
   */
  public default void setPositionClosedLoop(Rotation2d rotation) {}

}
