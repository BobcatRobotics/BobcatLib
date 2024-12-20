package BobcatLib.Hardware.Motors;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  /** Represents the inputs for the gyro sensor. */
  @AutoLog
  public static class MotorIOInputs {
    /** The current yaw position of the gyro. */
    public Rotation2d motorPosition = new Rotation2d();
    /** The current pitch position of the gyro. */
    public double motorVelocity = 0.00;
    /** The current roll position of the gyro. */
    public boolean faulted = false;
  }

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(MotorIOInputs inputs) {}

  public default double getTimeDiff() {
    return 1.0;
  }

  public default void periodic(KrakenMotor motor) {}

  public default void periodic(FalconMotor motor) {}

  public default void periodic() {}

  public default Rotation2d getPosition() {
    return new Rotation2d();
  }

  public default double getVelocity() {
    return 0;
  }

  public default void setSpeed(double speedInMPS) {}

  public default void setSpeed(double speedInMPS, double mechanismCircumference) {}

  public default void setAngle(double angleInRotations) {}

  public default void setControl(double volts) {}

  public default void stopMotor() {}
}
