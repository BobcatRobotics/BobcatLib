package BobcatLib.Hardware.Gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Represents an interface for interacting with a gyro sensor. */
public interface GyroIO {
  /** Represents the inputs for the gyro sensor. */
  @AutoLog
  public static class GyroIOInputs {
    /** Indicates if the gyro is connected. */
    public boolean connected = false;
    /** The current yaw position of the gyro. */
    public Rotation2d yawPosition = new Rotation2d();
    /** Indicates if the gyro has a fault. */
    public boolean faulted = false;
  }

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  public default double getTimeDiff() {
    return 1.0;
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public default void setYaw(double yaw) {}

  public default void periodic(Pigeon2 imu) {}

  public default void periodic(AHRS imu) {}

  public default void periodic() {}
}
