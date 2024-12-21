package BobcatLib.Hardware.Gyros;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;

/** Represents an interface for interacting with a gyro sensor. */
public class NavXGyro implements GyroIO {
  private final AHRS gyro;

  public NavXGyro() {
    gyro = new AHRS(Port.kUSB);
    setYaw(0);
  }

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromRadians(getYaw());
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public void setYaw(double yaw) {
    gyro.zeroYaw();
  }

  private double getYaw() {
    return gyro.getRotation3d().getZ();
  }

  public void handleFaults() {}
}
