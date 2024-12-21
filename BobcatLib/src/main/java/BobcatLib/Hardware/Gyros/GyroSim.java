package BobcatLib.Hardware.Gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroSim implements GyroIO {
  /** Main timer to control movement estimations. */
  private Timer timer;
  /** The last time the timer was read, used to determine position changes. */
  private double lastTime;
  /** Heading of the robot. */
  private double angle;

  public GyroSim() {
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
    configGyro();
  }

  public void configGyro() {}

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(getYaw());
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public void setYaw(double yaw) {
    this.angle = yaw;
  }

  public void periodic(Pigeon2 imu) {}

  public void periodic(AHRS imu) {}

  public void periodic() {}

  private double getYaw() {
    return angle;
  }

  public double getTimeDiff() {
    return (timer.get() - lastTime);
  }
}
