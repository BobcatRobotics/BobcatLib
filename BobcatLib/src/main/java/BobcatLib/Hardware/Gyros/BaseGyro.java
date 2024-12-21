package BobcatLib.Hardware.Gyros;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class BaseGyro {
  private final GyroIO io;
  private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  public BaseGyro(GyroIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gyro", inputs);
  }

  public void setYaw(double yaw) {
    io.setYaw(yaw);
  }

  public Rotation2d getYaw() {
    return inputs.yawPosition;
  }

  public double getTimeDiff() {
    return io.getTimeDiff();
  }
}
