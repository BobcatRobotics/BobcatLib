package BobcatLib.Hardware.Motors;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class BaseMotor {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  public BaseMotor(MotorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("motor", inputs);
  }

  public Rotation2d getPosition() {
    return io.getPosition();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public void stopMotor() {
    io.stopMotor();
  }

  /** Sets the Motor Control Speed */
  public void setSpeed(double speedInMPS, double mechanismCircumference) {
    io.setSpeed(speedInMPS, mechanismCircumference);
  }

  public void setAngle(double angleInRotations) {
    io.setAngle(angleInRotations);
  }

  /** SysID Mode Methods * */
  public void setControl(double volts) {
    io.setControl(volts);
  }
}
