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
    checkForFaults();
  }

  /** Sets the Motor Control Speed */
  public void setSpeed(double speedInMPS, double mechanismCircumference, boolean isOpenLoop) {
    io.setSpeed(speedInMPS, mechanismCircumference, isOpenLoop);
    checkForFaults();
  }

  public void setAngle(double angleInRotations) {
    io.setAngle(angleInRotations);
    checkForFaults();
  }

  /** SysID Mode Methods * */
  public void setControl(double volts) {
    io.setControl(volts);
    checkForFaults();
  }

  public void checkForFaults() {
    io.checkForFaults();
  }
}
