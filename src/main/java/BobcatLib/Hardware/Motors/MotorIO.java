package BobcatLib.Hardware.Motors;

import org.littletonrobotics.junction.AutoLog;
import BobcatLib.Hardware.Motors.MotorStateMachine.MotorState;
  /**
   * Motor I/O Interface for logging some initial pieces of data.
   */
public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public MotorState state = MotorState.IDLE;
  }

  /**
   * @param inputs
   */
  public default void updateInputs(MotorIOInputs inputs) {}
}