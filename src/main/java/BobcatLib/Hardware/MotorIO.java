package BobcatLib.Hardware;

import org.littletonrobotics.junction.AutoLog;
import BobcatLib.Hardware.MotorStateMachine.MotorState;

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

  public default void updateInputs(MotorIOInputs inputs) {}
}