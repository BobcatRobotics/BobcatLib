package BobcatLib.Hardware.Motors.TalonFX.utils;

/**
 * Represents a generic control interface for CTRE motor controllers.
 * <p>
 * This interface defines methods that must be implemented by any class that manages motor control
 * behavior such as configuration, periodic updates, and retrieving control values.
 */
public interface MotorInterface {

  /**
   * Configures the motor controller for velocity control.
   * <p>
   * This may include setting PID constants, motion parameters, or sensor configuration as needed by
   * the implementing class.
   */
  public void configure();

  /**
   * Updates the given {@link MotorValues} object with the latest values from the motor controller.
   *
   * @param values The {@code ControlValues} instance to be updated with current controller state.
   */
  public void getUpdatedVals(MotorValues values);

  /**
   * Called once per control loop cycle to perform periodic updates.
   * <p>
   * This can include real-time control logic, feedback polling, or safety checks.
   */
  public void periodic();
}
