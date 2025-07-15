package org.Hardware.CTRE.Motors.Controls;

import org.Hardware.CTRE.Motors.Controls.ControlValues;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public interface ControlInterface {

  /** Configures the velocity controller */
  public void configure();

  /**
   * Get the velocity controller's updated values and update the provided values
   * class accordingly
   *
   * @param values values class to be updated
   */
  public void getUpdatedVals(ControlValues values);

  /**
   * Sets the AngularVelocity RotationsPerSecond
   *
   * @param angularVelocityInRotationsPerSecond angle in rotations per second
   */
  public void setPos(AngularVelocity angularVelocityInRotationsPerSecond);

  /**
   * Sets the voltage of the controller
   *
   * @param posRotations voltage In Volts
   */
  public void setPos(Voltage voltageInVolts);

  /**
   * Sets the current in amps of the controller
   *
   * @param posRotations current in amps
   */
  public void setPos(Current currentInAmps);

  /**
   * Sets the angle in rotations of the controller
   *
   * @param angleInRotations angle in Rotations
   */
  public void setPos(Angle angleInRotations);

  /**
   * Sets the Jerk in Velocity SecondsCubed
   *
   * @param angleInRotations Jerk in VelocitySecondCubed
   */
  public void setPos(
      Velocity<AngularAccelerationUnit> jerkInVelocitySecondsCubed);

  /**
   * Sets the setpoint angular velocity in RotationsPerSecond
   *
   * @param angularVelocityInRotationsPerSecond AngularVelocity in
   *                                            RotationsPerSecond
   */
  public void setSetpoint(AngularVelocity angularVelocityInRotationsPerSecond);

  /**
   * Sets the setpoint Voltage in Volts
   *
   * @param voltageInVolts Voltage in Volts
   */
  public void setSetpoint(Voltage voltageInVolts);

  /**
   * Sets the setpoint current in amps
   *
   * @param voltageInVolts current in amps
   */
  public void setSetpoint(Current currentInAmps);

  /**
   * Sets the setpoint angle in rotations
   *
   * @param angleInRotations angle in rotations
   */
  public void setSetpoint(Angle angleInRotations);

  /**
   * Sets the setpoint jerk in VelocitySecondsCubed
   *
   * @param angleInRotations jerk in VelocitySecondsCubed
   */
  public void setSetpoint(Velocity<AngularAccelerationUnit> jerkInVelocitySecondsCubed);

  /** Called every periodic loop */
  public void periodic();
}
