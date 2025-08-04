package BobcatLib.Hardware.Motors;

import static BobcatLib.Hardware.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import BobcatLib.Hardware.Motors.Requests.PositionControl;
import BobcatLib.Hardware.Motors.Requests.VelocityControl;
import BobcatLib.Utils.CANDeviceDetails;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hardware I/O implementation for the CTRE TalonFX motor using the Phoenix 6 API.
 * <p>
 * This class handles motor input and output functionality, including telemetry updates and setting
 * control modes (open/closed-loop, voltage or torque-current).
 */
public class MotorIOTalonFX implements MotorIO {

  private final TalonFX motor;
  private final StatusSignal<Angle> relativePosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final TalonFXConfiguration config;
  private final MotorStateMachine state;

  // Control mode objects reused for performance
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final PositionControl positionControl = new PositionControl(0);
  private final VelocityControl velocityControl = new VelocityControl(0);

  /** Builder that defines this motor's behavior and control mode */
  public MotorBuilder builder;

  /**
   * Constructs a TalonFX motor IO interface using the provided builder and CAN device details.
   *
   * @param builder Configuration builder with motor control settings.
   * @param device CAN ID and bus info for the TalonFX device.
   * @param usesTorqueCurrent Whether to use torque FOC mode instead of voltage.
   */
  public MotorIOTalonFX(MotorBuilder builder, CANDeviceDetails device, boolean usesTorqueCurrent) {
    this.builder = builder;
    this.motor = new TalonFX(device.id(), device.bus());
    this.config = builder.build();

    positionControl.withTorqueFOC(usesTorqueCurrent);
    velocityControl.withTorqueFOC(usesTorqueCurrent);

    // Apply initial configuration with retry logic
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // Get references to sensor signals
    relativePosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    state = new MotorStateMachine();
    state.setMotorState(motorCurrent.getValueAsDouble());

    // Optimize CAN bus usage
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, relativePosition, motorVelocity,
        motorAppliedVolts, motorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  /**
   * Periodically updates input signals from the motor for use in system-level logic.
   *
   * @param inputs A container class used to store telemetry data from the motor.
   */
  @Override
  public void updateInputs(MotorIOInputs inputs) {
    var motorStatus = BaseStatusSignal.refreshAll(relativePosition, motorVelocity,
        motorAppliedVolts, motorCurrent);

    inputs.connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(relativePosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
    inputs.state = state.setMotorState(motorCurrent.getValueAsDouble());
  }

  /**
   * Sets the motor to open-loop velocity control using duty cycle output.
   *
   * @param output The desired output voltage (in range [-1.0, 1.0]).
   */
  public void setVelocityOpenLoop(double output) {
    dutyCycleOut.withOutput(output);
    motor.setControl(dutyCycleOut);
  }

  /**
   * Sets the motor to open-loop position control using duty cycle output.
   *
   * @param output The desired output voltage (in range [-1.0, 1.0]).
   */
  public void setPositionOpenLoop(double output) {
    dutyCycleOut.withOutput(output);
    motor.setControl(dutyCycleOut);
  }

  /**
   * Runs the motor in closed-loop velocity control mode using the internal velocity PID controller.
   *
   * @param velocityRadPerSec The desired velocity in radians per second.
   */
  public void setVelocityClosedLoop(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    var request = velocityControl.setRequest(velocityRotPerSec).withFOC(builder.isFOC());
    motor.setControl(request.getRequest());
  }

  /**
   * Runs the motor in closed-loop position control mode using the internal position PID controller.
   *
   * @param rotation The desired target position as a {@link Rotation2d} object.
   */
  public void setPositionClosedLoop(Rotation2d rotation) {
    var rotations = rotation.getRotations();
    var request = positionControl.setRequest(rotations).withFOC(builder.isFOC());
    motor.setControl(request.getRequest());
  }
}
