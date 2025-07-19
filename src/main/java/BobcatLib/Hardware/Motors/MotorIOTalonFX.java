package BobcatLib.Hardware.Motors;

import static BobcatLib.Hardware.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import BobcatLib.Utils.CANDeviceDetails;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * IO implementation for TalonFX motor using CTRE Phoenix 6 API.
 * Supports both voltage and torque-current control modes with open/closed-loop control.
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

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

  public MotorBuilder builder;

  /**
   * Constructs a TalonFX motor IO with settings from the given builder.
   * @param builder The configuration builder for this motor.
   */
  public MotorIOTalonFX(MotorBuilder builder, CANDeviceDetails device) {
    this.builder = builder;
    this.motor = new TalonFX(device.id(), device.bus());
    this.config = builder.build();

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    relativePosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    state = new MotorStateMachine();
    state.setMotorState(motorCurrent.getValueAsDouble());

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  /**
   * Updates the motor's input state for reading by higher-level systems.
   * @param inputs MotorIOInputs object to populate with sensor data.
   */
  @Override
  public void updateInputs(MotorIOInputs inputs) {
    var motorStatus = BaseStatusSignal.refreshAll(relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);
    inputs.connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(relativePosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
    inputs.state = state.setMotorState(motorCurrent.getValueAsDouble());
  }

  /**
   * Runs the motor in open-loop velocity mode.
   * @param output Output value between -1 and 1.
   */
  public void setVelocityOpenLoop(double output) {
    var request = builder.getRequestType() == MotorBuilder.RequestType.TORQUE_CURRENT
      ? torqueCurrentRequest.withOutput(output)
      : voltageRequest.withOutput(output).withEnableFOC(builder.isFOC());
    motor.setControl(request);
  }

  /**
   * Runs the motor in open-loop position mode.
   * @param output Output value between -1 and 1.
   */
  public void setPositionOpenLoop(double output) {
    var request = builder.getRequestType() == MotorBuilder.RequestType.TORQUE_CURRENT
      ? torqueCurrentRequest.withOutput(output)
      : voltageRequest.withOutput(output).withEnableFOC(builder.isFOC());
    motor.setControl(request);
  }

  /**
   * Runs the motor in closed-loop velocity mode using radians per second.
   * @param velocityRadPerSec Target velocity in radians per second.
   */
  public void setVelocityClosedLoop(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    var request = builder.getRequestType() == MotorBuilder.RequestType.TORQUE_CURRENT
      ? velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec)
      : velocityVoltageRequest.withVelocity(velocityRotPerSec).withEnableFOC(builder.isFOC());
    motor.setControl(request);
  }

  /**
   * Runs the motor in closed-loop position mode.
   * @param rotation Target rotation as a {@link Rotation2d} object.
   */
  public void setPositionClosedLoop(Rotation2d rotation) {
    var request = builder.getRequestType() == MotorBuilder.RequestType.TORQUE_CURRENT
      ? positionTorqueCurrentRequest.withPosition(rotation.getRotations())
      : positionVoltageRequest.withPosition(rotation.getRotations());
    motor.setControl(request);
  }

  /**
   * Dynamically updates the motor PID and feedforward gains.
   * @param kp Proportional gain.
   * @param kd Derivative gain.
   * @param kv Velocity feedforward.
   * @param ka Acceleration feedforward.
   * @param ks Static feedforward.
   */
  public void setMotorPIDandFF(double kp, double kd, double kv, double ka, double ks) {
    Slot0Configs pid = new Slot0Configs();
    pid.kP = kp;
    pid.kD = kd;
    pid.kV = kv;
    pid.kA = ka;
    pid.kS = ks;
    tryUntilOk(5, () -> motor.getConfigurator().apply(pid, 0.25));
  }
}
