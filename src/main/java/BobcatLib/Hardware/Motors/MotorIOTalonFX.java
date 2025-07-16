package BobcatLib.Hardware.Motors;

import static BobcatLib.Hardware.PhoenixUtil.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** IO implementation for Pigeon 2. */
public class MotorIOTalonFX implements MotorIO {
  private final TalonFX motor = new TalonFX(0, "");
  // Inputs from turn motor
  private final StatusSignal<Angle> relativePosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final TalonFXConfiguration config;
  private final MotorStateMachine state;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);


  public boolean isTorqueCurrent = false;
  public boolean isFOC = false;


  private final Slot0Configs velocityGains;
  private final Slot0Configs positionGains;

  public MotorIOTalonFX(TalonFXConfiguration motorConfig, double statorCurrentLimit,
      boolean invertedState, double motorMechanismRatio, Slot0Configs motorPID, int feedbackId,
      FeedbackSensorSourceValue feedbackSensor, boolean isFeedbackEnabled, boolean isBrake,
      boolean isTorqueCurrent, boolean isFOC) {
    this.isTorqueCurrent = isTorqueCurrent;
    this.isFOC = isFOC;
    // Configure drive motor
    config = motorConfig;
    config.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Slot0 = motorPID;
    if (isFeedbackEnabled) {
      config.Feedback.FeedbackRemoteSensorID = feedbackId;
      config.Feedback.FeedbackSensorSource = switch (feedbackSensor) {
        case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
        case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
        case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
        default -> FeedbackSensorSourceValue.FusedCANcoder;
      };
      config.Feedback.SensorToMechanismRatio = motorMechanismRatio;
    }
    if (isTorqueCurrent) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = statorCurrentLimit;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -statorCurrentLimit;
    }
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        invertedState ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    // tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));

    // Create drive status signal
    relativePosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    state = new MotorStateMachine();
    state.setMotorState(motorCurrent.getValueAsDouble());
  
     // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        relativePosition,motorVelocity,motorAppliedVolts,motorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(motor);

  }


  @Override
  public void updateInputs(MotorIOInputs inputs) {
    // Update drive inputs
    var motorStatus = BaseStatusSignal.refreshAll(relativePosition, motorVelocity,
        motorAppliedVolts, motorCurrent);
    inputs.connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(relativePosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();

    inputs.state = state.setMotorState(motorCurrent.getValueAsDouble());
  }


  public void setVelocityOpenLoop(double output) {
    motor.setControl(
        switch (isTorqueCurrent) {
          case false -> voltageRequest.withOutput(output).withEnableFOC(isFOC);
          case true -> torqueCurrentRequest.withOutput(output);
        });
  }


  public void setPositionOpenLoop(double output) {
    motor.setControl(
        switch (isTorqueCurrent) {
          case false -> voltageRequest.withOutput(output);
          case true -> torqueCurrentRequest.withOutput(output);
        });
  }

  public void setVelocityClosedLoop(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    motor.setControl(
        switch (isTorqueCurrent) {
          case false -> velocityVoltageRequest
              .withVelocity(velocityRotPerSec)
              .withEnableFOC(isFOC);
          case true -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  public void setPositionClosedLoop(Rotation2d rotation) {
    motor.setControl(
        switch (isTorqueCurrent) {
          case false -> positionVoltageRequest.withPosition(rotation.getRotations());
          case true -> positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }

  public void setVelocityPIDandFF(double kp, double kd, double kv, double ka, double ks) {
    velocityGains.kP = kp;
    velocityGains.kD = kd;
    velocityGains.kV = kv;
    velocityGains.kA = ka;
    velocityGains.kS = ks;
    tryUntilOk(5, () -> motor.getConfigurator().apply(velocityGains, 0.25));
  }

  public void setPositionPIDandFF(double kp, double kd, double kv, double ka, double ks) {
    positionGains.kP = kp;
    positionGains.kD = kd;
    positionGains.kV = kv;
    positionGains.kA = ka;
    positionGains.kS = ks;
    tryUntilOk(5, () -> motor.getConfigurator().apply(velocityGains, 0.25));
  }
}
