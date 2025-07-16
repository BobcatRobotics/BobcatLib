package BobcatLib.Hardware;

import static BobcatLib.Hardware.PhoenixUtil.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import BobcatLib.Hardware.MotorStateMachine.MotorState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon 2. */
public class MotorIOFalcon implements MotorIO {
  private final TalonFX motor = new TalonFX(0, "");
  // Inputs from turn motor
  private final StatusSignal<Angle> relativePosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final TalonFXConfiguration config;
  private final MotorStateMachine state;

  public MotorIOFalcon(TalonFXConfiguration motorConfig, double statorCurrentLimit,
      boolean invertedState, double motorMechanismRatio, Slot0Configs motorPID, int feedbackId,
      FeedbackSensorSourceValue feedbackSensor, boolean isFeedbackEnabled, boolean isBrake) {
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

    setMotorState();
  }

  public void setMotorState() {
    if (motorCurrent.getValueAsDouble() == 0) {
      state.setState(MotorState.IDLE);
    } else if (motorCurrent.getValueAsDouble() > 0) {
      state.setState(MotorState.FORWARD);

    } else if (motorCurrent.getValueAsDouble() < 0) {
      state.setState(MotorState.REVERSE);
    } else {
      state.setState(MotorState.ERROR);
    }
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

    setMotorState();
    inputs.state = state.getCurrentState();
  }
}
