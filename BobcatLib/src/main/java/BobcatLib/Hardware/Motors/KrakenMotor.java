package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Logging.FaultsAndErrors.TalonFXFaults;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class KrakenMotor implements MotorIO {
  private int motorCanId = 0;
  private final SimpleMotorFeedforward motorFeedFordward;
  private TalonFXConfiguration motorConfig;
  private TalonFX mMotor;
  private String busName = "";
  private final DutyCycleOut motorDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage motorVelocity = new VelocityVoltage(0);
  private final PositionVoltage motorPositionVoltage = new PositionVoltage(0);
  private TalonFXFaults faults;

  public KrakenMotor(int id, String busname, MotorConfigs config) {
    this.busName = busname;
    motorCanId = id;
    double motorKS = 0.00;
    double motorKV = 0.00;
    double motorKA = 0.00;
    motorFeedFordward = new SimpleMotorFeedforward(motorKS, motorKV, motorKA);
    mMotor = new TalonFX(id, busName);
    configMotor(config);
    mMotor.getConfigurator().apply(motorConfig);
  }

  public void configMotor(MotorConfigs cfg) {
    motorConfig.MotorOutput.Inverted = new InvertedWrapper(cfg.isInverted).asCTRE();
    motorConfig.MotorOutput.NeutralMode = new NeutralModeWrapper(cfg.mode).asNeutralModeValue();
    motorConfig.Feedback.SensorToMechanismRatio = cfg.motorToGearRatio;
    motorConfig.Slot0 = new Slot0Configs();
    motorConfig.Slot0.kP = cfg.kP;
    motorConfig.Slot0.kP = cfg.kI;
    motorConfig.Slot0.kP = cfg.kD;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = cfg.optionalCtre.SupplyCurrentLimitEnable;
    motorConfig.CurrentLimits.SupplyCurrentLimit = cfg.optionalCtre.SupplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = cfg.optionalCtre.SupplyCurrentThreshold;
    motorConfig.CurrentLimits.SupplyTimeThreshold = cfg.optionalCtre.SupplyTimeThreshold;
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        cfg.optionalCtre.open_DutyCycleOpenLoopRampPeriod;
    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        cfg.optionalCtre.open_VoltageOpenLoopRampPeriod;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        cfg.optionalCtre.closed_DutyCycleOpenLoopRampPeriod;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        cfg.optionalCtre.closed_VoltageOpenLoopRampPeriod;
  }

  public void updateInputs(MotorIOInputs inputs) {
    inputs.motorPosition = getPosition();
    inputs.motorVelocity = getVelocity();
  }

  public Rotation2d getPosition() {
    double pos = mMotor.getPosition().getValueAsDouble();
    return Rotation2d.fromRotations(pos);
  }

  public double getVelocity() {
    return mMotor.getVelocity().getValueAsDouble();
  }

  public void setSpeed(double speedInMPS, boolean isOpenLoop) {
    motorDutyCycle.Output = speedInMPS;
    mMotor.setControl(motorDutyCycle);
  }

  /** Sets the Motor Control Speed */
  public void setSpeed(double speedInMPS, double mechanismCircumference, boolean isOpenLoop) {
    motorVelocity.Velocity = speedInMPS / mechanismCircumference;
    motorVelocity.FeedForward = motorFeedFordward.calculate(speedInMPS);
    mMotor.setControl(motorVelocity);
  }

  public void setAngle(double angleInRotations) {
    mMotor.setControl(motorPositionVoltage.withPosition(angleInRotations));
  }

  /** SysID Mode Methods * */
  public void setControl(double volts) {
    VoltageOut sysidControl = new VoltageOut(0);
    sysidControl.withOutput(volts);
    mMotor.setControl(sysidControl);
  }

  public void stopMotor() {
    mMotor.stopMotor();
  }

  public void checkForFaults() {
    faults.hasFaultOccured();
  }
}
