package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Logging.FaultsAndErrors.SparkMaxFaults;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class VortexMotor implements MotorIO {
  private int motorCanId = 0;
  private final SimpleMotorFeedforward motorFeedFordward;
  private CANSparkFlex mMotor;
  private RelativeEncoder encoder;
  private String busName = "";
  private SparkMaxFaults faults;

  public VortexMotor(int id, String busname, MotorConfigs config) {
    this.busName = busname;
    motorCanId = id;
    double motorKS = 0.00;
    double motorKV = 0.00;
    double motorKA = 0.00;
    motorFeedFordward = new SimpleMotorFeedforward(motorKS, motorKV, motorKA);
    /* Drive Motor Config */
    mMotor = new CANSparkFlex(id, MotorType.kBrushless);
    encoder = mMotor.getEncoder();
    configMotor(config);
    mMotor.burnFlash();
    encoder.setPosition(0.0);

    faults = new SparkMaxFaults(motorCanId);
  }

  public void configMotor(MotorConfigs cfg) {
    /** Motor Configuration */
    mMotor.restoreFactoryDefaults();

    /* Motor Inverts and Neutral Mode */
    mMotor.setInverted(new InvertedWrapper(cfg.isInverted).asREV());
    mMotor.setIdleMode(new NeutralModeWrapper(cfg.mode).asIdleMode());

    /* Gear Ratio Config */
    encoder.setVelocityConversionFactor(cfg.optionalRev.driveConversionVelocityFactor);
    encoder.setPositionConversionFactor(cfg.optionalRev.driveConversionPositionFactor);

    /* Current Limiting */
    mMotor.setSmartCurrentLimit(cfg.optionalRev.SupplyCurrentLimit);

    /* PID Config */
    SparkPIDController controller = mMotor.getPIDController();
    controller.setP(cfg.kP);
    controller.setI(cfg.kI);
    controller.setD(cfg.kD);
    controller.setFF(0);

    /* Open and Closed Loop Ramping */
    mMotor.setClosedLoopRampRate(cfg.optionalRev.closedLoopRamp);
    mMotor.setOpenLoopRampRate(cfg.optionalRev.openLoopRamp);

    /* Misc. Configs */
    mMotor.enableVoltageCompensation(12);
  }

  public void updateInputs(MotorIOInputs inputs) {
    inputs.motorPosition = getPosition();
    inputs.motorVelocity = getVelocity();
  }

  public Rotation2d getPosition() {
    double pos = encoder.getPosition();
    return Rotation2d.fromRotations(pos);
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void setSpeed(double speedInMPS) {
    double output = motorFeedFordward.calculate(speedInMPS);
    SparkPIDController controller = mMotor.getPIDController();
    controller.setReference(output, ControlType.kVelocity, 0);
  }

  /** Sets the Motor Control Speed */
  public void setSpeed(double speedInMPS, double mechanismCircumference) {
    double velocity = speedInMPS / mechanismCircumference;
    double output = motorFeedFordward.calculate(velocity);
    SparkPIDController controller = mMotor.getPIDController();
    controller.setReference(output, ControlType.kVelocity, 0);
  }

  public void setAngle(double angleInRotations) {
    SparkPIDController controller = mMotor.getPIDController();
    controller.setReference(angleInRotations, ControlType.kPosition, 0);
  }

  /** SysID Mode Methods * */
  public void setControl(double volts) {
    if (volts > 12) {
      volts = 12;
    }
    double output = volts / 12;
    mMotor.set(output);
  }

  public void stopMotor() {
    mMotor.stopMotor();
  }

  public void checkForFaults() {
    faults.hasFaultOccured();
  }
}
