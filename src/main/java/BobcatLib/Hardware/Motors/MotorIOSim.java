package BobcatLib.Hardware.Motors;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import BobcatLib.Hardware.Motors.MotorBuilder.RequestType;
import BobcatLib.Utils.CANDeviceDetails;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * Simulated implementation of the {@link MotorIO} interface.
 * This class uses WPILib's {@link DCMotorSim} to simulate real motor behavior,
 * including position and velocity control via PID.
 */
public class MotorIOSim implements MotorIO {

    // Simulation-only constants (not separated in TunerConstants)
    /** Drive PID proportional gain */
    public static final double DRIVE_KP = 0.0;

    /** Drive PID derivative gain */
    public static final double DRIVE_KD = 0.0;

    /** Static feedforward constant */
    public static final double DRIVE_KS = 0.1;

    /** Rotational velocity feedforward constant */
    public static final double DRIVE_KV_ROT = 0.0;

    /** Linear feedforward gain (volts per rad/s) */
    public static double DRIVE_KV = 0.77 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

    /** Turn PID proportional gain */
    public static final double TURN_KP = 8.0;

    /** Turn PID derivative gain */
    public static final double TURN_KD = 0.0;

    /** Gearbox for driving simulation */
    public static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);

    /** Gearbox for turning simulation */
    public static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private DCMotorSim driveSim;
    private DCMotorSim turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PIDController velocityController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController positionController = new PIDController(TURN_KP, 0, TURN_KD);

    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /** Whether the simulated motor is in position control mode */
    public boolean isPositionControl = false;

    private MotorBuilder builder;


  private final MotorStateMachine state;


    /**
     * Constructs a new MotorIOSim instance using the provided builder and device info.
     *
     * @param builder The {@link MotorBuilder} configuration for this motor
     * @param device  The CAN device details associated with this motor
     */
    public MotorIOSim(MotorBuilder builder, CANDeviceDetails device) {
        this.builder = builder;
        if (builder.getRequestType() == RequestType.POSITION) {
            this.isPositionControl = true;
        }
        driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_GEARBOX, 0.025, builder.build().Feedback.SensorToMechanismRatio),
            DRIVE_GEARBOX);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURN_GEARBOX, 0.004, builder.build().Feedback.SensorToMechanismRatio),
            TURN_GEARBOX);

        positionController.enableContinuousInput(-Math.PI, Math.PI);


        state = new MotorStateMachine();
        state.setMotorState(0);


    }

    /**
     * Constructs and initializes the sim motor for either velocity or position control based on {@code isPositionControl}.
     *
     * @param builder  The {@link MotorBuilder} used to configure the simulation
     * @param device   The CAN device being simulated
     * @param inertia  The moment of inertia of the mechanism
     */
    public MotorIOSim(MotorBuilder builder, CANDeviceDetails device, double inertia) {
        if (isPositionControl) {
            asPositionControl(inertia);
        } else {
            asVelocityControl(inertia);
        }

        state = new MotorStateMachine();
        state.setMotorState(0);
    }

    /**
     * Initializes this simulated motor for position control.
     *
     * @param inertia The inertia of the motor mechanism
     * @return this instance for method chaining
     */
    public MotorIOSim asPositionControl(double inertia) {
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, inertia, builder.build().Feedback.SensorToMechanismRatio),
                DRIVE_GEARBOX);
        return this;
    }

    /**
     * Initializes this simulated motor for velocity control.
     *
     * @param inertia The inertia of the motor mechanism
     * @return this instance for method chaining
     */
    public MotorIOSim asVelocityControl(double inertia) {
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, inertia, builder.build().Feedback.SensorToMechanismRatio),
                TURN_GEARBOX);
        positionController.enableContinuousInput(-Math.PI, Math.PI);
        return this;
    }

    /**
     * Periodically updates the motor inputs from the simulation state.
     *
     * @param inputs The container to populate with updated motor telemetry values
     */
    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // Run control loops
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + velocityController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            velocityController.reset();
        }

        if (turnClosedLoop) {
            turnAppliedVolts = positionController.calculate(turnSim.getAngularPositionRad());
        } else {
            positionController.reset();
        }

        // Simulate physics
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Populate output telemetry
        inputs.connected = true;
        inputs.positionRad = driveSim.getAngularPositionRad();
        inputs.velocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = driveAppliedVolts;
        inputs.currentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.state = state.setMotorState(inputs.currentAmps);
    }

    /**
     * Sets the motor output in open-loop (percent output) mode for velocity.
     *
     * @param output Raw output voltage (typically -12 to 12)
     */
    private void setVelocityOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    /**
     * Sets the motor output in open-loop mode for position control.
     *
     * @param output Raw output voltage
     */
    private void setPositionOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    /**
     * Enables closed-loop velocity control with the given setpoint.
     *
     * @param velocityRadPerSec Target velocity in radians per second
     */
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
        velocityController.setSetpoint(velocityRadPerSec);
    }

    /**
     * Enables closed-loop position control to reach the desired rotation.
     *
     * @param rotation Target rotation
     */
    public void setPositionClosedLoop(Rotation2d rotation) {
        turnClosedLoop = true;
        positionController.setSetpoint(rotation.getRadians());
    }

    /**
     * Sets PID and feedforward parameters based on the control mode.
     *
     * @param kp Proportional gain
     * @param kd Derivative gain
     * @param kv Velocity feedforward gain
     * @param ka Acceleration feedforward gain (unused)
     * @param ks Static feedforward gain (unused)
     */
    public void setMotorPIDandFF(double kp, double kd, double kv, double ka, double ks) {
        if (isPositionControl) {
            setPositionPIDandFF(kp, kd, kv, ka, ks);
        } else {
            setVelocityIDandFF(kp, kd, kv, ka, ks);
        }
    }

    /**
     * Sets velocity control PID and feedforward parameters.
     *
     * @param kp Proportional gain
     * @param kd Derivative gain
     * @param kv Velocity feedforward gain
     * @param ka Acceleration feedforward gain (unused)
     * @param ks Static feedforward gain (unused)
     */
    private void setVelocityIDandFF(double kp, double kd, double kv, double ka, double ks) {
        velocityController.setP(kp);
        velocityController.setD(kd);
        DRIVE_KV = kv;
    }

    /**
     * Sets position control PID and feedforward parameters.
     *
     * @param kp Proportional gain
     * @param kd Derivative gain
     * @param kv Velocity feedforward gain
     * @param ka Acceleration feedforward gain (unused)
     * @param ks Static feedforward gain (unused)
     */
    private void setPositionPIDandFF(double kp, double kd, double kv, double ka, double ks) {
        velocityController.setP(kp);
        velocityController.setD(kd);
        DRIVE_KV = kv;
    }
}
