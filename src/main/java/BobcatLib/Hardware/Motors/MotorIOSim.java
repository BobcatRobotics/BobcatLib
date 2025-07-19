package BobcatLib.Hardware.Motors;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.net.Authenticator.RequestorType;
import BobcatLib.Hardware.Motors.MotorBuilder.RequestType;
import BobcatLib.Utils.CANDeviceDetails;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class MotorIOSim implements MotorIO {
    // TunerConstants doesn't support separate sim constants, so they are declared locally
    public static final double DRIVE_KP = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV_ROT = 0.0; // Same units as TunerConstants: (volt *
                                                   // secs) / rotation
    public static double DRIVE_KV = 0.77 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    public static final double TURN_KP = 8.0;
    public static final double TURN_KD = 0.0;
    public static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
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

    public boolean isPositionControl = false;

    private MotorBuilder builder;

    public MotorIOSim(MotorBuilder builder, CANDeviceDetails device) {
        this.builder = builder;
        if( builder.getRequestType() == RequestType.POSITION){
            this.isPositionControl = true;
        }
        driveSim = null;
        turnSim = null;
        // Enable wrapping for turn PID
        positionController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public MotorIOSim(MotorBuilder builder, CANDeviceDetails device, double inertia) {
        if (isPositionControl) {
            asPositionControl(inertia);
        } else {
            asVelocityControl(inertia);
        }
    }

    public MotorIOSim asPositionControl(double inertia) {
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, inertia, builder.build().Feedback.SensorToMechanismRatio),
                DRIVE_GEARBOX);
        return this;
    }

    public MotorIOSim asVelocityControl(double inertia) {
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, inertia, builder.build().Feedback.SensorToMechanismRatio),
                TURN_GEARBOX);
        positionController.enableContinuousInput(-Math.PI, Math.PI);
        return this;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts
                    + velocityController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            velocityController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = positionController.calculate(turnSim.getAngularPositionRad());
        } else {
            positionController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Update drive inputs
        inputs.connected = true;
        inputs.positionRad = driveSim.getAngularPositionRad();
        inputs.velocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = driveAppliedVolts;
        inputs.currentAmps = Math.abs(driveSim.getCurrentDrawAmps());
    }

    private void setVelocityOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    private void setPositionOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    public void setVelocityClosedLoop(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
        velocityController.setSetpoint(velocityRadPerSec);
    }

    public void setPositionClosedLoop(Rotation2d rotation) {
        turnClosedLoop = true;
        positionController.setSetpoint(rotation.getRadians());
    }

    public void setMotorPIDandFF(double kp, double kd, double kv, double ka, double ks) {
        if (isPositionControl) {
            setPositionPIDandFF(kp, kd, kv, ka, ks);
        } else {
            setVelocityIDandFF(kp, kd, kv, ka, ks);
        }
    }

    private void setVelocityIDandFF(double kp, double kd, double kv, double ka, double ks) {
        velocityController.setP(kp);
        velocityController.setD(kd);
        DRIVE_KV = kv;
    }

    private void setPositionPIDandFF(double kp, double kd, double kv, double ka, double ks) {
        velocityController.setP(kp);
        velocityController.setD(kd);
        DRIVE_KV = kv;
    }
}
