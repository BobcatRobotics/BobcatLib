package BobcatLib.Hardware.Motors;

import static BobcatLib.Hardware.PhoenixUtil.tryUntilOk;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


/**
 * Simulated implementation of the {@link MotorIO} interface. This class uses WPILib's
 * {@link DCMotorSim} to simulate real motor behavior, including position and velocity control via
 * PID.
 */
public class MotorIOSim implements MotorIO {

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

    // Simulation
    private final TalonFXSimState simState;


    private double simulatedPosition = 0.0;
    private double simulatedVelocity = 0.0;
    private double maxSimVelocity = 10.0; // rotations/sec
    private double maxAcceleration = 100.0; // RPS * RPS
    private final double SIM_LOOP_PERIOD_SEC = 0.02; // 20ms typical loop time


    /**
     * Constructs a TalonFX motor IO interface using the provided builder and CAN device details.
     *
     * @param builder Configuration builder with motor control settings.
     * @param device CAN ID and bus info for the TalonFX device.
     * @param usesTorqueCurrent Whether to use torque FOC mode instead of voltage.
     */
    public MotorIOSim(MotorBuilder builder, CANDeviceDetails device, boolean usesTorqueCurrent) {
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

        simState = motor.getSimState();
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
        simState.setRotorVelocity(output);
    }

    /**
     * Sets the motor to open-loop position control using duty cycle output.
     *
     * @param output The desired output voltage (in range [-1.0, 1.0]).
     */
    public void setPositionOpenLoop(double output) {
        simState.setRawRotorPosition(output);
    }

    /**
     * Runs the motor in closed-loop velocity control mode using the internal velocity PID
     * controller.
     *
     * @param velocityRadPerSec The desired velocity in radians per second.
     */
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        // Simulate gradual acceleration to the target velocity
        double velocityError = velocityRotPerSec - simulatedVelocity;
        double accel = Math.signum(velocityError)
                * Math.min(Math.abs(velocityError), maxAcceleration * SIM_LOOP_PERIOD_SEC);
        simulatedVelocity += accel;

        // Update position based on simulated velocity
        simulatedPosition += simulatedVelocity * SIM_LOOP_PERIOD_SEC;

        // Feed simulated values to the motor's sim state
        simState.setRotorVelocity(simulatedVelocity);
        simState.setRawRotorPosition(simulatedPosition);
        simState.setSupplyVoltage(12.0); // simulate battery voltage
    }

    /**
     * Runs the motor in closed-loop position control mode using the internal position PID
     * controller.
     *
     * @param rotation The desired target position as a {@link Rotation2d} object.
     */
    public void setPositionClosedLoop(Rotation2d rotation) {
        var rotations = rotation.getRotations();
        double error = rotations - simulatedPosition;
        double delta = Math.signum(error)
                * Math.min(Math.abs(error), maxSimVelocity * SIM_LOOP_PERIOD_SEC);

        simulatedPosition += delta;

        simState.setRawRotorPosition(simulatedPosition);
        simState.setRotorVelocity(delta / SIM_LOOP_PERIOD_SEC);
        simState.setSupplyVoltage(12.0); // simulate battery voltage
    }
}
