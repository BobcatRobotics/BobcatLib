package BobcatLib.Hardware.Motors.TalonFX;

import BobcatLib.Hardware.Configurators.TalonFXConfigApplier;
import BobcatLib.Hardware.Motors.TalonFX.utils.MotorBuilder;
import BobcatLib.Hardware.Motors.TalonFX.utils.MotorInterface;
import BobcatLib.Hardware.Motors.TalonFX.utils.MotorStateMachine;
import BobcatLib.Hardware.Motors.TalonFX.utils.MotorValues;
import BobcatLib.Utils.CANDeviceDetails;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

/**
 * The main TalonFX Class that should be used in code.
 */
public class FalconMotor extends MotorBase implements MotorInterface {
    private MotorStateMachine state;

    /**
     * @param details the can device details and hardware details
     * @param motor MotorBuilder instance that defines the config
     */
    public FalconMotor(CANDeviceDetails details, MotorBuilder motor) {
        super(details, motor.build());

        configure();

        // Initialize status signals
        position = super.getPosition();
        velocity = super.getVelocity();
        acceleration = super.getAcceleration();
        volts = super.getMotorVoltage();
        amps = super.getStatorCurrent();

        state.updateState(this);
    }

    /**
     * Configures the TalonFX with factory defaults and applies the given configuration. Also sets
     * CAN signal update frequencies and optimizes bus utilization.
     */
    @Override
    public void configure() {
        BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration);
        BaseStatusSignal.setUpdateFrequencyForAll(50, volts, amps);

        ParentDevice.optimizeBusUtilizationForAll(this);

        TalonFXConfigApplier.applyFactoryDefault(this);
        TalonFXConfigApplier.apply(this, getConfig());
    }

    /**
     * Updates the {@link MotorValues} with the latest position, velocity, acceleration, voltage,
     * and current values from the motor.
     *
     * @param values Object to populate with the latest signal data.
     */
    @Override
    public void getUpdatedVals(MotorValues values) {
        BaseStatusSignal.refreshAll(position, velocity, acceleration, volts, amps);

        values.posRotations = position.getValueAsDouble();
        values.velRotationsPerSec = velocity.getValueAsDouble();
        values.accRotationsPerSecPerSec = acceleration.getValueAsDouble();
        values.motorVolts = volts.getValueAsDouble();
        values.motorAmps = amps.getValueAsDouble();
    }

    /**
     * Periodically called method that updates the motor with the current velocity setpoint using
     * DutyCycle control.
     */
    @Override
    public void periodic() {
        state.updateState(this);
    }
}
