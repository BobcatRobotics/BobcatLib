package BobcatLib.Hardware.Motors.TalonFX;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import BobcatLib.Utils.CANDeviceDetails;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Base class for CTRE motor control signal tracking.
 * <p>
 * This class holds protected {@link StatusSignal} references for common motor telemetry, including
 * position, velocity, acceleration, voltage, and current. It is intended to be extended by motor
 * control implementations.
 */
public class MotorBase extends TalonFX {

    /** Configuration object to apply to the motor. */
    private final TalonFXConfiguration config;

    /** CAN bus device ID and details. */
    private final CANDeviceDetails details;

    /** Angular position of the motor, typically in rotations or degrees. */
    protected StatusSignal<Angle> position;

    /** Angular velocity of the motor, typically in rotations per second. */
    protected StatusSignal<AngularVelocity> velocity;

    /** Angular acceleration of the motor, typically in rotations per second squared. */
    protected StatusSignal<AngularAcceleration> acceleration;

    /** Voltage applied to the motor. */
    protected StatusSignal<Voltage> volts;

    /** Current drawn by the motor. */
    protected StatusSignal<Current> amps;

    /**
     * Constructs an empty {@code MotorTelemetry}.
     * <p>
     * Fields should be assigned in derived classes based on specific motor configuration.
     * @param details device details
     * @param config device configuration
     */
    public MotorBase(CANDeviceDetails details, TalonFXConfiguration config) {
        super(details.id(), details.bus());
        // Initialization is deferred to subclasses.
        this.details = details;
        this.config = config;
    }

    /**
     * @return device config
     */
    public TalonFXConfiguration getConfig() {
        return config;
    }

    /**
     * @return device info
     */
    public CANDeviceDetails getDeviceInfo() {
        return details;
    }
}
