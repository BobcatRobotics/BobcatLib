package BobcatLib.Hardware.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import BobcatLib.Utils.PidControllerWrapper;

/**
 * A builder class for configuring and generating a {@link TalonFXConfiguration}
 * for CTRE TalonFX motor controllers.
 * <p>
 * This builder provides a fluent API for defining motor behavior including PID,
 * direction, feedback sensors, FOC, current limits, and software limits.
 * </p>
 */
public class MotorBuilder {

    /**
     * Type of control used for this motor.
     */
    public enum RequestType {
        /** Position control mode */
        POSITION,
        /** Velocity control mode */
        VELOCITY,
        /** Torque-based current control mode */
        TORQUE_CURRENT,
        /** No control mode set */
        NONE
    }

    /**
     * Type of feedback sensor used for closed-loop control.
     */
    public enum FeedbackSensorType {
        /** A remote CANcoder sensor */
        REMOTE_CANCODER,
        /** A fused CANcoder sensor */
        FUSED_CANCODER,
        /** A synchronized CANcoder sensor */
        SYNC_CANCODER,
        /** No feedback sensor */
        NONE
    }

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    // Configuration state
    private RequestType requestType;
    private FeedbackSensorType feedbackSensorType = FeedbackSensorType.NONE;
    private boolean isFOC;
    private SoftwareLimits forwardLimits;
    private SoftwareLimits reverseLimits;

    /**
     * Private constructor for required parameters.
     *
     * @param requestType         the control mode for the motor
     * @param neutralBrake        true if motor should brake when idle
     * @param ccwPositive         true if counter-clockwise is positive
     * @param motorToMechRatio    motor-to-mechanism gear ratio
     * @param statorCurrentLimit  stator current limit in amps
     * @param supplyCurrentLimit  supply current limit in amps
     * @param isFOC               true to enable Field-Oriented Control
     */
    private MotorBuilder(RequestType requestType, boolean neutralBrake, boolean ccwPositive,
                         double motorToMechRatio, double statorCurrentLimit,
                         double supplyCurrentLimit, boolean isFOC) {
        this.requestType = requestType;
        this.isFOC = isFOC;

        // Core config
        config.MotorOutput.NeutralMode =
                neutralBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        ccwPositive(ccwPositive);
        withStatorCurrentLimit(statorCurrentLimit);
        withSupplyCurrentLimit(supplyCurrentLimit);
        withToMechanismRatio(motorToMechRatio);
    }

    /**
     * Returns a new MotorBuilder with safe defaults.
     *
     * @return a {@link MotorBuilder} instance
     */
    public static MotorBuilder defaults() {
        return new MotorBuilder(RequestType.NONE, false, true,
                1.0, 80.0, 40.0, false);
    }

    /**
     * Adds a PID controller configuration using a {@link PidControllerWrapper}.
     *
     * @param pid the PID configuration
     * @return this builder
     */
    public MotorBuilder withPIDController(PidControllerWrapper pid) {
        if (pid != null) {
            config.Slot0 = pid.getSlot0Config();
        }
        return this;
    }

    /**
     * Updates the control request type.
     *
     * @param type the new {@link RequestType}
     * @return this builder
     */
    public MotorBuilder requestType(RequestType type) {
        this.requestType = type;
        return this;
    }

    /**
     * Sets the positive rotation direction.
     *
     * @param ccwPositive true for CCW positive, false for CW positive
     * @return this builder
     */
    public MotorBuilder ccwPositive(boolean ccwPositive) {
        config.MotorOutput.Inverted = ccwPositive
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        return this;
    }

    /**
     * Sets the gear ratio from the motor to the mechanism.
     * Only applies if a feedback sensor is configured.
     *
     * @param mechanismRatio the ratio to set
     * @return this builder
     */
    public MotorBuilder withToMechanismRatio(double mechanismRatio) {
        if (feedbackSensorType != FeedbackSensorType.NONE) {
            config.Feedback.SensorToMechanismRatio = mechanismRatio;
        }
        return this;
    }

    /**
     * Enables and sets the stator current limit.
     *
     * @param limit current limit in amps
     * @return this builder
     */
    public MotorBuilder withStatorCurrentLimit(double limit) {
        config.CurrentLimits.StatorCurrentLimit = limit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return this;
    }

    /**
     * Enables and sets the supply current limit.
     *
     * @param limit current limit in amps
     * @return this builder
     */
    public MotorBuilder withSupplyCurrentLimit(double limit) {
        config.CurrentLimits.SupplyCurrentLimit = limit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        return this;
    }

    /**
     * Adds software limits for forward motion.
     *
     * @param limits the software limit config
     * @return this builder
     */
    public MotorBuilder withForwardLimits(SoftwareLimits limits) {
        this.forwardLimits = limits;
        if (limits != null && limits.isEnabled()) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = limits.get();
        }
        return this;
    }

    /**
     * Adds software limits for reverse motion.
     *
     * @param limits the software limit config
     * @return this builder
     */
    public MotorBuilder withReverseLimits(SoftwareLimits limits) {
        this.reverseLimits = limits;
        if (limits != null && limits.isEnabled()) {
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = limits.get();
        }
        return this;
    }

    /**
     * Configures a feedback sensor.
     *
     * @param feedbackId the CAN ID of the sensor
     * @param type       the {@link FeedbackSensorType}
     * @return this builder
     */
    public MotorBuilder withFeedbackSensor(int feedbackId, FeedbackSensorType type) {
        this.feedbackSensorType = type;
        if (type != FeedbackSensorType.NONE) {
            config.Feedback.FeedbackRemoteSensorID = feedbackId;
            config.Feedback.FeedbackSensorSource = switch (type) {
                case REMOTE_CANCODER -> FeedbackSensorSourceValue.RemoteCANcoder;
                case FUSED_CANCODER -> FeedbackSensorSourceValue.FusedCANcoder;
                case SYNC_CANCODER -> FeedbackSensorSourceValue.SyncCANcoder;
                default -> FeedbackSensorSourceValue.FusedCANcoder;
            };
        }
        return this;
    }

    /**
     * Enables torque current limits if {@link RequestType#TORQUE_CURRENT} is selected.
     *
     * @param statorCurrentLimit the limit to apply
     * @return this builder
     */
    public MotorBuilder withTorqueCurrent(double statorCurrentLimit) {
        if (requestType == RequestType.TORQUE_CURRENT) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = statorCurrentLimit;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -statorCurrentLimit;
        }
        return this;
    }

    /**
     * Builds and returns the completed {@link TalonFXConfiguration}.
     *
     * @return the configuration instance
     */
    public TalonFXConfiguration build() {
        return config;
    }

    /** @return the control mode for this motor */
    public RequestType getRequestType() {
        return requestType;
    }

    /** @return true if FOC is enabled */
    public boolean isFOC() {
        return isFOC;
    }

    /** @return the forward software limits */
    public SoftwareLimits getForwardLimit() {
        return forwardLimits;
    }

    /** @return the reverse software limits */
    public SoftwareLimits getReverseLimit() {
        return reverseLimits;
    }
}
