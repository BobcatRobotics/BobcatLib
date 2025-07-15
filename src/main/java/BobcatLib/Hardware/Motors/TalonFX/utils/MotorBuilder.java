package BobcatLib.Hardware.Motors.TalonFX.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * A builder class for creating a configured {@link TalonFXConfiguration}
 * based on custom motor parameters such as current limits, sensor ratios,
 * neutral modes, and software limit switches.
 */
public class MotorBuilder {

    /**
     * Internal configuration class for storing motor parameters.
     */
    private class MotorConfig {

        /**
         * Represents forward or reverse software limit configuration.
         */
        public class SoftwareLimits {
            private double limit;
            private boolean enable;

            /**
             * Constructs disabled software limits.
             */
            public SoftwareLimits() {
                disable();
            }

            /**
             * Constructs enabled software limits with a threshold.
             *
             * @param limit the limit threshold in sensor units
             */
            public SoftwareLimits(double limit) {
                this.limit = limit;
                enable();
            }

            /**
             * Disables the software limit.
             */
            public void disable() {
                this.enable = false;
            }

            /**
             * Enables the software limit.
             */
            public void enable() {
                this.enable = true;
            }
        }

        private SoftwareLimits forwardLimits;
        private SoftwareLimits reverseLimits;
        private boolean neutralBrake;
        private boolean ccwPositive;
        private double motorToMechRatio;
        private double statorCurrentLimit;
        private double supplyCurrentLimit;

        /**
         * Constructs a {@code MotorConfig} with all configuration options.
         *
         * @param neutralBrake        whether brake mode is enabled
         * @param ccwPositive         motor inversion direction
         * @param motorToMechRatio    sensor-to-mechanism ratio
         * @param statorCurrentLimit  stator current limit (amps)
         * @param supplyCurrentLimit  supply current limit (amps)
         */
        private MotorConfig(boolean neutralBrake, boolean ccwPositive, double motorToMechRatio,
                            double statorCurrentLimit, double supplyCurrentLimit) {
            neutralBrake(neutralBrake);
            ccwPositive(ccwPositive);
            motorToMechRatio(motorToMechRatio);
            statorCurrentLimit(statorCurrentLimit);
            supplyCurrentLimit(supplyCurrentLimit);
        }

        /**
         * Sets the neutral brake mode.
         *
         * @param neutralBrake true for brake, false for coast
         * @return this config instance
         */
        public MotorConfig neutralBrake(boolean neutralBrake) {
            this.neutralBrake = neutralBrake;
            return this;
        }

        /**
         * Sets the inversion direction.
         *
         * @param ccwPositive true if clockwise is positive
         * @return this config instance
         */
        public MotorConfig ccwPositive(boolean ccwPositive) {
            this.ccwPositive = ccwPositive;
            return this;
        }

        /**
         * Sets the sensor-to-mechanism ratio.
         *
         * @param motorToMechRatio the ratio to use
         * @return this config instance
         */
        public MotorConfig motorToMechRatio(double motorToMechRatio) {
            this.motorToMechRatio = motorToMechRatio;
            return this;
        }

        /**
         * Sets the stator current limit in amps.
         *
         * @param statorCurrentLimit stator limit
         * @return this config instance
         */
        public MotorConfig statorCurrentLimit(double statorCurrentLimit) {
            this.statorCurrentLimit = statorCurrentLimit;
            return this;
        }

        /**
         * Sets the supply current limit in amps.
         *
         * @param supplyCurrentLimit supply limit
         * @return this config instance
         */
        public MotorConfig supplyCurrentLimit(double supplyCurrentLimit) {
            this.supplyCurrentLimit = supplyCurrentLimit;
            return this;
        }

        /**
         * Sets forward software limit parameters.
         *
         * @param limits the forward limits to apply
         * @return this config instance
         */
        public MotorConfig withForwardLimits(SoftwareLimits limits) {
            this.forwardLimits = limits;
            return this;
        }

        /**
         * Sets reverse software limit parameters.
         *
         * @param limits the reverse limits to apply
         * @return this config instance
         */
        public MotorConfig withReverseLimits(SoftwareLimits limits) {
            this.reverseLimits = limits;
            return this;
        }
    }

    private final MotorConfig cfg;
    private final TalonFXConfiguration config;

    /**
     * Constructs a {@code MotorBuilder} with a provided {@link MotorConfig}.
     *
     * @param cfg the motor configuration settings
     */
    private MotorBuilder(MotorConfig cfg) {
        defaults();  // this seems redundant; see note below
        this.cfg = cfg;
        this.config = new TalonFXConfiguration();
    }

    /**
     * Returns a {@code MotorBuilder} instance with default parameters:
     * <ul>
     *     <li>Neutral brake disabled</li>
     *     <li>Clockwise positive</li>
     *     <li>1.0 sensor-to-mechanism ratio</li>
     *     <li>80A stator current limit</li>
     *     <li>40A supply current limit</li>
     * </ul>
     *
     * @return a default-configured {@code MotorBuilder}
     */
    public MotorBuilder defaults() {
        return new MotorBuilder(new MotorConfig(false, true, 1.0, 80.0, 40.0));
    }

    /**
     * Builds and returns a {@link TalonFXConfiguration} based on this builder's settings.
     *
     * @return a configured {@code TalonFXConfiguration} object
     */
    public TalonFXConfiguration build() {
        config.MotorOutput.Inverted =
                cfg.ccwPositive ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode =
                cfg.neutralBrake ? NeutralModeValue.Coast
                                 : NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = cfg.motorToMechRatio;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = cfg.statorCurrentLimit;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = cfg.supplyCurrentLimit;

        if (cfg.forwardLimits != null) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = cfg.forwardLimits.enable;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = cfg.forwardLimits.limit;
        }

        if (cfg.reverseLimits != null) {
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = cfg.reverseLimits.enable;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = cfg.reverseLimits.limit;
        }

        return config;
    }
}
