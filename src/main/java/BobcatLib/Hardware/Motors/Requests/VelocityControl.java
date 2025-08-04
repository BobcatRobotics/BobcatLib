package BobcatLib.Hardware.Motors.Requests;

import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import BobcatLib.Hardware.Motors.SoftwareLimits;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Class representing a velocity control request for motor controllers.
 * Supports both voltage-based and torque FOC control modes, with optional motion profiling.
 */
public class VelocityControl implements MotorRequest {

    private final VelocityVoltage voltageRequest;
    private final VelocityTorqueCurrentFOC torqueRequest ;
    private boolean useMotionProfile = false;
    private boolean useTorqueFOC = false;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    /**
     * Default constructor.
     */
    public VelocityControl() {
        voltageRequest = new VelocityVoltage(0).withSlot(0);
        torqueRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
    }

    /**
     * Constructor with raw velocity input. Not implemented yet.
     * @param velocity Target velocity in rotations per second.
     */
    public VelocityControl(double velocity) {
        voltageRequest = new VelocityVoltage(velocity).withSlot(0);
        torqueRequest = new VelocityTorqueCurrentFOC(velocity).withSlot(0);
    }

    /**
     * Constructor with AngularVelcoity input. Not implemented yet.
     * @param velocity Target position in rotations per second.
     */
    public VelocityControl(AngularVelocity velocity) {
        voltageRequest = new VelocityVoltage(velocity).withSlot(0);
        torqueRequest = new VelocityTorqueCurrentFOC(velocity).withSlot(0);
    }

    /**
     * Enables or disables Field-Oriented Control (FOC).
     * @param enabled True to enable, false to disable.
     * @return This instance for chaining.
     */
    public VelocityControl withFOC(boolean enabled) {
        voltageRequest.withEnableFOC(enabled);
        return this;
    }

    /**
     * Sets the control mode to Torque FOC or Voltage.
     * @param enabled True to use Torque FOC, false for Voltage control.
     * @return This instance for chaining.
     */
    public VelocityControl withTorqueFOC(boolean enabled) {
        this.useTorqueFOC = enabled;
        return this;
    }

    /**
     * Sets PID configuration (not implemented).
     * @param pid PID configuration object.
     * @return This instance for chaining.
     */
    public VelocityControl withPid(ParentConfiguration pid) {
        // Placeholder for future PID implementation
        return this;
    }

    /**
     * Applies software forward and reverse motion limits.
     * @param forwardLimit Forward motion limit.
     * @param reverseLimit Reverse motion limit.
     * @return This instance for chaining.
     */
    public VelocityControl withLimits(SoftwareLimits forwardLimit, SoftwareLimits reverseLimit) {
        if (useTorqueFOC) {
            torqueRequest.withLimitForwardMotion(forwardLimit.isEnabled());
            torqueRequest.withLimitReverseMotion(reverseLimit.isEnabled());
        } else {
            voltageRequest.withLimitForwardMotion(forwardLimit.isEnabled());
            voltageRequest.withLimitReverseMotion(reverseLimit.isEnabled());
        }
        return this;
    }

    /**
     * Sets a feedforward value.
     * @param feedForward Feedforward constant.
     * @return This instance for chaining.
     */
    public VelocityControl withFeedForward(double feedForward) {
        if (useTorqueFOC) {
            torqueRequest.withFeedForward(feedForward);
        } else {
            voltageRequest.withFeedForward(feedForward);
        }
        return this;
    }

    /**
     * Sets acceleration using AngularAcceleration units.
     * @param acceleration Target acceleration.
     * @return This instance for chaining.
     */
    public VelocityControl withAcceleration(AngularAcceleration acceleration) {
        if (useTorqueFOC) {
            torqueRequest.withAcceleration(acceleration);
        } else {
            voltageRequest.withAcceleration(acceleration);
        }
        return this;
    }

    /**
     * Sets acceleration using a raw double value.
     * @param acceleration Target acceleration.
     * @return This instance for chaining.
     */
    public VelocityControl withAcceleration(double acceleration) {
        if (useTorqueFOC) {
            torqueRequest.withAcceleration(acceleration);
        } else {
            voltageRequest.withAcceleration(acceleration);
        }
        return this;
    }

    /**
     * Sets velocity using AngularVelocity units.
     * @param velocity Target velocity.
     * @return This instance for chaining.
     */
    public VelocityControl withVelocity(AngularVelocity velocity) {
        if (useTorqueFOC) {
            torqueRequest.withVelocity(velocity);
        } else {
            voltageRequest.withVelocity(velocity);
        }
        return this;
    }

    /**
     * Sets velocity using a raw double value.
     * @param velocity Target velocity.
     * @return This instance for chaining.
     */
    public VelocityControl withVelocity(double velocity) {
        if (useTorqueFOC) {
            torqueRequest.withVelocity(velocity);
        } else {
            voltageRequest.withVelocity(velocity);
        }
        return this;
    }

    /**
     * Configures a trapezoidal motion profile.
     * @param maxVelocity Maximum velocity.
     * @param maxAccel Maximum acceleration.
     * @param targetPosition Desired final position.
     * @param targetVelocity Desired final velocity.
     * @return This instance for chaining.
     */
    public VelocityControl withMotionProfile(double maxVelocity, double maxAccel,
                                             double targetPosition, double targetVelocity) {
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity, maxAccel)
        );
        goal = new TrapezoidProfile.State(targetPosition, targetVelocity);
        setpoint = new TrapezoidProfile.State();
        useMotionProfile = true;
        return this;
    }

    /**
     * Applies a velocity request using AngularVelocity.
     * @param velocity Desired velocity.
     * @return The configured request object.
     */
    public VelocityControl setRequest(AngularVelocity velocity) {
        if (useMotionProfile) {
            // Motion profile logic to be added later
        } else {
            withVelocity(velocity);
        }
        return this;
    }


    /**
     * Applies a velocity request using a double.
     * @param velocity Desired velocity.
     * @return The configured request object.
     */
    public VelocityControl setRequest(double velocity) {
        if (useMotionProfile) {
            // Motion profile logic to be added later
        } else {
            withVelocity(velocity);
        }
        return this;
    }

    /**
     * Gets the active control request.
     * @return The current control request object.
     */
    public ControlRequest getRequest() {
        return useTorqueFOC ? torqueRequest : voltageRequest;
    }
}
