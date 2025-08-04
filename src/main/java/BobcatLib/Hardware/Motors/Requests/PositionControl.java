package BobcatLib.Hardware.Motors.Requests;

import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import BobcatLib.Hardware.Motors.SoftwareLimits;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Position control strategy for a motor using either voltage-based or FOC-based closed-loop control.
 */
public class PositionControl implements MotorRequest {
    final PositionVoltage request;
    final PositionTorqueCurrentFOC requestTorque;
    private boolean useMotionProfile = false;
    private boolean useTorqueFOC = false;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    /**
     * Constructs a PositionControl object with a given position in rotations.
     *
     * @param motorPosition The initial position to drive toward (in rotations).
     */
    public PositionControl(double motorPosition) {
        request = new PositionVoltage(motorPosition).withSlot(0);
        requestTorque = new PositionTorqueCurrentFOC(motorPosition).withSlot(0);
    }

    /**
     * Constructs a PositionControl object with a given position as an Angle.
     *
     * @param motorPosition The initial position to drive toward.
     */
    public PositionControl(Angle motorPosition) {
        request = new PositionVoltage(motorPosition).withSlot(0);
        requestTorque = new PositionTorqueCurrentFOC(motorPosition).withSlot(0);
    }

    /**
     * Constructs a PositionControl object targeting zero rotations.
     */
    public PositionControl() {
        request = new PositionVoltage(0).withSlot(0);
        requestTorque = new PositionTorqueCurrentFOC(0).withSlot(0);
    }

    /**
     * Enables or disables Field-Oriented Control (FOC) on the voltage request.
     *
     * @param isEnabled Whether FOC is enabled.
     * @return This PositionControl instance.
     */
    public PositionControl withFOC(boolean isEnabled) {
        request.withEnableFOC(isEnabled);
        return this;
    }

    /**
     * Configures whether to use Torque FOC control instead of Voltage control.
     *
     * @param isTorqueFOC Whether to use torque FOC.
     * @return This PositionControl instance.
     */
    public PositionControl withTorqueFOC(boolean isTorqueFOC) {
        this.useTorqueFOC = isTorqueFOC;
        return this;
    }

    /**
     * Sets custom PID gains via a configuration object (currently not implemented).
     *
     * @param pid The PID configuration.
     */
    public void withPid(ParentConfiguration pid) {
        // Placeholder
    }

    /**
     * Applies software forward and reverse motion limits.
     *
     * @param forwardLimit Forward motion limit.
     * @param reverseLimit Reverse motion limit.
     * @return This PositionControl instance.
     */
    public PositionControl withLimits(SoftwareLimits forwardLimit, SoftwareLimits reverseLimit) {
        if (useTorqueFOC) {
            requestTorque.withLimitForwardMotion(forwardLimit.isEnabled());
            requestTorque.withLimitReverseMotion(reverseLimit.isEnabled());
        } else {
            request.withLimitForwardMotion(forwardLimit.isEnabled());
            request.withLimitReverseMotion(reverseLimit.isEnabled());
        }
        return this;
    }

    /**
     * Sets an optional feedforward voltage or torque.
     *
     * @param feedForward Feedforward value.
     * @return This PositionControl instance.
     */
    public PositionControl withFeedForward(double feedForward) {
        if (useTorqueFOC) {
            requestTorque.withFeedForward(feedForward);
        } else {
            request.withFeedForward(feedForward);
        }
        return this;
    }

    /**
     * Sets a position target using WPILib Angle units.
     *
     * @param rotations Desired position.
     * @return This PositionControl instance.
     */
    public PositionControl withPosition(Angle rotations) {
        if (useTorqueFOC) {
            requestTorque.withPosition(rotations);
        } else {
            request.withPosition(rotations);
        }
        return this;
    }

    /**
     * Sets a position target using raw double (rotations).
     *
     * @param rotations Desired position.
     * @return This PositionControl instance.
     */
    public PositionControl withPosition(double rotations) {
        if (useTorqueFOC) {
            requestTorque.withPosition(rotations);
        } else {
            request.withPosition(rotations);
        }
        return this;
    }

    /**
     * Sets a velocity target using WPILib AngularVelocity units.
     *
     * @param speed Desired velocity.
     * @return This PositionControl instance.
     */
    public PositionControl withVelocity(AngularVelocity speed) {
        if (useTorqueFOC) {
            requestTorque.withVelocity(speed);
        } else {
            request.withVelocity(speed);
        }
        return this;
    }

    /**
     * Sets a velocity target using raw double (rotations/sec).
     *
     * @param speed Desired velocity.
     * @return This PositionControl instance.
     */
    public PositionControl withVelocity(double speed) {
        if (useTorqueFOC) {
            requestTorque.withVelocity(speed);
        } else {
            request.withVelocity(speed);
        }
        return this;
    }

    /**
     * Configures a trapezoidal motion profile to constrain velocity and acceleration.
     *
     * @param maxVelocity Maximum velocity.
     * @param maxAccel Maximum acceleration.
     * @param targetPosition Final target position.
     * @param targetVelocity Final target velocity.
     * @return This PositionControl instance.
     */
    public PositionControl withMotionProfile(double maxVelocity, double maxAccel,
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
     * Applies the control request for a new position setpoint (Angle).
     * Supports motion profiling.
     *
     * @param rotations Desired position.
     * @return This PositionControl instance.
     */
    public PositionControl setRequest(Angle rotations) {
        if (useMotionProfile) {
            setpoint = profile.calculate(0.020, setpoint, goal);
            request.Position = setpoint.position;
            request.Velocity = setpoint.velocity;
        } else {
            withPosition(rotations);
        }
        return this;
    }

    /**
     * Applies the control request for a new position setpoint (rotations).
     * Supports motion profiling.
     *
     * @param rotations Desired position.
     * @return This PositionControl instance.
     */
    public PositionControl setRequest(double rotations) {
        if (useMotionProfile) {
            setpoint = profile.calculate(0.020, setpoint, goal);
            request.Position = setpoint.position;
            request.Velocity = setpoint.velocity;
        } else {
            withPosition(rotations);
        }
        return this;
    }

    /**
     * Gets the final configured ControlRequest to pass into a motor controller.
     *
     * @return The CTRE control request instance.
     */
    public ControlRequest getRequest() {
        return useTorqueFOC ? requestTorque : request;
    }
}
