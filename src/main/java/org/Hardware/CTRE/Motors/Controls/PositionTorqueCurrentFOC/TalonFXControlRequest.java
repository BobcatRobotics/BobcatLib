package org.Hardware.CTRE.Motors.Controls.PositionTorqueCurrentFOC;

import org.Hardware.CTRE.Configurators.TalonFXConfigApplier;
import org.Hardware.CTRE.Motors.Controls.ControlBase;
import org.Hardware.CTRE.Motors.Controls.ControlInterface;
import org.Hardware.CTRE.Motors.Controls.ControlValues;
import org.Hardware.CTRE.Motors.Controls.FeedforwardController;
import org.Hardware.Utils.CANDeviceDetails;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXControlRequest extends ControlBase implements ControlInterface {
    private final TalonFX motor;
    private final TalonFXConfiguration config;
    private final CANDeviceDetails details;
    private final PositionTorqueCurrentFOC request;

    private Angle setPointAngleInRotations;
    private AngularVelocity setPointAngularVelocity;

    public TalonFXControlRequest(CANDeviceDetails details,
            TalonFXConfiguration config,
            FeedforwardController feedforwardController, boolean enableFOC, Angle angleInRotations) {
        super();
        this.details = details;
        this.config = config;
        // create hardware
        motor = new TalonFX(details.id(), details.bus());

        // status signals (values in memory updated at a fixed rate by hardware over
        // CAN)
        position = motor.getPosition();
        velocity = motor.getVelocity();
        acceleration = motor.getAcceleration();
        volts = motor.getMotorVoltage();
        amps = motor.getStatorCurrent();

        // default voltage
        request = new PositionTorqueCurrentFOC(angleInRotations);
        setPointAngleInRotations = angleInRotations;
    }

    @Override
    public void configure() {
      BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration);
      BaseStatusSignal.setUpdateFrequencyForAll(50, volts, amps);
  
      ParentDevice.optimizeBusUtilizationForAll(motor);
  
      TalonFXConfigApplier.applyFactoryDefault(motor);
      TalonFXConfigApplier.apply(motor, config);
    }
  
    @Override
    public void getUpdatedVals(ControlValues values) {
      BaseStatusSignal.refreshAll(position, velocity, acceleration, volts, amps);
  
      values.posRotations = position.getValueAsDouble();
      values.velRotationsPerSec = velocity.getValueAsDouble();
      values.accRotationsPerSecPerSec = acceleration.getValueAsDouble();
      values.motorVolts = volts.getValueAsDouble();
      values.motorAmps = amps.getValueAsDouble();
    }

    public TalonFXControlRequest withVelocitySetPoint(AngularVelocity angularVelocity) {
      setPointAngularVelocity = angularVelocity;
      // approach setpoint
      request.Velocity = setPointAngularVelocity.magnitude();
      return this;
    }

    @Override
    public void setPos(Angle angleInRotations) {
      // approach setpoint
      motor.setControl(request.withPosition(angleInRotations));
    }
  
    @Override
    public void setSetpoint(Angle angleInRotations) {
      setPointAngleInRotations = angleInRotations;
    }
  

    @Override
    public void periodic() {
      // approach setpoint
      motor.setControl(request.withPosition(setPointAngleInRotations));
    }
  
    @Override
    public void setPos(AngularVelocity angularVelocityInRotationsPerSecond) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPos'");
    }
  
    @Override
    public void setSetpoint(AngularVelocity angularVelocityInRotationsPerSecond) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }
  
    @Override
    public void setPos(Current currentInAmps) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPos'");
    }
  
    @Override
    public void setSetpoint(Current currentInAmps) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }

    @Override
    public void setPos(Voltage voltageInVolts) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPos'");
    }

    @Override
    public void setSetpoint(Voltage voltageInVolts) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }

    @Override
    public void setPos(Velocity<AngularAccelerationUnit> jerkInVelocitySecondsCubed) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPos'");
    }

    @Override
    public void setSetpoint(Velocity<AngularAccelerationUnit> jerkInVelocitySecondsCubed) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }
  }
  