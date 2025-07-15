package org.Hardware.CTRE.Motors.Controls.MotionMagicVelocityDutyCycle;

import org.Hardware.CTRE.Configurators.TalonFXConfigApplier;
import org.Hardware.CTRE.Motors.Controls.ControlBase;
import org.Hardware.CTRE.Motors.Controls.ControlInterface;
import org.Hardware.CTRE.Motors.Controls.ControlValues;
import org.Hardware.CTRE.Motors.Controls.FeedforwardController;
import org.Hardware.Utils.CANDeviceDetails;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXControlRequest extends ControlBase implements ControlInterface {
    private final TalonFX motor;
    private final TalonFXConfiguration config;
    private final CANDeviceDetails details;
    private final MotionMagicVelocityDutyCycle request;

    private AngularAcceleration setPointAcceleration;
    private AngularVelocity setPointVelocity;
  
    public TalonFXControlRequest(CANDeviceDetails details,
        TalonFXConfiguration config,
        FeedforwardController feedforwardController, boolean enableFOC, AngularVelocity angularVelocity) {
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
      request = new MotionMagicVelocityDutyCycle(angularVelocity);
      setPointVelocity = angularVelocity;
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
  
    @Override
    public void setPos(AngularVelocity angularVelocity) {
      // approach setpoint
      motor.setControl(request.withVelocity(angularVelocity));
    }
  
    @Override
    public void setSetpoint(AngularVelocity angularVelocity) {
      setPointVelocity = angularVelocity;
    }
  
    public TalonFXControlRequest withAccelerationSetPoint(AngularAcceleration angularAcceleration) {
      setPointAcceleration = angularAcceleration;
      // approach setpoint
      request.Acceleration = setPointAcceleration.magnitude();
      return this;
    }
  
    @Override
    public void periodic() {
      // approach setpoint
      request.Velocity = setPointVelocity.magnitude();
      motor.setControl(request);
    }
  
    @Override
    public void setPos(Voltage voltageInVolts) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPos'");
    }
  
    @Override
    public void setPos(Current currentInAmps) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPos'");
    }
  
    @Override
    public void setPos(Angle angleInRotations) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPos'");
    }
  
    @Override
    public void setSetpoint(Voltage voltageInVolts) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }
  
    @Override
    public void setSetpoint(Current currentInAmps) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }
  
    @Override
    public void setSetpoint(Angle angleInRotations) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }
  }