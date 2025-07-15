package org.Hardware.CTRE.Motors.Controls.Differential.DifferentialPositionDutyCycle;

import org.Hardware.CTRE.Configurators.TalonFXConfigApplier;
import org.Hardware.CTRE.Motors.Controls.ControlBase;
import org.Hardware.CTRE.Motors.Controls.ControlInterface;
import org.Hardware.CTRE.Motors.Controls.ControlValues;
import org.Hardware.CTRE.Motors.Controls.FeedforwardController;
import org.Hardware.Utils.CANDeviceDetails;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXControlRequest extends ControlBase implements ControlInterface {
    private final TalonFX motor;
    private final TalonFXConfiguration config;
    private final CANDeviceDetails details;
    private final  DifferentialPositionDutyCycle request;

    private double setpointRotations;

    public TalonFXControlRequest(CANDeviceDetails details,
            TalonFXConfiguration config,
            FeedforwardController feedforwardController, boolean enableFOC, double rotations) {
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
        request = new DifferentialPositionDutyCycle(0.0,rotations).withEnableFOC(enableFOC);
        setpointRotations = rotations;
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
    public void setPos(double posRotations) {
      request.DifferentialPosition = posRotations;
      motor.setControl(request);
    }
  
    @Override
    public void setSetpoint(double rotations) {
      setpointRotations = rotations;
    }
  
    @Override
    public void periodic() {
      // approach setpoint
      request.DifferentialPosition = setpointRotations;
      motor.setControl(request);
    }
}
