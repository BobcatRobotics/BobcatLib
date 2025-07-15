package org.Hardware.CTRE.Motors.Controls;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ControlBase {
    protected StatusSignal<Angle> position;
    protected StatusSignal<AngularVelocity> velocity;
    protected StatusSignal<AngularAcceleration> acceleration;
    protected StatusSignal<Voltage> volts;
    protected StatusSignal<Current> amps;

    public ControlBase(){

    }
}
