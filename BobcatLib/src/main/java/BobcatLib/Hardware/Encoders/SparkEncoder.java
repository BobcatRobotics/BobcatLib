package BobcatLib.Hardware.Encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

/** SparkMax/SparkFlex absolute encoder, attached through the data port. */
public class SparkEncoder {
  public AbsoluteEncoder encoder;
  public boolean isInverted = false;
  public EncoderConstants chosenModule;

  public SparkEncoder(CANSparkMax motor, EncoderConstants chosenModule) {
    this.chosenModule = chosenModule;
    configAbsEncoder();
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  public SparkEncoder(CANSparkFlex motor, EncoderConstants chosenModule) {
    this.chosenModule = chosenModule;
    configAbsEncoder();
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  /** Configs the absolute encoder sensor position , determining invertion. */
  public void configAbsEncoder() {
    isInverted = chosenModule.absoluteEncoderInvert.asBoolean();
    encoder.setPositionConversionFactor(1);
    encoder.setVelocityConversionFactor(1);
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-1 to 0 ) or (0 to 1)
   */
  public double getAbsolutePosition() {
    return encoder.getPosition();
  }

  /** Reset the encoder to factory defaults. */
  public void factoryDefault() {
    // Do nothing
  }

  /** Clear sticky faults on the encoder. */
  public void clearStickyFaults() {
    // Do nothing
  }

  public void checkForFaults() {}
}
