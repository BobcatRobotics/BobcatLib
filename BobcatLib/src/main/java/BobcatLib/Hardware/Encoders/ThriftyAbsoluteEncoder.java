package BobcatLib.Hardware.Encoders;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class ThriftyAbsoluteEncoder implements SwerveAbsEncoder {
  public AnalogInput encoder;
  private boolean isInverted;
  public EncoderConstants chosenModule;

  public ThriftyAbsoluteEncoder(int id, EncoderConstants chosenModule) {
    this.chosenModule = chosenModule;
    configAbsEncoder();
    encoder = new AnalogInput(id);
  }

  /** Configs the absolute encoder sensor position , determining invertion. */
  public void configAbsEncoder() {
    isInverted = chosenModule.absoluteEncoderInvert.asBoolean();
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-1 to 0 ) or (0 to 1)
   */
  public double getAbsolutePosition() {
    return (isInverted ? -1.0 : 1.0)
        * (encoder.getAverageVoltage() / RobotController.getVoltage5V());
  }

  /** Reset the encoder to factory defaults. */
  public void factoryDefault() {
    // Do nothing
  }

  /** Clear sticky faults on the encoder. */
  public void clearStickyFaults() {
    // Do nothing
  }
}
