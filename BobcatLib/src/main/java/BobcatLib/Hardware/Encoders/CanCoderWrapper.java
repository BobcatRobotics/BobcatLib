package BobcatLib.Hardware.Encoders;

import BobcatLib.Logging.Alert;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;

public class CanCoderWrapper implements SwerveAbsEncoder {
  public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
  private CANcoder encoder;
  /** Last angle reading was faulty. */
  public boolean readingError = false;

  public boolean isInverted = false;
  /** An {@link Alert} for if the CAN ID is greater than 40. */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  public final Alert magnetFieldLessThanIdeal;
  public final Alert readingFaulty;
  public EncoderConstants chosenModule;

  public CanCoderWrapper(int id, EncoderConstants chosenModule, String canivorename) {
    if (id >= 40) {
      canIdWarning.set(true);
    }
    magnetFieldLessThanIdeal =
        new Alert(
            "Encoders",
            "CANCoder " + id + " magnetic field is less than ideal.",
            Alert.AlertType.WARNING);
    readingFaulty =
        new Alert("Encoders", "CANCoder " + id + " reading was faulty.", Alert.AlertType.WARNING);
    this.chosenModule = chosenModule;
    configAbsEncoder();

    /* Angle Encoder Config */
    encoder = new CANcoder(id, canivorename);
    encoder.getConfigurator().apply(swerveCANcoderConfig);
  }

  /** Configs the absolute encoder sensor position , determining invertion. */
  public void configAbsEncoder() {
    /** Swerve CANCoder Configuration */
    swerveCANcoderConfig.MagnetSensor.SensorDirection =
        chosenModule.absoluteEncoderInvert.asSensorDirectionValue();
    isInverted = chosenModule.absoluteEncoderInvert.asBoolean();
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-1 to 0 ) or (0 to 1)
   */
  public double getAbsolutePosition() {
    readingError = false;
    MagnetHealthValue strength = encoder.getMagnetHealth().getValue();
    magnetFieldLessThanIdeal.set(strength != MagnetHealthValue.Magnet_Green);
    if (strength == MagnetHealthValue.Magnet_Invalid || strength == MagnetHealthValue.Magnet_Red) {
      readingError = true;
      readingFaulty.set(true);
      return 0;
    } else {
      readingFaulty.set(false);
    }
    return encoder.getAbsolutePosition().getValue();
  }

  /** Reset the encoder to factory defaults. */
  public void factoryDefault() {
    encoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  /** Clear sticky faults on the encoder. */
  public void clearStickyFaults() {
    encoder.clearStickyFaults();
  }
}
