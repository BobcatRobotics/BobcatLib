package BobcatLib.Hardware.Encoders;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.FaultsAndErrors.CanAndCoderFaults;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

public class CanAndCoderWrapper implements EncoderIO {
  private Canandcoder encoder;
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
  public EncoderConstants chosenModule;

  private CanAndCoderFaults faults;

  public CanAndCoderWrapper(int id, EncoderConstants chosenModule) {
    if (id >= 40) {
      canIdWarning.set(true);
    }
    magnetFieldLessThanIdeal =
        new Alert(
            "Encoders",
            "CanAndCoder " + id + " magnetic field is less than ideal.",
            Alert.AlertType.WARNING);
    this.chosenModule = chosenModule;
    configAbsEncoder();

    /* Angle Encoder Config */
    encoder = new Canandcoder(id);

    faults = new CanAndCoderFaults(encoder, id);
  }

  /** Configs the absolute encoder sensor position , determining invertion. */
  public void configAbsEncoder() {
    boolean inverted = chosenModule.absoluteEncoderInvert.asBoolean();
    encoder.setSettings(new Canandcoder.Settings().setInvertDirection(inverted));
    isInverted = inverted;
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-1 to 0 ) or (0 to 1)
   */
  public double getAbsolutePosition() {
    readingError = false;
    magnetFieldLessThanIdeal.set(encoder.magnetInRange());
    return encoder.getAbsPosition();
  }

  /**
   * Reset the encoder to factory defaults.
   *
   * <p>This will not clear the stored zero offset.
   */
  public void factoryDefault() {
    encoder.resetFactoryDefaults(false);
  }

  /** Clear sticky faults on the encoder. */
  public void clearStickyFaults() {
    encoder.clearStickyFaults();
  }

  public void checkForFaults() {
    faults.hasFaultOccured();
  }
}
