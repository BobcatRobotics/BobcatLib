package BobcatLib.Hardware.Encoders;

public interface SwerveAbsEncoder {
  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-1 to 0 ) or (0 to 1)
   */
  public default double getAbsolutePosition() {
    return 0;
  }
  ;

  /** Reset the encoder to factory defaults. */
  public default void factoryDefault() {
    // Do nothing
  }

  /** Clear sticky faults on the encoder. */
  public default void clearStickyFaults() {
    // Do nothing
  }
}
