package BobcatLib.Hardware.Sensors.TOFs;

import com.playingwithfusion.TimeOfFlight.RangingMode;

public interface TimeOfFlightIO {
  /**
   * Gets the range in front of the sensor.
   *
   * @return range in mm
   */
  public default double getRange() {
    return 0;
  }
  /** config the sensor with default setting for the device. */
  public default void configRangeSensor() {}
  /**
   * Given the mode , config the sensor to the right detection parameters.
   *
   * @param mode
   */
  public default void configRangeSensor(RangingMode mode) {}
  /**
   * Gets the Ranging Mode of the sensor
   *
   * @return RangingMode
   */
  public default RangingMode getMode() {
    return RangingMode.Short;
  }
}
