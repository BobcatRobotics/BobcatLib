package BobcatLib.Hardware.Sensors.TOFs;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class SENS3006 implements TimeOfFlightIO {
  public int id;
  public TimeOfFlight tof;
  public final double sampleTime;
  public RangingMode mode;
  public double range;
  public Alert sensorAlert;
  public boolean enable = false;

  public SENS3006(int id, RangingMode mode, double sampleTime) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {
      tof = new TimeOfFlight(id);
      configRangeSensor();
      enable = true;
    } catch (Exception e) {
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occured", level);
      sensorAlert.set(true);
    }
  }
  /**
   * Gets the range in front of the sensor.
   *
   * @return range in mm
   */
  public double getRange() {
    if (!enable) {
      return 0;
    }
    range = tof.getRange();
    return range;
  }

  public void configRangeSensor() {
    tof.setRangingMode(mode, sampleTime);
  }

  public void configRangeSensor(RangingMode m) {
    this.mode = m;
    tof.setRangingMode(mode, sampleTime);
  }

  public RangingMode getMode() {
    return mode;
  }
}
