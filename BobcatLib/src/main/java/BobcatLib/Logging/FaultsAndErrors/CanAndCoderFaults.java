package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class CanAndCoderFaults implements FaultsWrapper {
  public int id;
  public Canandcoder encoder;
  /**
   * An {@link Alert} the undervolt flag, which will raise if the encoder is experiencing brownout
   * conditions.
   */
  public static Alert underVoltAlert;
  /**
   * An {@link Alert} the magnet out of range flag, which will raise if the measured shaft's magnet
   * is not detected. This will match the encoder's LED shining red in normal operation.
   */
  public static Alert magnetOutOfRangeAlert;
  /**
   * An {@link Alert} the hardware fault flag, which will raise if a hardware issue is detected.
   * Generally will raise if the device's controller cannot read the physical sensor itself.
   */
  public static Alert hardwareFaultAlert;
  /**
   * An {@link Alert} the temperature range flag, which will raise if the encoder is not between
   * 0-70 degrees Celsius. This may be of concern if the encoder is near very active motors.
   */
  public static Alert outOfTemperatureRangeAlert;
  /**
   * An {@link Alert} the CAN general error flag, which will raise if the encoder cannot RX packets
   * reliably. This is usually due to wiring issues, such as a shorted CAN bus.
   */
  public static Alert canGeneralErrorAlert;
  /**
   * An {@link Alert} the CAN ID conflict flag, which is set to true if there is a CAN id conflict.
   * In practice, you should physically inspect the encoder to ensure it's not flashing blue.
   */
  public static Alert canIDConflictAlert;
  /**
   * An {@link Alert} the power cycle fault flag, which is set to true when the encoder first boots.
   * Clearing sticky faults and then checking this flag can be used to determine if the encoder
   * rebooted.
   */
  public static Alert powerCycleAlert;

  /**
   * @param enc
   * @param id
   */
  public CanAndCoderFaults(Canandcoder enc, int id) {
    this.id = id;
    this.encoder = enc;
    AlertType level = AlertType.INFO;
    underVoltAlert =
        new Alert(
            "Encoders", "CanAndCoder " + id + " is undervoltaged, potential brownout.", level);
    magnetOutOfRangeAlert =
        new Alert("Encoders", "CanAndCoder " + id + " magnetic field is less than ideal.", level);
    hardwareFaultAlert =
        new Alert("Encoders", "CanAndCoder " + id + " has a hardware fault.", level);
    outOfTemperatureRangeAlert =
        new Alert("Encoders", "CanAndCoder " + id + " out of specified temperature range.", level);
    canGeneralErrorAlert =
        new Alert("Encoders", "CanAndCoder " + id + " has a general error.", level);
    canIDConflictAlert =
        new Alert("Encoders", "CanAndCoder " + id + " has a conflict with it's CAN id.", level);
    powerCycleAlert = new Alert("Encoders", "CanAndCoder " + id + " recently rebooted.", level);
  }

  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("CanAndCoder " + id);
  }

  public void activateAlert(Alert alert, AlertType type) {
    alert.setLevel(type);
    alert.set(true);
  }

  public void disableAlert(Alert alert) {
    alert.set(false);
  }

  public boolean hasFaultOccured() {
    List<Alert> foundFaults = new ArrayList<>();
    Map<Boolean, Alert> faultChecks =
        Map.of(
            encoder.getActiveFaults().underVolt(), underVoltAlert,
            encoder.getActiveFaults().magnetOutOfRange(), magnetOutOfRangeAlert,
            encoder.getActiveFaults().hardwareFault(), hardwareFaultAlert,
            encoder.getActiveFaults().outOfTemperatureRange(), outOfTemperatureRangeAlert,
            encoder.getActiveFaults().canGeneralError(), canGeneralErrorAlert,
            encoder.getActiveFaults().canIDConflict(), canIDConflictAlert,
            encoder.getActiveFaults().powerCycle(), powerCycleAlert);

    faultChecks.forEach(
        (faultCondition, alert) -> {
          if (faultCondition) {
            foundFaults.add(alert);
          }
        });

    foundFaults.forEach(this::activateAlert);

    return !foundFaults.isEmpty();
  }
}
