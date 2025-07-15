package org.Hardware.Utils;

import java.util.Objects;

/** CAN class that makes using multiple buses simpler */
public record CANDeviceDetails(int id, String bus, Manufacturer manufacturer,
    String subsystemName) {
  /** Enum representing the manufacturer of the CAN device. */
  public enum Manufacturer {
    Unknown, // Unknown vendor
    Thrifty, // Thrifty vendor
    Grapple, // Grapple vendor
    Pwf, // Pwf vendor
    Redux, // Redux vendor
    Rev, // Rev vendor
    Ctre // Ctre vendor
  }

  /**
   * Creates a CAN identifier for a device
   *
   * @param id CAN id of device
   * @param bus CAN bus of device
   * @param manufacturer CAN bus of device
   * @param subsystemName subsystem this device belongs too.
   */
  public CANDeviceDetails {
    Objects.requireNonNull(id);
    Objects.requireNonNull(bus);
    Objects.requireNonNull(manufacturer);
    Objects.requireNonNull(subsystemName);
  }

  /**
   * CAN constructor that uses the default bus name, an empty string
   *
   * @param id CAN id of device
   */
  public CANDeviceDetails(int id) {
    this(id, "", Manufacturer.Unknown, "");
  }

  /**
   * CAN constructor that uses the default bus name, an empty string
   *
   * @param id CAN id of device
   * @param bus CAN bus of device
   */
  public CANDeviceDetails(int id, String bus) {
    this(id, bus, Manufacturer.Unknown, "");
  }

  /**
   * CAN constructor that uses the default bus name, an empty string
   *
   * @param id CAN id of device
   * @param bus CAN bus of device
   * @param manufacturer CAN bus of device
   */
  public CANDeviceDetails(int id, String bus, Manufacturer manufacturer) {
    this(id, bus, manufacturer, "");
  }
}
