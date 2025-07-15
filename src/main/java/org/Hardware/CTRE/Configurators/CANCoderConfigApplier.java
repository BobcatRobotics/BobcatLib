package org.Hardware.CTRE.Configurators;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DriverStation;

/** Applies CANcoder configs. */
public class CANCoderConfigApplier extends ConfigApplier {

  /**
   * Reports to the user that a CANcoder failed configuration.
   *
   * @param cancoder the CANcoder.
   */
  private static void report(CANcoder cancoder) {
    DriverStation.reportWarning(
        "Failed to apply config to CANcoder with ID: " + cancoder.getDeviceID(), false);
  }

  /**
   * Applies a factory default config to a CANcoder.
   *
   * @param cancoder the CANcoder.
   */
  public static void applyFactoryDefault(CANcoder cancoder) {
    CANcoderConfiguration factoryDefaults = new CANcoderConfiguration();

    CANcoderConfigurator configurator = cancoder.getConfigurator();

    if (attempt(() -> configurator.apply(factoryDefaults)) == false) {
      report(cancoder);
    }
  }

  /**
   * Applies an absolute encoder config to a CANcoder.
   *
   * @param cancoder the CANcoder.
   * @param cancoderConfig the absolute encoder config.
   */
  public static void apply(CANcoder cancoder, CANcoderConfiguration cancoderConfig) {
    CANcoderConfigurator configurator = cancoder.getConfigurator();

    if (attempt(() -> configurator.apply(cancoderConfig)) == false) {
      report(cancoder);
    }
  }
}
