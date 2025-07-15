package org.Hardware.CTRE.Configurators;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;

/** Applies TalonFX configs */
public class TalonFXConfigApplier extends ConfigApplier {
  /**
   * Reports to the user that a TalonFX failed configuration
   *
   * @param talonFX the TalonFX
   */
  private static void report(TalonFX talonFX) {
    DriverStation.reportWarning(
        "Failed to apply config to TalonFX with ID: " + talonFX.getDeviceID(), false);
  }

  /**
   * Applies a factory default config to a TalonFX
   *
   * @param talonFX the TalonFX
   */
  public static void applyFactoryDefault(TalonFX talonFX) {
    TalonFXConfiguration factoryDefaults = new TalonFXConfiguration();

    TalonFXConfigurator configurator = talonFX.getConfigurator();

    if (attempt(() -> configurator.apply(factoryDefaults)) == false) {
      report(talonFX);
    }
  }

  /**
   * Applies a motor config to a TalonFX
   *
   * @param talonFX the TalonFX
   * @param motorConfig the motor config
   */
  public static void apply(TalonFX talonFX, TalonFXConfiguration motorConfig) {
    TalonFXConfigurator configurator = talonFX.getConfigurator();

    if (attempt(() -> configurator.apply(motorConfig)) == false) {
      report(talonFX);
    }
  }
}
