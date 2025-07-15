package BobcatLib.Hardware.Configurators;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.DriverStation;

/** Applies Pigeon 2 configs */
public class Pigeon2ConfigApplier extends ConfigApplier {

  /**
   * Reports to the user that a Pigeon 2 failed configuration
   *
   * @param pigeon2 the Pigeon 2 that failed configuration
   */
  private static void report(Pigeon2 pigeon2) {
    DriverStation.reportWarning(
        "Failed to apply config to Pigeon 2 with ID: " + pigeon2.getDeviceID(), false);
  }

  /**
   * Applies a factory default config to a Pigeon 2
   *
   * @param pigeon2 pigeon instance
   */
  public static void applyFactoryDefault(Pigeon2 pigeon2) {
    Pigeon2Configuration factoryDefaults = new Pigeon2Configuration();
    apply(pigeon2, factoryDefaults);
  }

  /**
   * Applies a pigeon2 config to a Pigeon2
   *
   * @param pigeon2 the pigeon2
   * @param gyroConfig the gyro config
   */
  public static void apply(Pigeon2 pigeon2, Pigeon2Configuration gyroConfig) {
    Pigeon2Configurator configurator = pigeon2.getConfigurator();

    if (attempt(() -> configurator.apply(gyroConfig)) == false) {
      report(pigeon2);
    }
  }
}
