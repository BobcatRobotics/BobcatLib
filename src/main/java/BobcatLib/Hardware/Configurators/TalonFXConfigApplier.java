package BobcatLib.Hardware.Configurators;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import BobcatLib.Hardware.Motors.TalonFX.FalconMotor;
import edu.wpi.first.wpilibj.DriverStation;

/** Applies TalonFX configs */
public class TalonFXConfigApplier extends ConfigApplier {
  /**
   * Reports to the user that a TalonFX failed configuration
   *
   * @param talonFX the TalonFX
   */
  private static void report(FalconMotor talonFX) {
    DriverStation.reportWarning(
        "Failed to apply config to TalonFX with ID: " + talonFX.getDeviceID(), false);
  }

  /**
   * Applies a factory default config to a TalonFX
   *
   * @param talonFX the TalonFX
   */
  public static void applyFactoryDefault(FalconMotor talonFX) {
    TalonFXConfiguration factoryDefaults = new TalonFXConfiguration();
    apply(talonFX, factoryDefaults);
  }

  /**
   * Applies a motor config to a TalonFX
   *
   * @param talonFX the TalonFX
   * @param motorConfig the motor config
   */
  public static void apply(FalconMotor talonFX, TalonFXConfiguration motorConfig) {
    TalonFXConfigurator configurator = talonFX.getConfigurator();

    if (attempt(() -> configurator.apply(motorConfig)) == false) {
      report(talonFX);
    }
  }
}
