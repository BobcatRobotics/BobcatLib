package BobcatLib.Hardware.Gyros;

import BobcatLib.Hardware.Gyros.parser.GyroDeviceJson;
import BobcatLib.Hardware.Gyros.parser.GyroJson;
import BobcatLib.Logging.FaultsAndErrors.FaultsWrapper;
import BobcatLib.Logging.FaultsAndErrors.Pigeon2Faults;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

/** Represents an interface for interacting with a gyro sensor. */
public class Pigeon2Gyro implements GyroIO {
  private FaultsWrapper gyroFaults;
  private Pigeon2 gyro;

  // Subscribed too the
  public StringSubscriber configSubscriber;
  public String lastConfig = "{}";
  public GyroJson jsonGyro;

  public Pigeon2Gyro() {
    loadConfigurationFromFile();
    configGyro();
  }

  public void configGyro() {
    gyro = new Pigeon2(jsonGyro.imu.id, jsonGyro.imu.canbus);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.reset();
    gyro.setYaw(0);
    gyroFaults = new Pigeon2Faults(gyro, gyro.getDeviceID());
  }

  public GyroJson loadConfigurationFromFile() {
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve");
    assert new File(directory, "gyro.json").exists();
    File gyroFile = new File(directory, "gyro.json");
    assert gyroFile.exists();
    jsonGyro = new GyroJson();
    try {
      jsonGyro = new ObjectMapper().readValue(gyroFile, GyroJson.class);
    } catch (IOException e) {
      jsonGyro.imu = new GyroDeviceJson(35, "");
    }
    return jsonGyro;
  }

  public void periodic() {}

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(getYaw());
    inputs.faulted = gyroFaults.hasFaultOccured();
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public void setYaw(double yaw) {
    gyro.setYaw(yaw);
  }

  private double getYaw() {
    StatusSignal<Double> raw_yaw = gyro.getYaw();
    return raw_yaw.getValue();
  }

  public double getTimeDiff() {
    return 1.0;
  }
}
