package BobcatLib.Hardware.LedControllers;

import org.littletonrobotics.junction.Logger;

public class BaseLedController {
  private final LedControllerIO io;
  private final LedControllerIOInputsAutoLogged inputs = new LedControllerIOInputsAutoLogged();

  public BaseLedController(LedControllerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
  }

  public void checkForFaults() {
    io.checkForFaults();
  }
}
