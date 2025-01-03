package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper class for the Xbox controller that implements the {@link ControllerWrapper} interface.
 * This class provides standardized access to joystick axes and button triggers for the Xbox
 * controller.
 */
public class XboxControllerWrapper extends CommandXboxController implements ControllerWrapper {
  /** Axis index for forward/backward translation. */
  public final int translationAxis = XboxController.Axis.kLeftY.value;

  /** Axis index for side-to-side strafing. */
  public final int strafeAxis = XboxController.Axis.kLeftX.value;

  /** Axis index for rotation control. */
  public final int rotationAxis = XboxController.Axis.kRightX.value;

  /**
   * Constructs an Xbox controller wrapper for the specified port. The wrapper includes access to
   * left and right joysticks, POV controls, and button mappings (Y, B, A, X).
   *
   * @param port the port the Xbox controller is connected to.
   */
  public XboxControllerWrapper(int port) {
    super(port);
  }

  /**
   * Gets the axis value for translation (forward/backward) control.
   *
   * @return the axis value for translation control.
   */
  public double getTranslationAxis() {
    return super.getRawAxis(translationAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return the axis value for strafing control.
   */
  public double getStrafeAxis() {
    return super.getRawAxis(strafeAxis);
  }

  /**
   * Gets the axis value for rotation control.
   *
   * @return the axis value for rotation control.
   */
  public double getRotationAxis() {
    return super.getRawAxis(rotationAxis);
  }

  /**
   * Gets the trigger for the left trigger button.
   *
   * @return a {@link Trigger} object for the left trigger button.
   */
  public Trigger getLeftTrigger() {
    return super.leftTrigger(0.3);
  }

  /**
   * Gets the trigger for the right trigger button.
   *
   * @return a {@link Trigger} object for the right trigger button.
   */
  public Trigger getRightTrigger() {
    return super.rightTrigger(0.3);
  }

  /**
   * Gets the trigger for the left bumper button.
   *
   * @return a {@link Trigger} object for the left bumper button.
   */
  public Trigger getLeftBumper() {
    return super.leftBumper();
  }

  /**
   * Gets the trigger for the right bumper button.
   *
   * @return a {@link Trigger} object for the right bumper button.
   */
  public Trigger getRightBumper() {
    return super.rightBumper();
  }

  /**
   * Gets the trigger for the Y button.
   *
   * @return a {@link Trigger} object for the Y button.
   */
  public Trigger getYorTriangle() {
    return super.y();
  }

  /**
   * Gets the trigger for the B button.
   *
   * @return a {@link Trigger} object for the B button.
   */
  public Trigger getBorCircle() {
    return super.b();
  }

  /**
   * Gets the trigger for the A button.
   *
   * @return a {@link Trigger} object for the A button.
   */
  public Trigger getAorCross() {
    return super.a();
  }

  /**
   * Gets the trigger for the X button.
   *
   * @return a {@link Trigger} object for the X button.
   */
  public Trigger getXorSquare() {
    return super.x();
  }

  /**
   * Gets the trigger for the D-Pad up direction.
   *
   * @return a {@link Trigger} object for the D-Pad up direction.
   */
  public Trigger getDPadTriggerUp() {
    return super.povUp();
  }

  /**
   * Gets the trigger for the D-Pad down direction.
   *
   * @return a {@link Trigger} object for the D-Pad down direction.
   */
  public Trigger getDPadTriggerDown() {
    return super.povDown();
  }

  /**
   * Gets the trigger for the D-Pad left direction.
   *
   * @return a {@link Trigger} object for the D-Pad left direction.
   */
  public Trigger getDPadTriggerLeft() {
    return super.povLeft();
  }

  /**
   * Gets the trigger for the D-Pad right direction.
   *
   * @return a {@link Trigger} object for the D-Pad right direction.
   */
  public Trigger getDPadTriggerRight() {
    return super.povRight();
  }
}
