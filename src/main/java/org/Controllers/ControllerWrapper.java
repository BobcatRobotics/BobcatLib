package org.Controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Controller Wrapper */
public interface ControllerWrapper {

  /**
   * Gets the axis value for LeftY component.
   *
   * @return The axis value left y-component.
   */
  public default double getLeftYAxis() {
    return 0;
  }

  /**
   * Gets the axis value for Left X component.
   *
   * @return The axis value left X component.
   */
  public default double getLeftXAxis() {
    return 0;
  }

  /**
   * Gets the axis value for Right component.
   *
   * @return The axis value right y-component.
   */
  public default double getRightYAxis() {
    return 0;
  }

  /**
   * Gets the axis value for Right X component.
   *
   * @return The axis value right X component.
   */
  public default double getRightXAxis() {
    return 0;
  }

  /**
   * Get Left Trigger
   *
   * @return Trigger
   */
  public default Trigger getLeftTrigger() {
    return null;
  }
  /**
   * Get Right Trigger
   *
   * @return Trigger
   */
  public default Trigger getRightTrigger() {
    return null;
  }
  /**
   * Get Left Bumper
   *
   * @return Trigger
   */
  public default Trigger getLeftBumper() {
    return null;
  }
  /**
   * Get RightBumper
   *
   * @return Trigger
   */
  public default Trigger getRightBumper() {
    return null;
  }
  /**
   * Get YorTriangle
   *
   * @return Trigger
   */
  public default Trigger getYorTriangle() {
    return null;
  }
  /**
   * getBorCircle
   *
   * @return Trigger
   */
  public default Trigger getBorCircle() {
    return null;
  }

  /**
   * Get A Or Cross
   *
   * @return Trigger
   */
  public default Trigger getAorCross() {
    return null;
  }
  /**
   * getXorSquare
   *
   * @return Trigger
   */
  public default Trigger getXorSquare() {
    return null;
  }
  /**
   * getDPadTriggerUp
   *
   * @return Trigger
   */
  public default Trigger getDPadTriggerUp() {
    return null;
  }
  /**
   * getDPadTriggerDown
   *
   * @return Trigger
   */
  public default Trigger getDPadTriggerDown() {
    return null;
  }
  /**
   * getDPadTriggerLeft
   *
   * @return Trigger
   */
  public default Trigger getDPadTriggerLeft() {
    return null;
  }
  /**
   * getDPadTriggerRight
   *
   * @return Trigger
   */
  public default Trigger getDPadTriggerRight() {
    return null;
  }

  /**
   * getTopButton
   *
   * @return Trigger
   */
  public default Trigger getTopButton() {
    return null;
  }
}
