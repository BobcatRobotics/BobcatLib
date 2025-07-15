package org.Controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Represents an immutable controller input with deadband and slew rate limiting applied.
 *
 * <p>This class processes a raw input value by first applying a deadband and then applying slew
 * rate limiting to smooth out sudden changes. All fields are immutable, and the processing is done
 * once during construction.
 */
public final class ControllerInput {
  /** The raw input value received from the controller. */
  private final double input;

  /** The deadband threshold value below which input is ignored. */
  private final double stickDeadband;

  /** The maximum rate of change allowed for the input (units per second). */
  private final double slewRate;

  /** The input value after deadband has been applied. */
  private final double inputDeadband;

  /** The input value after deadband and slew rate limiting have been applied. */
  private final double inputLimited;

  /**
   * Constructs an immutable {@code ControllerInput} instance.
   *
   * @param input The raw input value.
   * @param stickDeadband The deadband threshold to apply.
   * @param slewRate The slew rate limit (units per second).
   */
  public ControllerInput(double input, double stickDeadband, double slewRate) {
    this.input = input;
    this.stickDeadband = stickDeadband;
    this.slewRate = slewRate;

    // Apply deadband to the input.
    this.inputDeadband = MathUtil.applyDeadband(this.input, stickDeadband);

    // Apply slew rate limiting using a temporary limiter (for initial value only).
    SlewRateLimiter limiter = new SlewRateLimiter(this.slewRate);
    this.inputLimited = limiter.calculate(this.input);
  }

  /**
   * Constructs a {@code ControllerInput} with a default slew rate of 3.0 units/second.
   *
   * @param input The raw input value.
   * @param stickDeadband The deadband threshold to apply.
   */
  public ControllerInput(double input, double stickDeadband) {
    this(input, stickDeadband, 3.0);
  }

  /**
   * Returns the raw input value.
   *
   * @return The raw controller input.
   */
  public double getInput() {
    return input;
  }

  /**
   * Returns the deadband threshold used.
   *
   * @return The stick deadband value.
   */
  public double getStickDeadband() {
    return stickDeadband;
  }

  /**
   * Returns the slew rate limit used.
   *
   * @return The slew rate in units per second.
   */
  public double getSlewRate() {
    return slewRate;
  }

  /**
   * Returns the input after applying deadband.
   *
   * @return The deadbanded input value.
   */
  public double getInputDeadband() {
    return inputDeadband;
  }

  /**
   * Returns the input after applying both deadband and slew rate limiting.
   *
   * @return The limited input value.
   */
  public double getInputLimited() {
    return inputLimited;
  }
}
