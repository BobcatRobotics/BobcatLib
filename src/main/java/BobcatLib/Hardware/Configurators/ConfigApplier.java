package BobcatLib.Hardware.Configurators;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Utility class for applying configuration settings to hardware devices
 * with built-in retry logic. This helps ensure settings are reliably applied,
 * especially when using Phoenix 6 motor controllers and devices.
 */
public class ConfigApplier {

  /**
   * Attempts to apply a configuration using a generic retry mechanism.
   * Retries up to {@code retries} times until the {@code isSuccess} function
   * returns true.
   *
   * @param applier   a {@link Supplier} that applies a configuration and returns a result
   * @param isSuccess a {@link Function} that checks if the result indicates success
   * @param retries   the number of times to retry the application if unsuccessful
   * @param <Result>  the result type returned by the applier
   * @return {@code true} if the config was successfully applied within the given retries;
   *         {@code false} otherwise
   */
  protected static <Result> boolean attempt(Supplier<Result> applier,
                                            Function<Result, Boolean> isSuccess,
                                            int retries) {
    for (int i = 0; i < retries; i++) {
      Result result = applier.get();

      if (isSuccess.apply(result)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Attempts to apply a configuration that returns a {@link StatusCode},
   * retrying up to 10 times or until {@link StatusCode#isOK()} returns true.
   *
   * <p>This is a Phoenix 6-specific convenience overload of the generic {@link #attempt} method.</p>
   *
   * @param applier a {@link Supplier} that applies a config and returns a {@link StatusCode}
   * @return {@code true} if the config was successfully applied; {@code false} otherwise
   */
  protected static boolean attempt(Supplier<StatusCode> applier) {
    return attempt(applier, StatusCode::isOK, 10);
  }
}
