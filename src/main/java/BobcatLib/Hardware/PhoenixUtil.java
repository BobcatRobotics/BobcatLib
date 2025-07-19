package BobcatLib.Hardware;


import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;
/** 
 * Phoenix utility too attempt to apply settings a given number of times.
 */
public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) {
        break;
      }
    }
  }
}
