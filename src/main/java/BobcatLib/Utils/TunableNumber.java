package BobcatLib.Utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A wrapper around {@link LoggedNetworkNumber} that adds change detection
 * and the ability to reset to a default value.
 * <p>
 * Useful for exposing tunable numeric constants (like PID gains) that
 * can be monitored and modified at runtime via NetworkTables.
 */
public class TunableNumber extends LoggedNetworkNumber {

    /** The default value assigned at creation. Used for resetting. */
    private final double defaultValue;

    /** The last known value, used to detect changes. */
    private double lastValue;

    /**
     * Constructs a new TunableNumber with a given key and default value.
     *
     * @param key          The NetworkTables key (typically a SmartDashboard path).
     * @param defaultValue The default numeric value to initialize and reset to.
     */
    public TunableNumber(String key, double defaultValue) {
        super(key, defaultValue);
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
    }

    /**
     * Gets the current value stored in this TunableNumber.
     *
     * @return The latest numeric value from NetworkTables.
     */
    public double get() {
        return super.get();
    }

    /**
     * Checks if the value has changed since the last time this method was called.
     * <p>
     * Internally updates the stored last value when a change is detected.
     *
     * @return True if the value has changed since last checked; false otherwise.
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastValue) {
            lastValue = currentValue;
            return true;
        }
        return false;
    }

    /**
     * Resets the value back to the original default and updates the last known value.
     */
    public void reset() {
        super.set(defaultValue);
        lastValue = defaultValue;
    }
}
