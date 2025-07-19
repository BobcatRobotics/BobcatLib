package BobcatLib.Hardware.Motors;

/**
 * Represents a software-enforced limit that can be enabled or disabled.
 * Used for constraining motor or mechanism behavior programmatically.
 */
public class SoftwareLimits {
    private double limit;
    private boolean enable;

    /**
     * Constructs a disabled software limit with no value.
     */
    public SoftwareLimits() {
        disable();
    }

    /**
     * Constructs and enables a software limit with the specified value.
     *
     * @param limit The limit value to enforce.
     */
    public SoftwareLimits(double limit) {
        this.limit = limit;
        enable();
    }

    /**
     * Returns the current limit value.
     *
     * @return The numeric limit.
     */
    public double get() {
        return limit;
    }

    /**
     * Returns whether the limit is currently enforced.
     *
     * @return {@code true} if enabled, {@code false} otherwise.
     */
    public boolean isEnabled() {
        return enable;
    }

    /**
     * Disables the software limit, allowing unrestricted operation.
     */
    public void disable() {
        this.enable = false;
    }

    /**
     * Enables the software limit, enforcing the configured value.
     */
    public void enable() {
        this.enable = true;
    }
}
