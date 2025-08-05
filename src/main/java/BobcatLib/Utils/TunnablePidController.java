package BobcatLib.Utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 * A tunable PID controller configuration helper using {@link TunableNumber}
 * and integrated with CTRE Phoenix6 configuration system.
 * <p>
 * This class exposes PID constants (and feedforward terms) as
 * tunable SmartDashboard values.
 */
public class TunnablePidController implements ParentConfiguration {

    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;
    private final TunableNumber kS;
    private final TunableNumber kV;
    private final TunableNumber kA;
    private final TunableNumber kG;
    private final String pidName;

    /** Optional: Gravity type setting (not used yet) */
    private GravityTypeValue gravityType;

    /**
     * Constructs a new TunnablePidController with a given name prefix.
     *
     * @param name SmartDashboard path prefix for the tunable constants.
     */
    public TunnablePidController(String name) {
        this.pidName = name;
        this.kP = new TunableNumber(pidName + "/Tunable/kP", 0.0);
        this.kI = new TunableNumber(pidName + "/Tunable/kI", 0.0);
        this.kD = new TunableNumber(pidName + "/Tunable/kD", 0.0);
        this.kS = new TunableNumber(pidName + "/Tunable/kS", 0.0);
        this.kV = new TunableNumber(pidName + "/Tunable/kV", 0.0);
        this.kA = new TunableNumber(pidName + "/Tunable/kA", 0.0);
        this.kG = new TunableNumber(pidName + "/Tunable/kG", 0.0);
        this.gravityType = GravityTypeValue.Elevator_Static; // default, can be changed externally
    }

    /**
     * Returns true if any of the tunable values have changed since last checked.
     *
     * @return whether any of the PID or feedforward values have changed.
     */
    public boolean hasChanged() {
        return kP.hasChanged() || kI.hasChanged() || kD.hasChanged() ||
               kS.hasChanged() || kV.hasChanged() || kA.hasChanged() || kG.hasChanged();
    }

    /**
     * Gets the current value of kP.
     */
    public double getKP() {
        return kP.get();
    }

    /**
     * Gets the current value of kI.
     */
    public double getKI() {
        return kI.get();
    }

    /**
     * Gets the current value of kD.
     */
    public double getKD() {
        return kD.get();
    }

    /**
     * Gets the current value of kS (static friction feedforward).
     */
    public double getKS() {
        return kS.get();
    }

    /**
     * Gets the current value of kV (velocity feedforward).
     */
    public double getKV() {
        return kV.get();
    }

    /**
     * Gets the current value of kA (acceleration feedforward).
     */
    public double getKA() {
        return kA.get();
    }

    /**
     * Gets the current value of kG (gravity feedforward).
     */
    public double getKG() {
        return kG.get();
    }

    /**
     * Gets the current GravityType setting.
     */
    public GravityTypeValue getGravityType() {
        return gravityType;
    }

    /**
     * Sets the GravityType to use (e.g. Elevator_Static, Arm_Cosine).
     */
    public void setGravityType(GravityTypeValue gravityType) {
        this.gravityType = gravityType;
    }

    /**
     * Stub for CTRE configuration serialization. Not implemented yet.
     */
    @Override
    public String serialize() {
        throw new UnsupportedOperationException("Unimplemented method 'serialize'");
    }

    /**
     * Stub for CTRE configuration deserialization. Not implemented yet.
     */
    @Override
    public StatusCode deserialize(String str) {
        throw new UnsupportedOperationException("Unimplemented method 'deserialize'");
    }
}
