package io.github.captainsoccer.basicmotor.gains;

/**
 * The basic interface for current limits in a motor controller.
 * Only supports output current limit because not every motor controller supports other kinds of current limits.
 */
public interface CurrentLimits {
    /**
     * Returns the current limit of the motor output.
     * (In Amperes).
     * (i.e. stator current limit).
     * The motor will lower the output voltage to prevent exceeding this limit.
     *
     * @return the stator current limit in Amperes.
     */
    int getCurrentLimit();
}
