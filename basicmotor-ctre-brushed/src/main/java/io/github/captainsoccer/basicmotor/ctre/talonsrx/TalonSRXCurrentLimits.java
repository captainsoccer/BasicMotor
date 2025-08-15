package io.github.captainsoccer.basicmotor.ctre.talonsrx;

import io.github.captainsoccer.basicmotor.gains.CurrentLimits;

public record TalonSRXCurrentLimits(int continuousCurrentLimit, int peakCurrentLimit,
                                    int peakCurrentDuration) implements CurrentLimits {
    /**
     * Creates a CurrentLimitsTalonSRX instance with the provided current limits.
     * This constructor sets the continuous current limit, peak current limit, and peak current duration.
     * All the current limits are in supply current, not motor current. (Amps)
     *
     * @param continuousCurrentLimit The continuous current limit for the TalonSRX motor controller.
     *                               If the peak current limit or peak current duration is not set,
     *                               This will also be used as the peak current limit.
     * @param peakCurrentLimit       The peak current limit for the TalonSRX motor controller.
     *                               This is the maximum current that the motor can draw for a short duration.
     *                               If this is not set, the continuous current limit will be used as the peak current limit.
     *                               The duration is the peak current duration.
     * @param peakCurrentDuration    The duration for which the peak current limit can be sustained.
     *                               This is the time in seconds that the motor can draw the peak current limit before it is limited to the continuous current limit.
     */
    public TalonSRXCurrentLimits(int continuousCurrentLimit, int peakCurrentLimit, int peakCurrentDuration) {
        if (continuousCurrentLimit < 0) {
            throw new IllegalArgumentException("Continuous current limit must be non-negative.");
        }
        this.continuousCurrentLimit = continuousCurrentLimit;

        if (peakCurrentLimit < 0) {
            throw new IllegalArgumentException("Peak current limit must be non-negative.");
        }
        this.peakCurrentLimit = peakCurrentLimit;

        if (peakCurrentDuration < 0) {
            throw new IllegalArgumentException("Peak current duration must be non-negative.");
        }
        this.peakCurrentDuration = peakCurrentDuration;
    }

    /**
     * Creates a CurrentLimitsTalonSRX instance with the provided continuous current limit.
     * This sets the current limit of the talonSRX motor to the provided value.
     *
     * @param currentLimit The current limit for the TalonSRX motor controller.
     *                     This is the maximum current that the motor can draw.
     */
    public TalonSRXCurrentLimits(int currentLimit) {
        this(currentLimit, 0, 0);
    }

    @Override
    public int getCurrentLimit() {
        return continuousCurrentLimit;
    }
}
