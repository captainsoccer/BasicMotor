package io.github.captainsoccer.basicmotor.ctre.talonsrx;

import io.github.captainsoccer.basicmotor.gains.CurrentLimits;

/**
 * This class represents the current limits for a TalonSRX motor controller.
 * It implements the {@link CurrentLimits} interface and provides specific current limits.
 * Use this for TalonSRX motor controllers instead of the generic {@link CurrentLimits}
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
public record TalonSRXCurrentLimits(int continuousCurrentLimit, int peakCurrentLimit,
                                    int peakCurrentDuration) implements CurrentLimits {
    /**
     * Validates the current limits for the TalonSRX motor controller.
     */
    public TalonSRXCurrentLimits {
        if (continuousCurrentLimit < 0) {
            throw new IllegalArgumentException("Continuous current limit must be non-negative.");
        }

        if (peakCurrentLimit < 0) {
            throw new IllegalArgumentException("Peak current limit must be non-negative.");
        }

        if (peakCurrentDuration < 0) {
            throw new IllegalArgumentException("Peak current duration must be non-negative.");
        }
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
