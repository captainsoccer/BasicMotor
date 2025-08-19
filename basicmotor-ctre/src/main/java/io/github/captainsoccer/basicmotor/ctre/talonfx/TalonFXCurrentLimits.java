package io.github.captainsoccer.basicmotor.ctre.talonfx;

import io.github.captainsoccer.basicmotor.gains.CurrentLimits;

/**
 * This class represents the current limits for a TalonFX motor controller.
 * It implements the {@link CurrentLimits} interface and provides specific current limits.
 *
 * @param statorCurrentLimit The maximum current output of the motor controller (in amps).
 *                           If this is zero, there will be no stator current limit.
 * @param supplyCurrentLimit The maximum current draw of the motor controller (in amps).
 *                           If this is zero, there will be no supply current limit.
 * @param supplyLowerTime    The time (in seconds) that the motor can stay at supply current limit.
 *                           If this or supplyLowerLimit is zero, it will be ignored.
 * @param supplyLowerLimit   The current the motor drops to after the supply current limit is
 *                           reached for supplyLowerTime.
 *                           If this or supplyLowerTime is zero, it will be ignored.
 */
public record TalonFXCurrentLimits(int statorCurrentLimit, int supplyCurrentLimit, double supplyLowerTime,
                                   int supplyLowerLimit) implements CurrentLimits {
    /**
     * Checks the validity of the current limits.
     * @param statorCurrentLimit The maximum current output of the motor controller (in amps).
     *                           If this is zero, there will be no stator current limit.
     * @param supplyCurrentLimit The maximum current draw of the motor controller (in amps).
     *                           If this is zero, there will be no supply current limit.
     * @param supplyLowerTime    The time (in seconds) that the motor can stay at supply current limit.
     *                           If this or supplyLowerLimit is zero, it will be ignored.
     * @param supplyLowerLimit   The current the motor drops to after the supply current limit is
     *                           reached for supplyLowerTime.
     *                           If this or supplyLowerTime is zero, it will be ignored.
     * @throws IllegalArgumentException if any of the limits are negative.
     */
    public TalonFXCurrentLimits {
        if (statorCurrentLimit < 0) {
            throw new IllegalArgumentException("Stator current limit must be non-negative.");
        }

        if (supplyCurrentLimit < 0) {
            throw new IllegalArgumentException("Supply current limit must be non-negative.");
        }

        if (supplyLowerTime < 0) {
            throw new IllegalArgumentException("Supply lower time must be non-negative.");
        }

        if (supplyLowerLimit < 0) {
            throw new IllegalArgumentException("Supply lower limit must be non-negative.");
        }
    }

    /**
     * Creates a current limit with the given values.
     *
     * @param statorCurrentLimit The maximum current output of the motor controller (in amps).
     *                           If this is zero, there will be no stator current limit.
     * @param supplyCurrentLimit The maximum current draw of the motor controller (in amps).
     *                           If this is zero, there will be no supply current limit.
     */
    public TalonFXCurrentLimits(int statorCurrentLimit, int supplyCurrentLimit) {
        this(statorCurrentLimit, supplyCurrentLimit, 0, 0);
    }

    @Override
    public int getCurrentLimit() {
        return statorCurrentLimit;
    }
}
