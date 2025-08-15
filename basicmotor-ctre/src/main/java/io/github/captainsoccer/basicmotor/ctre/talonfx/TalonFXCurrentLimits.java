package io.github.captainsoccer.basicmotor.ctre.talonfx;

import io.github.captainsoccer.basicmotor.gains.CurrentLimits;

/**
 * This class represents the current limits for a TalonFX motor controller.
 * It implements the {@link CurrentLimits} interface and provides specific current limits.
 */
public record TalonFXCurrentLimits(int statorCurrentLimit, int supplyCurrentLimit, double supplyLowerTime,
                                   int supplyLowerLimit) implements CurrentLimits {
    /**
     * Creates a current limit with the given values
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
    public TalonFXCurrentLimits(int statorCurrentLimit, int supplyCurrentLimit, double supplyLowerTime, int supplyLowerLimit) {
        if (statorCurrentLimit < 0) {
            throw new IllegalArgumentException("Stator current limit must be non-negative.");
        }
        this.statorCurrentLimit = statorCurrentLimit;

        if (supplyCurrentLimit < 0) {
            throw new IllegalArgumentException("Supply current limit must be non-negative.");
        }
        this.supplyCurrentLimit = supplyCurrentLimit;

        if (supplyLowerTime < 0) {
            throw new IllegalArgumentException("Supply lower time must be non-negative.");
        }
        this.supplyLowerTime = supplyLowerTime;

        if (supplyLowerLimit < 0) {
            throw new IllegalArgumentException("Supply lower limit must be non-negative.");
        }
        this.supplyLowerLimit = supplyLowerLimit;
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

    /**
     * Gets the maximum current draw of the motor controller (in amps).
     *
     * @return The maximum current draw of the motor controller (in amps).
     */
    @Override
    public int supplyCurrentLimit() {
        return supplyCurrentLimit;
    }

    /**
     * Gets the time (in seconds) that the motor can stay at supply current limit before it lowers to
     * supply lower limit.
     *
     * @return The time (in seconds) that the motor can stay at supply current limit.
     */
    @Override
    public double supplyLowerTime() {
        return supplyLowerTime;
    }

    /**
     * Gets the current the motor drops to after the supply current limit is reached for supplyLowerTime.
     *
     * @return The current the motor drops to after the supply current limit is reached for
     * supplyLowerTime.
     */
    @Override
    public int supplyLowerLimit() {
        return supplyLowerLimit;
    }
}
