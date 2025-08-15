package io.github.captainsoccer.basicmotor.rev;

import io.github.captainsoccer.basicmotor.gains.CurrentLimits;

/**
 * This class represents the current limits of a Spark Base motor controller.
 * It can be used on a Spark Max or a Spark Flex motor controller.
 * It has specific current limits offered by the Spark Base motor controller.
 * Use this for Spark Max and Spark Flex motor controllers instead of the generic {@link CurrentLimits} interface.
 */
public record SparkCurrentLimits(int freeSpeedCurrentLimit, int stallCurrentLimit, int freeSpeedRPM,
                                 int secondaryCurrentLimit) implements CurrentLimits {

    /**
     * Creates a current limit with the given values
     *
     * @param freeSpeedCurrentLimit The maximum current output of the motor controller (in amps) when
     *                              not in stall.
     *                              If the stall current limit is not set, the motor will use this as the current limit at all times.
     *                              If this and stallCurrentLimit are zero, the motor will not limit the current output.
     * @param stallCurrentLimit     The maximum current output of the motor controller (in amps) while in stall.
     *                              If this value is zero, the motor will use only the {@link #freeSpeedCurrentLimit} as the current limit at all times.
     * @param freeSpeedRPM          The speed of the mechanism in RPM that is considered as free speed.
     *                              If the motor speed is below this speed, the motor is considered in stall and the stall current limit is applied.
     *                              Otherwise, the free speed current limit is applied.
     *                              This value is in the motors rotations per minute (RPM), not the mechanisms rotations per minute (RPM).
     *                              If this value is zero, the motor will linearly interpolate between the free speed current limit and the stall current limit.
     * @param secondaryCurrentLimit The secondary current limit of the motor controller (in amps).
     *                              When the motor reaches this current limit, it will stop for a short time.
     */
    public SparkCurrentLimits(int freeSpeedCurrentLimit, int stallCurrentLimit, int freeSpeedRPM, int secondaryCurrentLimit) {

        if (freeSpeedCurrentLimit < 0) {
            throw new IllegalArgumentException("Free speed current limit must be non-negative.");
        }
        this.freeSpeedCurrentLimit = freeSpeedCurrentLimit;

        if (stallCurrentLimit < 0) {
            throw new IllegalArgumentException("Stall current limit must be non-negative.");
        }
        this.stallCurrentLimit = stallCurrentLimit;

        if (freeSpeedRPM < 0) {
            throw new IllegalArgumentException("Free speed RPS must be non-negative.");
        }
        this.freeSpeedRPM = freeSpeedRPM;

        if (secondaryCurrentLimit < 0) {
            throw new IllegalArgumentException("Secondary current limit must be non-negative.");
        }
        this.secondaryCurrentLimit = secondaryCurrentLimit;
    }

    /**
     * Creates a current limit with the given values
     *
     * @param freeSpeedCurrentLimit The maximum current output of the motor controller (in amps) when
     *                              not in stall.
     *                              If this and stallCurrentLimit are zero, the motor will not limit the current output.
     * @param stallCurrentLimit     The maximum current output of the motor controller (in amps) while in
     *                              stall.
     *                              If this value is zero, the motor will use only the {@link #freeSpeedCurrentLimit} as the current limit at all times.
     * @param freeSpeedRPM          The speed of the mechanism in RPM that is considered as free speed.
     *                              If the motor speed is below this speed, the motor is considered in stall and the stall current limit is applied.
     *                              Otherwise, the free speed current limit is applied.
     *                              This value is in the motors rotations per minute (RPM), not the mechanisms rotations per minute (RPM).
     *                              If this value is zero, the motor will linearly interpolate between the free speed current limit and the stall current limit.
     */
    public SparkCurrentLimits(int freeSpeedCurrentLimit, int stallCurrentLimit, int freeSpeedRPM) {
        this(freeSpeedCurrentLimit, stallCurrentLimit, freeSpeedRPM, 0);
    }

    /**
     * Creates a current limit with the given values
     *
     * @param currentLimit          The maximum current output of the motor controller (in amps).
     *                              If this is zero, there will be no current limit.
     * @param secondaryCurrentLimit The secondary current limit of the motor controller (in amps).
     *                              When the motor reaches this current limit, it will stop for a short time.
     *                              If this value is zero, the motor will not limit the current output.
     */
    public SparkCurrentLimits(int currentLimit, int secondaryCurrentLimit) {
        this(currentLimit, 0, 0, secondaryCurrentLimit);
    }

    /**
     * Creates a current limit with the given values
     *
     * @param currentLimit The maximum current output of the motor controller (in amps).
     */
    public SparkCurrentLimits(int currentLimit) {
        this(currentLimit, 0, 0, 0);
    }

    @Override
    public int getCurrentLimit() {
        return freeSpeedCurrentLimit;
    }
}
