package io.github.captainsoccer.basicmotor.rev;

import io.github.captainsoccer.basicmotor.gains.CurrentLimits;

/**
 * This class represents the current limits of a Spark Base motor controller.
 * It can be used on a Spark Max or a Spark Flex motor controller.
 * It has specific current limits offered by the Spark Base motor controller.
 * Use this for Spark Max and Spark Flex motor controllers instead of the generic {@link CurrentLimits} interface.
 */
public class SparkCurrentLimits implements CurrentLimits {

    /**
     * The maximum current output of the motor controller (in amps) when not in stall.
     * If the stall current limit is not set, the motor will use this as the current limit at all times.
     * Free speed is any speed above {@link #freeSpeedRPM}.
     * If the motor reaches this current limit, it will lower the output voltage to prevent exceeding this limit.
     */
    private final int freeSpeedCurrentLimit;

    /**
     * The maximum current output of the motor controller (in amps) while in stall.
     * If this value is not set, the motor will use only the {@link #freeSpeedCurrentLimit} as the current limit at all times.
     * Stall is defined as any speed below {@link #freeSpeedRPM}.
     * If the motor reaches this current limit, it will lower the output voltage to prevent exceeding this limit.
     */
    private final int stallCurrentLimit;

    /**
     * The secondary current limit of the motor controller (in amps).
     * If the motor reaches this current limit, it will stop for a short time.
     * This is output current limit, not supply current limit.
     * Useful for small motors like the Neo 550, which can burn out quickly.
     */
    private final int secondaryCurrentLimit;

    /**
     * The speed of the mechanism in RPM that is considered as free speed.
     * Any speed below this speed is considered stall and the stall current limit is applied.
     * Otherwise, the free speed current limit is applied.
     * This value is in the motors rotations per minute (RPM).
     * (no gear ratio, no unit conversion).
     * If this value is zero, the motor will linearly interpolate between the free speed current limit and the stall current limit.
     */
    private final int freeSpeedRPM;

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

    /**
     * Gets the stall current limit of the motor controller.
     * If it is not set, the motor will use only the {@link #getCurrentLimit()} as the current limit at all times.
     *
     * @return the current limit of the motor controller while in stall (in amps).
     */
    public int getStallCurrentLimit() {
        return stallCurrentLimit;
    }

    /**
     * Gets the free speed of the motor controller.
     * Any speed above this speed is considered free speed and the free speed current limit will be applied.
     * If the motor speed is below this speed, the motor is considered in stall and the stall current limit is applied.
     *
     * @return The free speed of the motor controller in RPM (revolutions per minute).
     */
    public int getFreeSpeedRPM() {
        return freeSpeedRPM;
    }

    /**
     * Gets the secondary current limit of the motor controller
     *
     * @return The secondary current limit of the motor controller (in amps)
     */
    public int getSecondaryCurrentLimit() {
        return secondaryCurrentLimit;
    }
}
