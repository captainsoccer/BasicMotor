package com.basicMotor.gains.currentLimits;

/**
 * This class represents the current limits for a TalonSRX motor controller.
 * It implements the CurrentLimits interface and provides methods to get the current limits.
 */
public class CurrentLimitsTalonSRX implements  CurrentLimits {
    /**
     * The continuous current limit for the TalonSRX motor controller.
     * This is the maximum current that the motor can draw continuously.
     * If {@link #peakCurrentLimit} or {@link #peakCurrentDuration} is not set,
     * this will also be used as the peak current limit.
     */
    private final int continuousCurrentLimit;
    /**
     * The peak current limit for the TalonSRX motor controller.\
     * This is the maximum current that the motor can draw for a short duration.
     * If this is not set, the {@link #continuousCurrentLimit} will be used as the peak current limit.
     * The duration fo the peak current limit is defined by {@link #peakCurrentDuration}.
     */
    private final int peakCurrentLimit;
    /**
     * The duration for which the peak current limit can be sustained.
     * This is the time in seconds that the motor can draw the {@link #peakCurrentLimit} before it is limited to the {@link #continuousCurrentLimit}.
     */
    private final int peakCurrentDuration;

    /**
     * Creates a CurrentLimitsTalonSRX instance with the provided current limits.
     * This constructor sets the continuous current limit, peak current limit, and peak current duration.
     * All the current limits are in supply current, not motor current. (Amps)
     * @param continuousCurrentLimit The continuous current limit for the TalonSRX motor controller.
     *                               If the peak current limit or peak current duration is not set,
     *                               This will also be used as the peak current limit.
     * @param peakCurrentLimit The peak current limit for the TalonSRX motor controller.
     *                         This is the maximum current that the motor can draw for a short duration.
     *                         If this is not set, the continuous current limit will be used as the peak current limit.
     *                         The duration is the peak current duration.
     * @param peakCurrentDuration The duration for which the peak current limit can be sustained.
     *                            This is the time in seconds that the motor can draw the peak current limit before it is limited to the continuous current limit.
     */
    public CurrentLimitsTalonSRX(int continuousCurrentLimit, int peakCurrentLimit, int peakCurrentDuration) {
        this.continuousCurrentLimit = continuousCurrentLimit;
        this.peakCurrentLimit = peakCurrentLimit;
        this.peakCurrentDuration = peakCurrentDuration;
    }

    /**
     * Creates a CurrentLimitsTalonSRX instance with the provided continuous current limit.
     * This sets the current limit of the talonSRX motor to the provided value.
     * @param currentLimit The current limit for the TalonSRX motor controller.
     *                     This is the maximum current that the motor can draw.
     */
    public CurrentLimitsTalonSRX(int currentLimit){
        this(currentLimit, 0, 0);
    }

    @Override
    public int getCurrentLimit() {
        return continuousCurrentLimit;
    }

    /**
     * Gets the peak current limit for the TalonSRX motor controller.
     * This is the maximum current that the motor can draw for a short duration.
     * The duration of the peak current limit is defined by {@link #getPeakCurrentDuration()}.
     * @return the peak current limit in amps.
     */
    public int getPeakCurrentLimit() {
        return peakCurrentLimit;
    }

    /**
     * Gets the duration for which the peak current limit can be sustained.
     * This is the time in seconds that the motor can draw the peak current limit before it is limited to the continuous current limit.
     * @return the peak current duration in seconds.
     */
    public int getPeakCurrentDuration() {
        return peakCurrentDuration;
    }
}
