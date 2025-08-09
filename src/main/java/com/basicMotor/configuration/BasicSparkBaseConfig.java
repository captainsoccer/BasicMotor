package com.basicMotor.configuration;

import com.basicMotor.gains.currentLimits.CurrentLimitsSparkBase;

/**
 * This class represents the configuration for a basic spark base motor controller.
 * It extends the BasicMotorConfig class and provides specific configurations for spark base motors.
 * Use this class to use a spark max or a spark flex motor controller.
 * (Neo 1.1, Neo 550, Neo vortex).
 * See the <a href="https://github.com/captainsoccer/MotorUtils/wiki">wiki</a> //TODO: add wiki link
 * for more information on how to use this class.
 */
public class BasicSparkBaseConfig extends BasicMotorConfig {

    /**
     * The current limit configuration for the spark base motor controller.
     */
    public CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

    /**
     * The external encoder configuration for the spark base motor controller.
     * If you want to use an external encoder (connected directly to the motor controller), you can use this to simplify the process.
     */
    public ExternalEncoderConfig externalEncoderConfig = new ExternalEncoderConfig();

    /**
     * The absolute encoder configuration for the spark base motor controller.
     * If you want to use an Absolute encoder (connected directly to the motor controller), you can use this to simplify the process.
     */
    public AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();

    //check if using either external or absolute encoder
    @Override
    public boolean usingExternalEncoder() {
        return externalEncoderConfig.useExternalEncoder || absoluteEncoderConfig.useAbsoluteEncoder;
    }

    /**
     * Creates a copy of the BasicSparkBaseConfig.
     * This method creates a new instance of BasicSparkBaseConfig and copies the values from the
     * current instance to the new instance.
     *
     * @return A new BasicSparkBaseConfig object with the same values as this instance
     */
    @Override
    public BasicSparkBaseConfig copy() {
        BasicSparkBaseConfig copy = new BasicSparkBaseConfig();

        // Copy the basic motor configuration
        super.copy(copy);

        copy.currentLimitConfig = this.currentLimitConfig.copy();
        copy.externalEncoderConfig = this.externalEncoderConfig.copy();
        copy.absoluteEncoderConfig = this.absoluteEncoderConfig.copy();

        return copy;
    }

    /**
     * Handles configuration for the currentLimits, It has current limits specifically tailored to the spark base motor controllers.
     */
    public static class CurrentLimitConfig {
        /**
         * The maximum current output (i.e. applied current and not current draw) of the motor controller while in free speed (in amps).
         * Free speed can be defined in {@link #freeSpeedRPM}.
         * Use this to limit the current draw of the motor controller when it is not stalled.
         */
        public int freeSpeedCurrentLimit = 0;
        /**
         * The maximum current output (i.e. applied current and not current draw) of the motor controller while in stall (in amps).
         * Anything below the free speed is considered stall, free speed can be defined in {@link #freeSpeedRPM}
         */
        public int stallCurrentLimit = 0;
        /**
         * The free speed of the motor controller (in RPM).
         * Any speed above this speed is considered free speed and the free speed current limit will be applied.
         * If the motor speed is below this speed, the motor is considered in stall and the stall current limit is applied.
         * This value is in the motors rotations per minute (RPM), not the mechanisms rotations per minute (RPM).
         * (so no gear ratio, no unit conversion).
         */
        public int freeSpeedRPM = 0;
        /**
         * The max current output of the motor controller (in amps).
         * If the current output of the motor exceeds this limit, the motor will stop for a short time.
         * This should be only used for extreme cases where the motor is drawing too much current.
         * This is especially useful for small motors like the neo 550, which can burn out quickly.
         */
        public int secondaryCurrentLimit = 0;

        /**
         * Creates the current limit configuration with the given values
         *
         * @return The current limits of the motor controller
         */
        public CurrentLimitsSparkBase getCurrentLimits() {
            return new CurrentLimitsSparkBase(
                    freeSpeedCurrentLimit, stallCurrentLimit, freeSpeedRPM, secondaryCurrentLimit);
        }

        /**
         * Creates a copy of the current limit configuration.
         *
         * @return A new CurrentLimitConfig object with the same values
         */
        public CurrentLimitConfig copy() {
            CurrentLimitConfig copy = new CurrentLimitConfig();

            copy.freeSpeedCurrentLimit = this.freeSpeedCurrentLimit;
            copy.stallCurrentLimit = this.stallCurrentLimit;
            copy.freeSpeedRPM = this.freeSpeedRPM;
            copy.secondaryCurrentLimit = this.secondaryCurrentLimit;

            return copy;
        }

        /**
         * Creates a CurrentLimitConfig from the given CurrentLimitsSparkBase.
         * This is useful for converting existing current limits to the configuration format.
         *
         * @param currentLimits The CurrentLimitsSparkBase to convert
         * @return A new CurrentLimitConfig object with the values from the CurrentLimitsSparkBase
         */
        public static CurrentLimitConfig fromCurrentLimits(CurrentLimitsSparkBase currentLimits) {
            var currentLimitConfig = new CurrentLimitConfig();

            currentLimitConfig.freeSpeedCurrentLimit = currentLimits.getCurrentLimit();
            currentLimitConfig.stallCurrentLimit = currentLimits.getStallCurrentLimit();
            currentLimitConfig.freeSpeedRPM = currentLimits.getFreeSpeedRPM();
            currentLimitConfig.secondaryCurrentLimit = currentLimits.getSecondaryCurrentLimit();

            return currentLimitConfig;
        }
    }

    /**
     * Handles the configuration parameters for the external encoder
     */
    public static class ExternalEncoderConfig {
        /**
         * Should the motor use an external encoder (connected directly to the motor controller)?
         * Change this to true if you want to use an external encoder.
         * If the encoder is a through-bore encoder and is connected to a spark flex motor controller,
         * you can use the absolute encoder to reset the relative encoder then use the higher quality relative encoder.
         * To do that, set this flag to true and configure the absolute encoder in {@link BasicSparkBaseConfig#absoluteEncoderConfig}.
         */
        public boolean useExternalEncoder = false;

        /**
         * Is the external encoder inverted (does it count in the opposite direction of the motor)?
         * Unlike motors, external encoders default positive direction is usually clockwise.
         */
        public boolean inverted = false;

        /**
         * The number that the reading of the external encoder should be multiplied by to get the motor position.
         * In most cases, the sensor will be in a reduction to the motor, so this value will be greater than 1.0.
         * If the sensor is mounted not directly on the mechanism, use also {@link #mechanismToSensorRatio}.
         * In Most cases, this value will be the same as {@link MotorConfig#gearRatio}.
         */
        public double sensorToMotorRatio = 1.0;

        /**
         * The number that the reading of the external encoder should be divided by to get the mechanism position.
         * In most cases, this number will stay 1, due to the sensor being mounted directly on the mechanism.
         * But if the sensor has a reduction to the mechanism, this value will be greater than 1.0.
         * (i.e. the sensor spins more than the mechanism)
         */
        public double mechanismToSensorRatio = 1.0;

        /**
         * Copies the current configuration of the external encoder.
         * This method creates a new instance of ExternalEncoderConfig and copies the values from the current instance to the new instance.
         *
         * @return A new ExternalEncoderConfig object with the same values as this instance
         */
        public ExternalEncoderConfig copy() {
            ExternalEncoderConfig copy = new ExternalEncoderConfig();

            copy.useExternalEncoder = this.useExternalEncoder;
            copy.inverted = this.inverted;
            copy.sensorToMotorRatio = this.sensorToMotorRatio;
            copy.mechanismToSensorRatio = this.mechanismToSensorRatio;

            return copy;
        }
    }

    /**
     * Handles the configuration parameters for an absolute encoder
     */
    public static class AbsoluteEncoderConfig {
        /**
         * Should the motor use an absolute encoder (connected directly to the motor controller)?
         * Change this to true if you want to use an absolute encoder.
         * If you want to use also an external encoder (relative encoder), use {@link BasicSparkBaseConfig.ExternalEncoderConfig}.
         */
        public boolean useAbsoluteEncoder = false;

        /**
         * Is the absolute encoder inverted (does it count in the opposite direction of the motor)?
         * Unlike motors, absolute encoders default positive direction is usually clockwise.
         */
        public boolean inverted = false;

        /**
         * The position of the absolute encoder where the mechanism is at it's zero position (in rotations).
         * If changed {@link #absoluteEncoderRange}, adjust this value accordingly.
         * Default is a range of 0.0 to 1.0, so the zero offset is 0.0.
         */
        public double zeroOffset = 0.0;

        /**
         * The number that the reading of the absolute encoder should be multiplied by to get the motor position.
         * In most cases, the sensor will be in a reduction to the motor, so this value will be greater than 1.0.
         * This value will be the same as {@link MotorConfig#gearRatio}.
         */
        public double sensorToMotorRatio = 1.0;

        /**
         * The range that the absolute encoder reads.
         * This affects how the sensor reading is interpreted.
         * Default is {@link AbsoluteEncoderRange#ZERO_TO_ONE} which means the sensor reads from 0.0 to 1.0.
         * If you want the sensor to read from -0.5 to 0.5, use {@link AbsoluteEncoderRange#HALF_REVOLUTION}.
         * Half revolution is useful for a swerve steer motor to get to the zero angle the fastest at the start.
         */
        public AbsoluteEncoderRange absoluteEncoderRange = AbsoluteEncoderRange.ZERO_TO_ONE;

        /**
         * Creates a copy of the absolute encoder configuration.
         * This method creates a new instance of AbsoluteEncoderConfig and copies the values from the current instance to the new instance.
         *
         * @return A new AbsoluteEncoderConfig object with the same values as this instance
         */
        public AbsoluteEncoderConfig copy() {
            AbsoluteEncoderConfig copy = new AbsoluteEncoderConfig();

            copy.useAbsoluteEncoder = this.useAbsoluteEncoder;
            copy.inverted = this.inverted;
            copy.zeroOffset = this.zeroOffset;
            copy.sensorToMotorRatio = this.sensorToMotorRatio;
            copy.absoluteEncoderRange = this.absoluteEncoderRange;

            return copy;
        }

        /**
         * An enum representing the range of the absolute encoder.
         */
        public enum AbsoluteEncoderRange {
            /**
             * 0 to 1 of range.
             */
            ZERO_TO_ONE,
            /**
             * -0.5 to 0.5 of range.
             */
            HALF_REVOLUTION;

            /**
             * Checks if the range is zero centered (i.e. -0.5 to 0.5)
             *
             * @return True if the range is zero centered, false otherwise
             */
            public boolean zeroCentered() {
                return this == HALF_REVOLUTION;
            }
        }
    }
}
