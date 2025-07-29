package com.basicMotor.configuration;

import com.basicMotor.gains.currentLimits.CurrentLimitsTalonSRX;
import com.basicMotor.motors.talonSRX.BasicTalonSRX;

/**
 * This class represents the configuration for a basic TalonSRX motor controller.
 * It extends the BasicMotorConfig class and provides specific configurations for TalonSRX motors.
 */
public class BasicTalonSRXConfig extends BasicMotorConfig{
    /**
     * The encoder configuration for the TalonSRX motor controller.
     * Use this to configure the encoder type and ticks per revolution.
     * The reading of the encoder will also be affected by the gear ratio and unit conversion.
     */
    public EncoderConfig encoderConfig = new EncoderConfig();

    /**
     * The current limits configuration for the TalonSRX motor controller.
     * Use this to protect the motor from overheating and drawing too much current.
     */
    public CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

    /**
     * This class handles the configuration for the encoder.
     * It has encoder types specifically tailored to the TalonSRX motor controllers.
     */
    public static class EncoderConfig {
        /**
         * The type of encoder used with the TalonSRX motor controller.
         */
        public BasicTalonSRX.EncoderType type = BasicTalonSRX.EncoderType.NONE;

        /**
         * The number of ticks per revolution for the encoder.
         * This is used to convert the encoder readings to rotations.
         * The default value is 2048, which is common for many encoders.
         */
        public int tickPerRevolution = 2048;
    }

    /**
     * Handles the configuration for the current limits.
     * It has current limits specifically tailored to the TalonSRX motor controllers.
     */
    public static class CurrentLimitConfig{
        /**
         * The maximum continuous current output of the motor controller (in amps).
         * This is the current that the motor controller will output continuously without overheating.
         */
        public int continuousCurrentLimit = 0;
        /**
         * The maximum peak current output of the motor controller (in amps).
         * This is the current that the motor controller can output for a short duration.
         * If this is not set, the continuous current limit will be used as the peak current limit.
         */
        public int peakCurrentLimit = 0;
        /**
         * The duration for which the peak current limit can be sustained (in seconds).
         * This is the time that the motor can draw the peak current before it is limited to the continuous current limit.
         * If this is not set, the peak current limit will be treated as continuous.
         */
        public int peakCurrentDuration = 0;

        /**
         * Creates a CurrentLimitsTalonSRX instance with the provided current limits.
         * This constructor sets the continuous current limit, peak current limit, and peak current duration.
         * All the current limits are in supply current, not motor current. (Amps)
         * @return The current limits of the TalonSRX motor controller
         */
        public CurrentLimitsTalonSRX toCurrentLimits() {
            return new CurrentLimitsTalonSRX(continuousCurrentLimit, peakCurrentLimit, peakCurrentDuration);
        }
    }
}
