package io.github.captainsoccer.basicmotor.ctre.talonfx;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;

/**
 * This class represents the configuration for a basic TalonFX motor controller.
 * It extends the BasicMotorConfig class and provides specific configurations for TalonFX motors.
 * Use this class when using a TalonFX motor controller.
 * (Falcon 500, Kraken X60, Kraken X44).
 * See the <a href="https://github.com/captainsoccer/BasicMotor/wiki/Usage-TalonFX">wiki</a>
 * for more information on how to use this class.
 */
public class BasicTalonFXConfig extends BasicMotorConfig {
    /**
     * The current limits configuration for the TalonFX motor controller.
     * Use this to protect the motor from overheating and drawing too much current.
     */
    public CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

    /**
     * The configuration for the canCoder.
     * Use this to create automatically a canCoder for the motor.
     * you can access the created canCoder with the {@link BasicTalonFX#getActiveCanCoder()}.
     */
    public CanCoderConfig canCoderConfig = new CanCoderConfig();

    /**
     * The name of the CAN bus that the TalonFX motor controller is connected to.
     * This is only useful when using a canivore and the motor is connected to a canivore.
     * Otherwise, do not change this value.
     */
    public CANBus canBus = CANBus.roboRIO();

    /**
     * This enables the Field Oriented Control (FOC) for the TalonFX motor controller.
     * This is supported only on the TalonFX, with a licensed phoenix pro firmware.
     * For more information on FOC, see the FOC section in the
     * <a href="https://v6.docs.ctr-electronics.com/en/latest/docs/migration/new-to-phoenix.html">phoenix documentation</a>.
     */
    public boolean enableFOC = false;

    /**
     * This flag determines whether the measurements will wait for all signals to update before returning values.
     * This will work only if the device is connected to a CANivore and the Pro features are enabled.
     * Otherwise, it will be ignored.
     */
    public boolean waitForAllSignals = false;

    @Override
    public BasicTalonFXConfig copy() {
        var copy = new BasicTalonFXConfig();

        // Copy the basic motor configuration
        super.copy(copy);

        copy.currentLimitConfig = this.currentLimitConfig.copy();
        copy.canCoderConfig = canCoderConfig.copy();
        copy.canBus = this.canBus;
        copy.enableFOC = this.enableFOC;
        copy.waitForAllSignals = this.waitForAllSignals;

        return copy;
    }

    /**
     * Handles the configuration for the current limits.
     */
    public static class CurrentLimitConfig {
        /**
         * The maximum current output of the motor controller (in amps).
         * This is different from the supply current limit, and will usually be higher.
         * This can be used to limit the force output of the motor, to prevent damaging the mechanism.
         */
        public int statorCurrentLimit = 0;
        /**
         * The maximum current draw of the motor controller (in amps).
         * This is the current that the motor controller will draw from the battery.
         * If the motor draws this amount of current for more then {@link #lowerLimitTime} seconds,
         * it will lower to {@link #lowerCurrentLimit}.
         * Use this if there are brownouts or breakers tripping.
         * Otherwise, it is recommended to use {@link #statorCurrentLimit} instead.
         */
        public int supplyCurrentLimit = 0;
        /**
         * The time (in seconds) that the motor controller will stay in the supply current limit before lowering to the lower current limit.
         * This is useful to prevent the motor from overheating and tripping breakers.
         */
        public double lowerLimitTime = 0;
        /**
         * The current the motor drops to after the supply current limit is reached for {@link #lowerLimitTime}.
         * Works only if {@link #supplyCurrentLimit} and {@link #lowerLimitTime} is set.
         */
        public int lowerCurrentLimit = 0;

        /**
         * Creates the current limit configuration with the given values
         *
         * @return The current limits of the motor controller
         */
        public TalonFXCurrentLimits getCurrentLimits() {
            return new TalonFXCurrentLimits(
                    statorCurrentLimit, supplyCurrentLimit, lowerLimitTime, lowerCurrentLimit);
        }

        /**
         * Sets the current limits of the motor controller from the given current limits.
         *
         * @param currentLimits The current limits to set
         */
        public void fromCurrentLimits(TalonFXCurrentLimits currentLimits) {
            this.statorCurrentLimit = currentLimits.getCurrentLimit();
            this.supplyCurrentLimit = currentLimits.supplyCurrentLimit();
            this.lowerLimitTime = currentLimits.supplyLowerTime();
            this.lowerCurrentLimit = currentLimits.supplyLowerLimit();
        }

        /**
         * Creates a copy of the current limit configuration.
         *
         * @return A new CurrentLimitConfig object with the same values
         */
        public CurrentLimitConfig copy() {
            var copy = new CurrentLimitConfig();

            copy.statorCurrentLimit = this.statorCurrentLimit;
            copy.supplyCurrentLimit = this.supplyCurrentLimit;
            copy.lowerLimitTime = this.lowerLimitTime;
            copy.lowerCurrentLimit = this.lowerCurrentLimit;

            return copy;
        }
    }

    /**
     * The config for the CanCoder that can be used with a talonFX directly
     */
    public static class CanCoderConfig{
        /**
         * The type of canCoder to use.
         * If no using a canCoder leave at the default RotorSensor.
         * Some types may require phoenix pro
         */
        public FeedbackSensorSourceValue canCoderType = FeedbackSensorSourceValue.RotorSensor;

        /**
         * The CAN ID of the canCoder to use
         */
        public int canCoderID = 0;
        /**
         * The zero offset of the canCoder.
         * This is the value subtracted from the canCoder raw position
         */
        public double zeroOffset= 0;
        /**
         * The discontinuity of the canCoder.
         * This is where the absolute position reading jumps from its max value to it's minimum value.
         * The default is 0: which means the canCoder will read values between 0 to 1 and jump from 1 to zero.
         * recommended value for swerve azimuth is 0.5, which means the canCoder will read between -0.5 to 0.5.
         */
        public double canCoderDiscontinuityPoint = 0;

        /**
         * The canBus the canCoder is connected to.
         * The Cancoder and talonFX must be on the same can bus in order for it to work.
         * if not using a canivore leave it at the default value of roboRio.
         */
        public CANBus canCoderCanBus = CANBus.roboRIO();

        /**
         * The ratio between the sensor reading and the motor.
         * This value is multiplied by the encoder reading to get the reading of the motor.
         * A value greater than 1 represent a reduction (the canCoder spins more than the motor)
         */
        public double sensorToMotorRatio = 1;
        /**
         * The ratio between the mechanism to the sensor.
         * this value will be the divisor for the sensor reading in order to get the mechanism reading.
         * A value greater than 1 represents a reduction (the mechanism spins less than the sensor).
         */
        public double mechanismToSensorRatio = 1;

        /**
         * Copies all the value in this configuration to a new configuration
         * @return the new configuration
         */
        public CanCoderConfig copy(){
            var copy = new CanCoderConfig();

            copy.canCoderType = canCoderType;
            copy.canCoderID = canCoderID;
            copy.zeroOffset = zeroOffset;
            copy.canCoderDiscontinuityPoint = canCoderDiscontinuityPoint;
            copy.canCoderCanBus = canCoderCanBus;
            copy.sensorToMotorRatio = sensorToMotorRatio;
            copy.mechanismToSensorRatio = mechanismToSensorRatio;

            return copy;
        }
    }
}
